# -*- coding: utf-8 -*-
"""
Marker tracker for locating n-fold edges in images using convolution.

@author: Henrik Skov Midtiby
"""
import cv2
import numpy as np
import math
from .MarkerPose import MarkerPose


class MarkerTracker:
    """
    Purpose: Locate a certain marker in an image.
    """

    def __init__(self, order, kernel_size, scale_factor):
        self.kernel_size = kernel_size
        (kernel_real, kernel_imag) = self.generate_symmetry_detector_kernel(order, kernel_size)

        self.order = order
        self.mat_real = kernel_real / scale_factor
        self.mat_imag = kernel_imag / scale_factor

        self.frame_real = None
        self.frame_imag = None
        self.last_marker_location = None
        self.orientation = None
        self.track_marker_with_missing_black_leg = True

        # Create kernel used to remove arm in quality-measure
        (kernel_remove_arm_real, kernel_remove_arm_imag) = self.generate_symmetry_detector_kernel(1, self.kernel_size)
        self.kernelComplex = np.array(kernel_real + 1j*kernel_imag, dtype=complex)
        self.KernelRemoveArmComplex = np.array(kernel_remove_arm_real + 1j*kernel_remove_arm_imag, dtype=complex)

        # Values used in quality-measure
        absolute = np.absolute(self.kernelComplex)
        self.threshold = 0.4*absolute.max()
        self.quality = None
        self.y1 = int(math.floor(float(self.kernel_size)/2))
        self.y2 = int(math.ceil(float(self.kernel_size)/2))
        self.x1 = int(math.floor(float(self.kernel_size)/2))
        self.x2 = int(math.ceil(float(self.kernel_size)/2))

        # Information about the located marker.
        self.pose = None

    @staticmethod
    def generate_symmetry_detector_kernel(order, kernel_size):
        # type: (int, int) -> numpy.ndarray
        value_range = np.linspace(-1, 1, kernel_size)
        temp1 = np.meshgrid(value_range, value_range)
        kernel = temp1[0] + 1j * temp1[1]

        magnitude = abs(kernel)
        kernel = np.power(kernel, order)
        kernel = kernel * np.exp(-8 * magnitude ** 2)

        return np.real(kernel), np.imag(kernel)

    def refine_marker_location(self):
        try: 
            delta = 1
            # Fit a parabola to the frame_sum_squared marker response
            # and then locate the top of the parabola.
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(self.frame_sum_squared)
            x = max_loc[1]
            y = max_loc[0]
            frame_sum_squared_cutout = self.frame_sum_squared[x-delta:x+delta+1, y-delta:y+delta+1]
            # Taking the square root of the frame_sum_squared improves the accuracy of the 
            # refied marker position.
            frame_sum_squared_cutout = np.sqrt(frame_sum_squared_cutout)

            nx, ny = (1 + 2*delta, 1 + 2*delta)
            x = np.linspace(-delta, delta, nx)
            y = np.linspace(-delta, delta, ny)
            xv, yv = np.meshgrid(x, y)

            xv = xv.ravel()
            yv = yv.ravel()

            coefficients = np.concatenate([[xv**2], [xv], [yv**2], [yv], [yv**0]], axis = 0).transpose()
            values = frame_sum_squared_cutout.ravel().reshape(-1, 1)
            solution, residuals, rank, s = np.linalg.lstsq(coefficients, values, rcond=None)
            dx = -solution[1] / (2*solution[0])
            dy = -solution[3] / (2*solution[2])
            return dx[0], dy[0]
        except np.linalg.LinAlgError:
            # This error is triggered when the marker is detected close to an edge.
            # In that case the refine method bails out and returns two zeros.
            return 0, 0

    def locate_marker(self, frame):
        assert len(frame.shape) == 2, "Input image is not a single channel image."
        self.frame_real = frame.copy()
        self.frame_imag = frame.copy()

        # Calculate convolution and determine response strength.
        self.frame_real = cv2.filter2D(self.frame_real, cv2.CV_32F, self.mat_real)
        self.frame_imag = cv2.filter2D(self.frame_imag, cv2.CV_32F, self.mat_imag)
        frame_real_squared = cv2.multiply(self.frame_real, self.frame_real, dtype=cv2.CV_32F)
        frame_imag_squared = cv2.multiply(self.frame_imag, self.frame_imag, dtype=cv2.CV_32F)
        self.frame_sum_squared = cv2.add(frame_real_squared, frame_imag_squared, dtype=cv2.CV_32F)

        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(self.frame_sum_squared)
        self.last_marker_location = max_loc
        self.determine_marker_orientation(frame)
        self.determine_marker_quality(frame)
        dx, dy = self.refine_marker_location()
        #print(f"dx: {dx: 0.2f}  dy: {dy: 0.2f}")
        max_loc = (max_loc[0] + dx, max_loc[1] + dy)

        self.pose = MarkerPose(max_loc[0], max_loc[1], self.orientation, self.quality, self.order)
        return self.pose

    def determine_marker_orientation(self, frame):
        (xm, ym) = self.last_marker_location
        real_value = self.frame_real[ym, xm]
        imag_value = self.frame_imag[ym, xm]
        self.orientation = (math.atan2(-real_value, imag_value) - math.pi / 2) / self.order

        max_value = 0
        max_orientation = self.orientation
        search_distance = self.kernel_size / 3
        for k in range(self.order):
            orient = self.orientation + 2 * k * math.pi / self.order
            xm2 = int(xm + search_distance * math.cos(orient))
            ym2 = int(ym + search_distance * math.sin(orient))
            try:
                intensity = frame[ym2, xm2]
                if intensity > max_value:
                    max_value = intensity
                    max_orientation = orient
            except Exception as e:
                print("determineMarkerOrientation: error: %d %d %d %d" % (ym2, xm2, frame.shape[1], frame.shape[0]))
                print(e)
                pass

        self.orientation = self.limit_angle_to_range(max_orientation)

    @staticmethod
    def limit_angle_to_range(angle):
        while angle < math.pi:
            angle += 2 * math.pi
        while angle > math.pi:
            angle -= 2 * math.pi
        return angle

    def determine_marker_quality(self, frame):
        (bright_regions, dark_regions) = self.generate_template_for_quality_estimator()
        # cv2.imshow("bright_regions", 255*bright_regions)
        # cv2.imshow("dark_regions", 255*dark_regions)

        try:
            frame_img = self.extract_window_around_maker_location(frame)
            (bright_mean, bright_std) = cv2.meanStdDev(frame_img, mask=bright_regions)
            (dark_mean, dark_std) = cv2.meanStdDev(frame_img, mask=dark_regions)

            mean_difference = bright_mean - dark_mean
            normalised_mean_difference = mean_difference / (0.5*bright_std + 0.5*dark_std)
            # Ugly hack for translating the normalised_mean_differences to the range [0, 1]
            temp_value_for_quality = 1 - 1/(1 + math.exp(0.75*(-7+normalised_mean_difference)))
            self.quality = temp_value_for_quality
        except Exception as e:
            print("error")
            print(e)
            self.quality = 0.0
            return

    def extract_window_around_maker_location(self, frame):
        (xm, ym) = self.last_marker_location
        frame_tmp = np.array(frame[ym - self.y1:ym + self.y2, xm - self.x1:xm + self.x2])
        frame_img = frame_tmp.astype(np.uint8)
        return frame_img

    def generate_template_for_quality_estimator(self):
        phase = np.exp((self.limit_angle_to_range(-self.orientation)) * 1j)
        angle_threshold = 3.14 / (2 * self.order)
        t3 = np.angle(self.KernelRemoveArmComplex * phase) < angle_threshold
        t4 = np.angle(self.KernelRemoveArmComplex * phase) > -angle_threshold

        signed_mask = 1 - 2 * (t3 & t4)
        adjusted_kernel = self.kernelComplex * np.power(phase, self.order)
        if self.track_marker_with_missing_black_leg:
            adjusted_kernel *= signed_mask
        bright_regions = (adjusted_kernel.real < -self.threshold).astype(np.uint8)
        dark_regions = (adjusted_kernel.real > self.threshold).astype(np.uint8)

        return bright_regions, dark_regions
