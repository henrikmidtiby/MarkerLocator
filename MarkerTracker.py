# -*- coding: utf-8 -*-
"""
Marker tracker for locating n-fold edges in images using convolution.

@author: Henrik Skov Midtiby
"""
import cv2
import numpy as np
import math


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

    def locate_marker(self, frame):
        self.frame_real = frame.copy()
        self.frame_imag = frame.copy()

        # Calculate convolution and determine response strength.
        self.frame_real = cv2.filter2D(self.frame_real, cv2.CV_32F, self.mat_real)
        self.frame_imag = cv2.filter2D(self.frame_imag, cv2.CV_32F, self.mat_imag)
        frame_real_squared = cv2.multiply(self.frame_real, self.frame_real, dtype=cv2.CV_32F)
        frame_imag_squared = cv2.multiply(self.frame_imag, self.frame_imag, dtype=cv2.CV_32F)
        frame_sum_squared = cv2.add(frame_real_squared, frame_imag_squared, dtype=cv2.CV_32F)

        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(frame_sum_squared)
        self.last_marker_location = max_loc
        self.determine_marker_orientation(frame)
        self.determine_marker_quality(frame)

        return max_loc

    def determine_marker_orientation(self, frame):
        (xm, ym) = self.last_marker_location
        real_value = self.frame_real[ym, xm]
        imag_value = self.frame_imag[ym, xm]
        self.orientation = (math.atan2(-real_value, imag_value) - math.pi / 2) / self.order

        max_value = 0
        max_orientation = 0
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
        template = self.generate_template_for_quality_estimator()
        try:
            frame_img = self.extract_window_around_maker_location(frame)
            frame_w, frame_h = frame_img.shape
            template = template[0:frame_h, 0:frame_w].astype(np.uint8)

            # For the quality estimator cv2.TM_CCORR_NORMED shows best results.
            quality_match = cv2.matchTemplate(frame_img, template, cv2.TM_CCORR_NORMED)
            self.quality = quality_match[0, 0]
        except Exception as e:
            print "error"
            print e
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
        t1 = (self.kernelComplex * np.power(phase, self.order)).real > self.threshold
        t2 = (self.kernelComplex * np.power(phase, self.order)).real < -self.threshold
        img_t1_t2_diff = t1.astype(np.float32) - t2.astype(np.float32)
        t3 = np.angle(self.KernelRemoveArmComplex * phase) < angle_threshold
        t4 = np.angle(self.KernelRemoveArmComplex * phase) > -angle_threshold
        mask = 1 - 1 * (t3 & t4)
        template = ((1 - img_t1_t2_diff * mask) * 255).astype(np.uint8)
        return template
