import cv2
import numpy as np
import MarkerLocator.MarkerTracker as MarkerTracker


def main():
    image_of_real_and_imag_part_of_kernel()
    image_of_quality_templates()
    hubsan_image = cv2.imread('input/hubsanwithmarker.jpg')
    hubsan_image_small =cv2.resize(hubsan_image, None, fx=0.1, fy=0.1)
    cv2.imwrite("hubsanwithmarker_small.png", hubsan_image_small)
    tracker = MarkerTracker.MarkerTracker(order=4,
                                          kernel_size=91,
                                          scale_factor=0.01)
    tracker.track_marker_with_missing_black_leg = False
    gray_scale_image = cv2.cvtColor(hubsan_image_small, cv2.COLOR_BGR2GRAY)
    tracker.locate_marker(gray_scale_image)
    magnitude = np.sqrt(tracker.frame_sum_squared)
    argument = np.arctan2(tracker.frame_imag, tracker.frame_real)
    maximum_magnitude = np.max(magnitude)
    cv2.imwrite("hubsan_magnitude_response_n4_kernel.png",
                magnitude/maximum_magnitude * 255)
    cv2.imwrite("hubsan_magnitude_response_inverted_n4_kernel.png",
                (1-magnitude/maximum_magnitude) * 255)
    cv2.imwrite("hubsan_argument_response_n4_kernel.png",
                (3.1415 + argument) / 7 * 255)

    hue = (argument + 3.1415) / (np.pi) * 180
    saturation = 1 + 0*magnitude / maximum_magnitude
    value = np.sqrt(magnitude / maximum_magnitude) * 255
    hsv_image = cv2.merge((hue, saturation, value))
    rgb_image = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR)
    cv2.imwrite("hubsan_magnutude_and_argument_as_hsv_n4_kernel.png",
                rgb_image)
    


    region_around_detected_marker = tracker.extract_window_around_maker_location(gray_scale_image)
    cv2.imwrite("hubsan_region_around_detected_marker.png",
                region_around_detected_marker)

    (bright_regions, dark_regions) = tracker.generate_template_for_quality_estimator()
    quality_template = (0.5 + bright_regions - dark_regions)
    cv2.imwrite("hubsan_oriented_quality_template_oriented.png", quality_template * 255)

    merged_input_and_quality_template = cv2.addWeighted(region_around_detected_marker.astype('float64'), 0.5,
                                                        quality_template*255, 0.5,
                                                        1)
    cv2.imwrite("hubsan_merged_input_and_oriented_quality_template_oriented.png", merged_input_and_quality_template)


def image_of_real_and_imag_part_of_kernel():
    tracker = MarkerTracker.MarkerTracker(order=4,
                                          kernel_size=101,
                                          scale_factor=1)
    maximum_value = np.max(tracker.mat_real)
    real_part_of_kernel = 0.5 * tracker.mat_real / maximum_value + 0.5
    imaginary_part_of_kernel = 0.5 * tracker.mat_imag / maximum_value + 0.5
    kernel_magnitude = np.absolute(tracker.kernelComplex) / maximum_value
    kernel_argument = np.angle(tracker.kernelComplex).astype('float32')
    kernel_argument_hsv = np.dstack((kernel_argument*180/3.1415926535 + 180,
                                     kernel_argument*0 + 1,
                                     kernel_argument*0 + 255.))
    kernel_argument_rgb = cv2.cvtColor(kernel_argument_hsv, cv2.COLOR_HSV2BGR)
    print(np.max(kernel_argument_rgb))
    cv2.imwrite("kernel_real_part.png", real_part_of_kernel * 255)
    cv2.imwrite("kernel_imaginary_part.png", imaginary_part_of_kernel * 255)
    cv2.imwrite("kernel_magnitude.png", kernel_magnitude * 255)
    cv2.imwrite("kernel_argument.png", kernel_argument_rgb)


def image_of_quality_templates():
    tracker = MarkerTracker.MarkerTracker(order=4,
                                          kernel_size=101,
                                          scale_factor=0.01)
    # Set orientation of the marker
    tracker.orientation = 0.5
    (bright_regions, dark_regions) = tracker.generate_template_for_quality_estimator()
    cv2.imwrite("quality_template_oriented_white_regions.png", bright_regions * 255)
    cv2.imwrite("quality_template_oriented.png", (0.5 + bright_regions - dark_regions) * 255)
    cv2.imwrite("quality_template_oriented_black_regions.png", dark_regions * 255)
    # Set orientation of the marker
    tracker.track_marker_with_missing_black_leg = False
    tracker.orientation = 0.5
    (bright_regions, dark_regions) = tracker.generate_template_for_quality_estimator()
    cv2.imwrite("quality_template_plain_white_regions.png", bright_regions * 255)
    cv2.imwrite("quality_template_plain.png", (0.5 + bright_regions - dark_regions) * 255)
    cv2.imwrite("quality_template_plain_black_regions.png", dark_regions * 255)


main()
