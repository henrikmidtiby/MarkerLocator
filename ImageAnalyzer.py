# -*- coding: utf-8 -*-
"""
Image analyzer class for talking with the MarkerTracker.

@author: Henrik Skov Midtiby
"""

import cv2
import numpy as np

from MarkerTracker import MarkerTracker


class ImageAnalyzer:
    """
    Purpose: Locate markers in the presented images.
    """
    def __init__(self, downscale_factor=2):
        self.downscale_factor = downscale_factor
        self.markerTrackers = []
        pass

    def add_marker_to_track(self, order, kernel_size, scale_factor):
        self.markerTrackers.append(MarkerTracker(order, kernel_size, scale_factor))

    def analyze_image(self, frame):
        reduced_image = cv2.resize(frame, (0,0), fx=1.0/self.downscale_factor, fy=1.0 / self.downscale_factor)
        original_image = reduced_image
        frame_gray = cv2.cvtColor(original_image, cv2.cv.CV_RGB2GRAY)

        for k in range(len(self.markerTrackers)):
            self.markerTrackers[k].locate_marker(frame_gray)
            self.markerTrackers[k].pose.scale_position(self.downscale_factor)

        return frame
