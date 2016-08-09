# -*- coding: utf-8 -*-
"""
Window mode marker tracker.

@author: Henrik Skov Midtiby
"""
from MarkerTracker import MarkerTracker
import math
import cv2
from MarkerPose import MarkerPose
import numpy as np
import sys


class TrackerInWindowMode:
    def __init__(self, order=7, defaultKernelSize=21):
        self.window_width = 100
        self.window_height = 100
        self.frameGray = np.zeros((self.window_height, self.window_width, 1), dtype=np.float32)
        self.originalImage = np.zeros((self.window_height, self.window_width, 3), dtype=np.float32)
        self.markerTracker = MarkerTracker(order, defaultKernelSize, 2500)
        self.trackerIsInitialized = False
        self.subImagePosition = None
        self.reducedImage = None

    def crop_frame(self, frame, last_marker_location_x, last_marker_location_y):
        if not self.trackerIsInitialized:
            self.reducedImage = np.zeros((self.window_height, self.window_width, 3), frame.dtype)
        x_corner_pos = last_marker_location_x - self.window_width / 2
        y_corner_pos = last_marker_location_y - self.window_height / 2
        # Ensure that extracted window is inside the original image.
        if x_corner_pos < 1:
            x_corner_pos = 1
        if y_corner_pos < 1:
            y_corner_pos = 1
        if x_corner_pos > frame.shape[1] - self.window_width:
            x_corner_pos = frame.shape[1] - self.window_width
        if y_corner_pos > frame.shape[0] - self.window_height:
            y_corner_pos = frame.shape[0] - self.window_height
        try:
            self.subImagePosition = (x_corner_pos, y_corner_pos, self.window_width, self.window_height)
            self.reducedImage = frame[self.subImagePosition[1]:self.subImagePosition[1]+self.subImagePosition[3],
                                      self.subImagePosition[0]:self.subImagePosition[0]+self.subImagePosition[2],
                                      :]
            self.frameGray = cv2.cvtColor(self.reducedImage, cv2.cv.CV_RGB2GRAY)
        except:
            print("frame: ", frame.dtype)
            print("originalImage: ", self.originalImage.shape[0], self.originalImage.shape[1], self.originalImage)
            print("frameGray: ", self.frameGray.shape[0], self.frameGray.shape[1], self.frameGray.dtype)
            print "Unexpected error:", sys.exc_info()[0]
            pass
        
    def locate_marker(self):
        (xm, ym) = self.markerTracker.locate_marker(self.frameGray)

        red_color = (55, 55, 255)
        blue_color = (255, 0, 0)

        orientation = self.markerTracker.orientation
        cv2.circle(self.reducedImage, (xm, ym), 4, red_color, 2)
        xm2 = int(xm + 50*math.cos(orientation))
        ym2 = int(ym + 50*math.sin(orientation))
        cv2.line(self.reducedImage, (xm, ym), (xm2, ym2), blue_color, 2)

        xm = xm + self.subImagePosition[0]
        ym = ym + self.subImagePosition[1]

        return MarkerPose(xm, ym, orientation, self.markerTracker.quality, self.markerTracker.order)
        
    def show_cropped_image(self):
        pass
