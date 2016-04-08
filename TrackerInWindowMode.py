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
    def __init__(self, order = 7, defaultKernelSize = 21):
        #cv.NamedWindow('reducedWindow', cv.CV_WINDOW_AUTOSIZE)
        self.windowWidth = 100
        self.windowHeight = 100
        self.frameGray = np.zeros((self.windowHeight, self.windowWidth,1), dtype=np.float32)
        self.originalImage = np.zeros((self.windowHeight, self.windowWidth,3), dtype=np.float32)
        self.markerTracker = MarkerTracker(order, defaultKernelSize, 2500)
        self.trackerIsInitialized = False
        self.subImagePosition = None
        pass
    
    def cropFrame(self, frame, lastMarkerLocationX, lastMarkerLocationY):
        if(not self.trackerIsInitialized):
            self.markerTracker.allocateSpaceGivenFirstFrame(self.originalImage)
            self.reducedImage = np.zeros((self.windowHeight, self.windowWidth,3), frame.dtype)
        xCornerPos = lastMarkerLocationX - self.windowWidth / 2
        yCornerPos = lastMarkerLocationY - self.windowHeight / 2
        # Ensure that extracted window is inside the original image.
        if(xCornerPos < 1):
            xCornerPos = 1
        if(yCornerPos < 1):
            yCornerPos = 1
        if(xCornerPos > frame.shape[1] - self.windowWidth):
            xCornerPos = frame.shape[1] - self.windowWidth
        if(yCornerPos > frame.shape[0] - self.windowHeight):
            yCornerPos = frame.shape[0] - self.windowHeight
        try:
            self.subImagePosition = (xCornerPos, yCornerPos, self.windowWidth, self.windowHeight)
            #self.reducedImage = cv.GetSubRect(frame, self.subImagePosition)
            self.reducedImage = frame[self.subImagePosition[1]:self.subImagePosition[1]+self.subImagePosition[3] , self.subImagePosition[0]:self.subImagePosition[0]+self.subImagePosition[2], :]
            #cv.ConvertScale(self.reducedImage, self.originalImage)
            self.frameGray = cv2.cvtColor(self.reducedImage, cv2.cv.CV_RGB2GRAY)

        except:
            print("frame: ", frame.dtype)
            print("originalImage: ", self.originalImage.shape[0], self.originalImage.shape[1], self.originalImage)
            print("frameGray: ", self.frameGray.shape[0], self.frameGray.shape[1], self.frameGray.dtype)
            print "Unexpected error:", sys.exc_info()[0]
            #quit(0)
            pass
        
    def locateMarker(self):
        (xm, ym) = self.markerTracker.locateMarker(self.frameGray)
        #xm = 50
        #ym = 50
        #cv.Line(self.reducedImage, (0, ym), (self.originalImage.width, ym), (0, 0, 255)) # B, G, R
        #cv.Line(self.reducedImage, (xm, 0), (xm, self.originalImage.height), (0, 0, 255))

        redColor = (55, 55, 255)
        blueColor = (255, 0, 0)

        orientation = self.markerTracker.orientation
        cv2.circle(self.reducedImage, (xm, ym), 4, redColor, 2)
        xm2 = int(xm + 50*math.cos(orientation))
        ym2 = int(ym + 50*math.sin(orientation))
        cv2.line(self.reducedImage, (xm, ym), (xm2, ym2), blueColor, 2)

        
        xm = xm + self.subImagePosition[0]
        ym = ym + self.subImagePosition[1]
        #print((xm, ym))
        
        #return [xm, ym, orientation, self.markerTracker.quality]
        return MarkerPose(xm, ym, orientation, self.markerTracker.quality, self.markerTracker.order)        
        
    def showCroppedImage(self):
        pass
        #cv.ShowImage('reducedWindow', self.reducedImage)
        #cv.ShowImage('reducedWindow', self.originalImage)
        #cv.ShowImage('reducedWindow', self.frameGray)
        #cv.ShowImage('reducedWindow', self.markerTracker.frameSumSq)
        
    
