# -*- coding: utf-8 -*-
"""
Image analyzer class for talking with the MarkerTracker.

@author: Henrik Skov Midtiby
"""

import cv

from MarkerTracker import MarkerTracker

class ImageAnalyzer:
    '''
    Purpose: Locate markers in the presented images.
    '''
    def __init__(self, downscaleFactor = 2):
        self.downscaleFactor = downscaleFactor
        self.markerTrackers = []
        self.tempImage = None
        self.greyScaleImage = None
        self.subClassesInitialized = False
        self.markerLocationsX = []
        self.markerLocationsY = []
        pass

    def addMarkerToTrack(self, order, kernelSize, scaleFactor):
        self.markerTrackers.append(MarkerTracker(order, kernelSize, scaleFactor))
        self.markerLocationsX.append(0)
        self.markerLocationsY.append(0)
        self.subClassesInitialized = False 
    
    # Is called with a colour image.
    def initializeSubClasses(self, frame):
        self.subClassesInitialized = True
        reducedWidth = frame.width / self.downscaleFactor
        reducedHeight = frame.height / self.downscaleFactor
        reducedDimensions = (reducedWidth, reducedHeight)
        self.frameGray = cv.CreateImage (reducedDimensions, cv.IPL_DEPTH_32F, 1)
        self.originalImage = cv.CreateImage(reducedDimensions, cv.IPL_DEPTH_32F, 3)
        self.reducedImage = cv.CreateImage(reducedDimensions, frame.depth, frame.nChannels)
        for k in range(len(self.markerTrackers)):
            self.markerTrackers[k].allocateSpaceGivenFirstFrame(self.reducedImage)
    
    # Is called with a colour image.
    def analyzeImage(self, frame):
        assert(frame.nChannels == 3)
        if(self.subClassesInitialized is False):
            self.initializeSubClasses(frame)

        cv.Resize(frame, self.reducedImage)

        cv.ConvertScale(self.reducedImage, self.originalImage)
        cv.CvtColor(self.originalImage, self.frameGray, cv.CV_RGB2GRAY)

        for k in range(len(self.markerTrackers)):
            markerLocation = self.markerTrackers[k].locateMarker(self.frameGray)
            (xm, ym) = markerLocation
            (xm, ym) = (self.downscaleFactor * xm, self.downscaleFactor * ym)
            self.markerLocationsX[k] = xm
            self.markerLocationsY[k] = ym
            #cv.Line(frame, (0, ym), (frame.width, ym), (0, 0, 255)) # B, G, R
            #cv.Line(frame, (xm, 0), (xm, frame.height), (0, 0, 255))

        return frame