# -*- coding: utf-8 -*-
"""
Image analyzer class for talking with the MarkerTracker.

@author: Henrik Skov Midtiby
"""

import cv2
import numpy as np

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
        reducedWidth = frame.shape[1] / self.downscaleFactor
        reducedHeight = frame.shape[0] / self.downscaleFactor
        reducedDimensions = (reducedHeight, reducedWidth)
        self.frameGray = np.zeros (reducedDimensions+(1,), dtype=np.float32)
        self.frameGray = np.zeros (reducedDimensions+(1,), dtype=np.float32)
        
        self.originalImage = np.zeros (reducedDimensions+(3,), dtype=np.float32)
        self.reducedImage = np.zeros (reducedDimensions+(frame.shape[2],), dtype=np.float32)
        for k in range(len(self.markerTrackers)):
            self.markerTrackers[k].allocateSpaceGivenFirstFrame(self.reducedImage)
    
    # Is called with a colour image.
    def analyzeImage(self, frame):
        assert(frame.shape[2] == 3)
        if(self.subClassesInitialized is False):
            self.initializeSubClasses(frame)

        self.reducedImage = cv2.resize(frame,(0,0), fx=1.0/self.downscaleFactor, fy=1.0/self.downscaleFactor)

        self.originalImage=self.reducedImage
        #cv.ConvertScale(self.reducedImage, self.originalImage)
        self.frameGray=cv2.cvtColor(self.originalImage,cv2.cv.CV_RGB2GRAY)

        for k in range(len(self.markerTrackers)):
            markerLocation = self.markerTrackers[k].locateMarker(self.frameGray)
            (xm, ym) = markerLocation
            (xm, ym) = (self.downscaleFactor * xm, self.downscaleFactor * ym)
            self.markerLocationsX[k] = xm
            self.markerLocationsY[k] = ym
            #cv.Line(frame, (0, ym), (frame.width, ym), (0, 0, 255)) # B, G, R
            #cv.Line(frame, (xm, 0), (xm, frame.height), (0, 0, 255))

        return frame
