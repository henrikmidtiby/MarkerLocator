# -*- coding: utf-8 -*-
"""
Window mode marker tracker.

@author: Henrik Skov Midtiby
"""
import cv
from MarkerTracker import MarkerTracker
import math
from MarkerPose import MarkerPose

class TrackerInWindowMode:
    def __init__(self, order = 7, defaultKernelSize = 21):
        #cv.NamedWindow('reducedWindow', cv.CV_WINDOW_AUTOSIZE)
        self.windowWidth = 100
        self.windowHeight = 100
        self.frameGray = cv.CreateImage ((self.windowWidth, self.windowHeight), cv.IPL_DEPTH_32F, 1)
        self.originalImage = cv.CreateImage((self.windowWidth, self.windowHeight), cv.IPL_DEPTH_32F, 3)
        self.markerTracker = MarkerTracker(order, defaultKernelSize, 2500)
        self.trackerIsInitialized = False
        self.subImagePosition = None
        pass
    
    def cropFrame(self, frame, lastMarkerLocationX, lastMarkerLocationY):
        if(not self.trackerIsInitialized):
            self.markerTracker.allocateSpaceGivenFirstFrame(self.originalImage)
            self.reducedImage = cv.CreateImage((self.windowWidth, self.windowHeight), frame.depth, 3)
        xCornerPos = lastMarkerLocationX - self.windowWidth / 2
        yCornerPos = lastMarkerLocationY - self.windowHeight / 2
        # Ensure that extracted window is inside the original image.
        if(xCornerPos < 1):
            xCornerPos = 1
        if(yCornerPos < 1):
            yCornerPos = 1
        if(xCornerPos > frame.width - self.windowWidth):
            xCornerPos = frame.width - self.windowWidth
        if(yCornerPos > frame.height - self.windowHeight):
            yCornerPos = frame.height - self.windowHeight
        try:
            self.subImagePosition = (xCornerPos, yCornerPos, self.windowWidth, self.windowHeight)
            self.reducedImage = cv.GetSubRect(frame, self.subImagePosition)
            cv.ConvertScale(self.reducedImage, self.originalImage)
            cv.CvtColor(self.originalImage, self.frameGray, cv.CV_RGB2GRAY)
        except:
            print("frame: ", frame.depth)
            print("originalImage: ", self.originalImage.height, self.originalImage.width, self.originalImage)
            print("frameGray: ", self.frameGray.height, self.frameGray.width, self.frameGray.depth)
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
        cv.Circle(self.reducedImage, (xm, ym), 4, redColor, 2)
        xm2 = int(xm + 50*math.cos(orientation))
        ym2 = int(ym + 50*math.sin(orientation))
        cv.Line(self.reducedImage, (xm, ym), (xm2, ym2), blueColor, 2)

        
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
        
    
