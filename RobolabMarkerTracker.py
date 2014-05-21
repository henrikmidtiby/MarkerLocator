# -*- coding: utf-8 -*-
"""
Created on Wed May 21 09:42:51 2014

@author: henrik
"""

#!/usr/bin/env python
from time import time, strftime
import sys
import os

sys.path.append('/opt/ros/hydro/lib/python2.7/dist-packages')
import cv
import math

from ImageAnalyzer import ImageAnalyzer
from PerspectiveTransform import PerspectiveCorrecter
from MarkerPose import MarkerPose

'''
2012-10-10
Script developed by Henrik Skov Midtiby (henrikmidtiby@gmail.com).
Provided for free but use at your own risk.

2013-02-13 
Structural changes allows simultaneous tracking of several markers.
Frederik Hagelskjaer added code to publish marker locations to ROS.
'''
    
class CameraDriver:
    ''' 
    Purpose: capture images from a camera and delegate procesing of the 
    images to a different class.
    '''
    def __init__(self, markerOrders = [7, 8], defaultKernelSize = 21, scalingParameter = 2500):
        # Initialize camera driver.
        # Open output window.
        cv.NamedWindow('filterdemo', cv.CV_WINDOW_AUTOSIZE)

        self.setFocus()
        # Select the camera where the images should be grabbed from.
        self.camera = cv.CaptureFromCAM(0)
        #self.setResolution()

        # Storage for image processing.
        self.currentFrame = None
        self.processedFrame = None
        self.running = True
        # Storage for trackers.
        self.trackers = []
        self.oldLocations = []

        # Initialize trackers.
        for markerOrder in markerOrders:
            temp = ImageAnalyzer(downscaleFactor=1)
            temp.addMarkerToTrack(markerOrder, defaultKernelSize, scalingParameter)
            self.trackers.append(temp)
            self.oldLocations.append(MarkerPose(None, None, None, None))

        self.cnt = 0
        self.defaultOrientation = 0

    def setFocus(self):
        # Disable autofocus
        os.system('v4l2-ctl -d 1 -c focus_auto=0')
        
        # Set focus to a specific value. High values for nearby objects and
        # low values for distant objects.
        os.system('v4l2-ctl -d 1 -c focus_absolute=0')

        # sharpness (int)    : min=0 max=255 step=1 default=128 value=128
        os.system('v4l2-ctl -d 1 -c sharpness=200')
        
    
    def setResolution(self):
        cv.SetCaptureProperty(self.camera, cv.CV_CAP_PROP_FRAME_WIDTH, 1920)
        cv.SetCaptureProperty(self.camera, cv.CV_CAP_PROP_FRAME_HEIGHT, 1080)
        #cv.SetCaptureProperty(self.camera, cv.CV_CAP_PROP_FRAME_WIDTH, 2304)
        #cv.SetCaptureProperty(self.camera, cv.CV_CAP_PROP_FRAME_HEIGHT, 1536)

    def getImage(self):
        # Get image from camera.
        self.currentFrame = cv.QueryFrame(self.camera)
        
    def processFrame(self):
        # Locate all markers in image.
        for k in range(len(self.trackers)):
            # Previous marker location is unknown, search in the entire image.
            self.processedFrame = self.trackers[k].analyzeImage(self.currentFrame)
            markerX = self.trackers[k].markerLocationsX[0]
            markerY = self.trackers[k].markerLocationsY[0]
            orientation = self.trackers[k].markerTrackers[0].orientation
            quality = self.trackers[k].markerTrackers[0].quality
            order = self.trackers[k].markerTrackers[0].order
            self.oldLocations[k] = MarkerPose(markerX, markerY, orientation, quality, order)
    
    def drawDetectedMarkers(self):
        for k in range(len(self.trackers)):
            xm = self.oldLocations[k].x
            ym = self.oldLocations[k].y
            orientation = self.oldLocations[k].theta
            cv.Circle(self.processedFrame, (xm, ym), 4, (55, 55, 255), 2)
            xm2 = int(xm + 50*math.cos(orientation))
            ym2 = int(ym + 50*math.sin(orientation))
            cv.Line(self.processedFrame, (xm, ym), (xm2, ym2), (255, 0, 0), 2)
    
    def showProcessedFrame(self):
        cv.ShowImage('filterdemo', self.processedFrame)
        pass

    def resetAllLocations(self):
        # Reset all markers locations, forcing a full search on the next iteration.
        for k in range(len(self.trackers)):
            self.oldLocations[k] = MarkerPose(None, None, None, None)

        
    def handleKeyboardEvents(self):
        # Listen for keyboard events and take relevant actions.
        key = cv.WaitKey(20) 
        # Discard higher order bit, http://permalink.gmane.org/gmane.comp.lib.opencv.devel/410
        key = key & 0xff
        if key == 27: # Esc
            self.running = False
        if key == 114: # R
            print("Resetting")
            self.resetAllLocations()
        if key == 115: # S
            # save image
            print("Saving image")
            filename = strftime("%Y-%m-%d %H-%M-%S")
            cv.SaveImage("output/%s.png" % filename, self.currentFrame)

    def returnPositions(self):
        # Return list of all marker locations.
        return self.oldLocations
 

def main():
    t0 = time()
    t1 = time()

    toFind = [4, 5, 6, 7]
    #toFind = [7] 
    # 4 Is not optimal, can be confused by 5
    # 5 works fine
      
    #cd = CameraDriver(toFind, defaultKernelSize = 25) # Best in robolab.
    #cd = CameraDriver(toFind, defaultKernelSize = 21)
    cd = CameraDriver(toFind, defaultKernelSize = 21)
    t0 = time()
     
    pointLocationsInImage = [[1328, 340], [874, 346], [856, 756], [1300, 762]]
    realCoordinates = [[0, 0], [300, 0], [300, 250], [0, 250]]
    perspectiveConverter = PerspectiveCorrecter(pointLocationsInImage, realCoordinates)
     
    while cd.running:
        (t1, t0) = (t0, time())
        #print "time for one iteration: %f" % (t0 - t1)
        cd.getImage()
        timestamp = time()
        cd.processFrame()
        cd.drawDetectedMarkers()
        cd.showProcessedFrame()
        cd.handleKeyboardEvents()
        y = cd.returnPositions()     
        for k in range(len(y)):
            try:
                poseCorrected = perspectiveConverter.convertPose(y[k])
                #print("%8.3f %8.3f %8.3f %8.3f %s" % (poseCorrected.x, poseCorrected.y, poseCorrected.theta, poseCorrected.quality, poseCorrected.order))
                print(poseToString(poseCorrected, timestamp))                
                #print("%3d %3d %8.3f" % (y[0][0], y[0][1], y[0][2]))
            except:
                pass
            
    print("Stopping")

def poseToString(pose, timestamp):
    return "PMLPS,%s,%.3f,%.3f,%.3f,%.2f,%.2f" % (pose.order, timestamp, pose.x, pose.y, pose.theta, pose.quality)

main()
