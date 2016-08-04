#!/usr/bin/env python
from time import time, strftime
import sys
import os

sys.path.append('/opt/ros/hydro/lib/python2.7/dist-packages')
import cv2
import math
import numpy as np

from ImageAnalyzer import ImageAnalyzer
from TrackerInWindowMode import TrackerInWindowMode
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

PublishToROS = False

if PublishToROS:
    import rospy
    from geometry_msgs.msg import Point
            

    
class CameraDriver:
    ''' 
    Purpose: capture images from a camera and delegate procesing of the 
    images to a different class.
    '''
    def __init__(self, markerOrders = [6], defaultKernelSize = 21, scalingParameter = 2500):
        # Initialize camera driver.
        # Open output window.

        cv2.namedWindow('filterdemo', cv2.cv.CV_WINDOW_AUTOSIZE)

        self.setFocus()
        # Select the camera where the images should be grabbed from.
        self.camera = cv2.VideoCapture(0)
        self.setResolution()

        # Storage for image processing.
        self.currentFrame = None
        self.processedFrame = None
        self.running = True
        # Storage for trackers.
        self.trackers = []
        self.windowedTrackers = []
        self.oldLocations = []
        # Initialize trackers.
        for markerOrder in markerOrders:
            temp = ImageAnalyzer(downscaleFactor=1)
            temp.addMarkerToTrack(markerOrder, defaultKernelSize, scalingParameter)
            self.trackers.append(temp)
            self.windowedTrackers.append(TrackerInWindowMode(markerOrder, defaultKernelSize))
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
        self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 1920)
        self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 1080)

    def getImage(self):
        # Get image from camera.
        self.currentFrame = self.camera.read()[1]
        print(self.currentFrame[1:5,1,1])
        pass

    def processFrame(self):
        # Locate all markers in image.
        for k in range(len(self.trackers)):
            if self.oldLocations[k].x is None:
                # Previous marker location is unknown, search in the entire image.
                self.processedFrame = self.trackers[k].analyzeImage(self.currentFrame)
                markerX = self.trackers[k].markerLocationsX[0]
                markerY = self.trackers[k].markerLocationsY[0]
                order = self.trackers[k].markerTrackers[0].order
                quality = self.trackers[k].markerTrackers[0].quality
                self.oldLocations[k] = MarkerPose(markerX, markerY, self.defaultOrientation, quality, order)
            else:
                # Search for marker around the old location.
                self.processedFrame = self.currentFrame
                self.windowedTrackers[k].crop_frame(self.currentFrame, self.oldLocations[k].x, self.oldLocations[k].y)
                self.oldLocations[k] = self.windowedTrackers[k].locate_marker()
                self.windowedTrackers[k].show_cropped_image()
    
    def drawDetectedMarkers(self):
        for k in range(len(self.trackers)):
            xm = self.oldLocations[k].x
            ym = self.oldLocations[k].y
            orientation = self.oldLocations[k].theta
            cv2.circle(self.processedFrame, (xm, ym), 4, (55, 55, 255), 2)
            
            xm2 = int(xm + 50*math.cos(orientation))
            ym2 = int(ym + 50*math.sin(orientation))
            cv2.line(self.processedFrame, (xm, ym), (xm2, ym2), (255, 0, 0), 2)
    
    def showProcessedFrame(self):
        cv2.imshow('filterdemo', self.processedFrame)

    def resetAllLocations(self):
        # Reset all markers locations, forcing a full search on the next iteration.
        for k in range(len(self.trackers)):
            self.oldLocations[k] = MarkerPose(None, None, None, None)
        
    def handleKeyboardEvents(self):
        # Listen for keyboard events and take relevant actions.
        key = cv2.waitKey(100) 
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
            cv2.imwrite("output/%s.png" % filename, self.currentFrame)

    def returnPositions(self):
        # Return list of all marker locations.
        return self.oldLocations

 
class CameraDriverSimple:
    ''' 
    Purpose: capture images from a camera and delegate procesing of the 
    images to a different class.
    '''
    def __init__(self, markerOrders = [7, 8], defaultKernelSize = 21, scalingParameter = 2500):
        # Initialize camera driver.
        # Open output window.
        cv2.namedWindow('filterdemo', cv2.cv.CV_WINDOW_AUTOSIZE)

        self.setFocus()
        # Select the camera where the images should be grabbed from.
        self.camera = cv2.VideoCapture(0)
        self.setResolution()

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
        self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 1920)
        self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 1080)        
        
        
    def getImage(self):
        # Get image from camera.
        self.currentFrame = self.camera.read()[1]
        
    def processFrame(self):
        # Locate all markers in image.
        for k in range(len(self.trackers)):
            # Previous marker location is unknown, search in the entire image.
            self.processedFrame = self.trackers[k].analyzeImage(self.currentFrame)
            markerX = self.trackers[k].markerLocationsX[0]
            markerY = self.trackers[k].markerLocationsY[0]
            orientation = self.trackers[k].markerTrackers[0].orientation
            quality = self.trackers[k].markerTrackers[0].quality
            self.oldLocations[k] = MarkerPose(markerX, markerY, orientation, quality)
    
    def drawDetectedMarkers(self):
        for k in range(len(self.trackers)):
            xm = self.oldLocations[k].x
            ym = self.oldLocations[k].y
            cv2.circle(self.processedFrame, (xm, ym), 4, (55, 55, 255), 2)
            xm2 = xm + 20
            ym2 = ym + 20
            cv2.line(self.processedFrame, (xm, ym), (xm2, ym2), (255, 0, 0), 2)
    
    def showProcessedFrame(self):
        cv2.imshow('filterdemo', self.processedFrame)
        pass

    def resetAllLocations(self):
        # Reset all markers locations, forcing a full search on the next iteration.
        for k in range(len(self.trackers)):
            self.oldLocations[k] = MarkerPose(None, None, None, None)
        
    def handleKeyboardEvents(self):
        # Listen for keyboard events and take relevant actions.
        key = cv2.waitKey(20) 
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
            cv2.imwrite("output/%s.png" % filename, self.currentFrame)

    def returnPositions(self):
        # Return list of all marker locations.
        return self.oldLocations




class ImageDriver:
    ''' 
    Purpose: Same as cameraDriver, but for single image
    '''
    def __init__(self, markerOrders = [7, 8], defaultKernelSize = 21, scalingParameter = 2500):

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

        
    def getImage(self):
        # Get image from camera.
        self.currentFrame = cv2.imread('/home/henrik/Dropbox/Camera Uploads/2015-11-12 10.06.20.jpg')
        if not isinstance(self.currentFrame, (np.ndarray, np.generic)):
            raise TypeError('Your input type is not a numpy array')


    def drawDetectedMarkers(self):
        for k in range(len(self.trackers)):
            xm = self.oldLocations[k].x
            ym = self.oldLocations[k].y
            cv2.circle(self.processedFrame, (xm, ym), 4, (55, 55, 255), 2)
            xm2 = xm + 20
            ym2 = ym + 20
            cv2.line(self.processedFrame, (xm, ym), (xm2, ym2), (255, 0, 0), 2)
    
        
    def processFrame(self):
        # Locate all markers in image.
        for k in range(len(self.trackers)):
            # Previous marker location is unknown, search in the entire image.
            if not isinstance(self.currentFrame, (np.ndarray, np.generic)):
                raise TypeError('Your input type is not a numpy array')
            self.processedFrame = self.trackers[k].analyzeImage(self.currentFrame)
            markerX = self.trackers[k].markerLocationsX[0]
            markerY = self.trackers[k].markerLocationsY[0]
            orientation = self.trackers[k].markerTrackers[0].orientation
            quality = self.trackers[k].markerTrackers[0].quality
            self.oldLocations[k] = MarkerPose(markerX, markerY, orientation, quality)
            
            
    def showProcessedFrame(self):
        cv2.imshow('filterdemo', self.processedFrame)
        cv2.waitKey(1)
        

    def handleKeyboardEvents(self):
        # Listen for keyboard events and take relevant actions.
        key = cv2.waitKey(20)
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
            cv2.imwrite("output/%s.png" % filename, self.currentFrame)

    def returnPositions(self):
        # Return list of all marker locations.
        return self.oldLocations

    def resetAllLocations(self):
        pass


class RosPublisher:
    def __init__(self, markers):
        # Instantiate ros publisher with information about the markers that 
        # will be tracked.
        self.pub = []
        self.markers = markers
        for i in markers:
            self.pub.append( rospy.Publisher('positionPuplisher' + str(i), Point, queue_size = 10)  )
        rospy.init_node('FrobitLocator')

    def publishMarkerLocations(self, locations):
        j = 0
        for i in self.markers:
            print 'x%i %i  y%i %i  o%i %i' %(i, locations[j].x, i, locations[j].y, i, locations[j].theta)
            # ros function
            self.pub[j].publish(Point(locations[j].x, locations[j].y, locations[j].theta))
            j += 1


def main():
    toFind = [4]

    if PublishToROS:  
        RP = RosPublisher(toFind)

    cd = CameraDriver(toFind, defaultKernelSize = 25) # Best in robolab.
    #cd = ImageDriver(toFind, defaultKernelSize = 21)
    t0 = time()

    # Calibration of setup in robolab, so that the coordinates correspond to real world coordinates.
    pointLocationsInImage = [[1328, 340], [874, 346], [856, 756], [1300, 762]]
    realCoordinates = [[0, 0], [300, 0], [300, 250], [0, 250]]
    perspectiveConverter = PerspectiveCorrecter(pointLocationsInImage, realCoordinates)
     
    while cd.running:
        (t1, t0) = (t0, time())
        print "time for one iteration: %f" % (t0 - t1)
        cd.getImage()
        cd.processFrame()
        cd.drawDetectedMarkers()
        cd.showProcessedFrame()
        cd.handleKeyboardEvents()
        y = cd.returnPositions()     
        if PublishToROS:
            RP.publishMarkerLocations(y)
        else:
            for k in range(len(y)):
                try:
                    poseCorrected = perspectiveConverter.convertPose(y[k])
                    print("%8.3f %8.3f %8.3f %8.3f %s" % (poseCorrected.x, poseCorrected.y, poseCorrected.theta, poseCorrected.quality, poseCorrected.order))
                except:
                    pass
                
    print("Stopping")

main()
