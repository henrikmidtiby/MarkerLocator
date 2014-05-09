#!/usr/bin/env python
from time import time
import sys
import os

sys.path.append('/opt/ros/hydro/lib/python2.7/dist-packages')
import cv

from ImageAnalyzer import ImageAnalyzer
from TrackerInWindowMode import TrackerInWindowMode

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
    import roslib; roslib.load_manifest('frobitLocator')
    import rospy
    from geometry_msgs.msg import Point
            


    
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
        self.camera = cv.CaptureFromCAM(1)
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
            temp = ImageAnalyzer(1)
            temp.addMarkerToTrack(markerOrder, defaultKernelSize, scalingParameter)
            self.trackers.append(temp)
            self.windowedTrackers.append(TrackerInWindowMode(markerOrder, defaultKernelSize))
            self.oldLocations.append(None)
        self.cnt = 0
        self.defaultOrientation = 0

    def setFocus(self):
        # Disable autofocus
        os.system('v4l2-ctl -d 1 -c focus_auto=0')
        
        # Set focus to a specific value. High values for nearby objects and
        # low values for distant objects.
        os.system('v4l2-ctl -d 1 -c focus_absolute=0')
        
    
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
            if(self.oldLocations[k] is None):
                # Previous marker location is unknown, search in the entire image.
                self.processedFrame = self.trackers[k].analyzeImage(self.currentFrame)
                markerX = self.trackers[k].markerLocationsX[0]
                markerY = self.trackers[k].markerLocationsY[0]
                self.oldLocations[k] = [markerX, markerY, self.defaultOrientation]
            else:
                # Search for marker around the old location.
                self.windowedTrackers[k].cropFrame(self.currentFrame, self.oldLocations[k][0], self.oldLocations[k][1])
                self.oldLocations[k] = self.windowedTrackers[k].locateMarker()
                self.windowedTrackers[k].showCroppedImage()
    
    def drawDetectedMarkers(self):
        for k in range(len(self.trackers)):
            xm = self.oldLocations[k][0]
            ym = self.oldLocations[k][1]
            cv.Circle(self.processedFrame, (xm, ym), 4, (55, 55, 255), 2)
            xm2 = xm + 20
            ym2 = ym + 20
            cv.Line(self.processedFrame, (xm, ym), (xm2, ym2), (255, 0, 0), 2)

    
    def showProcessedFrame(self):
        cv.ShowImage('filterdemo', self.processedFrame)
        pass

    def resetAllLocations(self):
        # Reset all markers locations, forcing a full search on the next iteration.
        for k in range(len(self.trackers)):
            self.oldLocations[k] = None
        
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
            cv.SaveImage("output/filename%03d.png" % self.cnt, self.currentFrame)
            self.cnt = self.cnt + 1

    def returnPositions(self):
        # Return list of all marker locations.
        return self.oldLocations


class RosPublisher:
    def __init__(self, markers):
        # Instantiate ros publisher with information about the markers that 
        # will be tracked.
        self.pub = []
        self.markers = markers
        for i in markers:
            self.pup.append( rospy.Publisher('positionPuplisher' + str(i), Point)  )       
        rospy.init_node('FrobitLocator')   

    def publishMarkerLocations(self, locations):
        j = 0        
        for i in self.markers:
            print 'x%i %i  y%i %i  o%i %i' %(i, locations[j][0], i, locations[j][1], i, locations[j][2])
            #ros function        
            self.pup[j].publish(  Point( locations[j][0], locations[j][1], locations[j][2] )  )
            j = j + 1                
        

def main():
    
    t0 = time()
    t1 = time()
    t2 = time()

    print 'function vers1 takes %f' %(t1-t0)
    print 'function vers2 takes %f' %(t2-t1)
    
    toFind = [7, 9]    

    if PublishToROS:  
        RP = RosPublisher(toFind)
       
    cd = CameraDriver(toFind, defaultKernelSize = 31)
    t0 = time()
     
    while cd.running:
        (t1, t0) = (t0, time())
      #  print "time for one iteration: %f" % (t0 - t1)
        cd.getImage()
        cd.processFrame()
        #cd.drawDetectedMarkers()
        cd.showProcessedFrame()
        cd.handleKeyboardEvents()
        y = cd.returnPositions()     
        if PublishToROS:
            RP.publishMarkerLocations(y)
        else:
            pass
            #print y
            try:
                print("%3d %3d %8.3f" % (y[0][0], y[0][1], y[0][2]))
            except:
                pass
                
            
    print("Stopping")


main()
