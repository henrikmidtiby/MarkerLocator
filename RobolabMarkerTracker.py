#!/usr/bin/env python
from time import time, strftime
import os

# sys.path.append('/opt/ros/indigo/lib/python2.7/dist-packages')
# need to run the following line before running the script in ros mode
# source /opt/ros/indigo/setup.bash

import cv2
import math
import numpy as np

from ImageAnalyzer import ImageAnalyzer
from PerspectiveTransform import PerspectiveCorrecter
from MarkerPose import MarkerPose
from MarkerTracker import MarkerTracker


PublishToROS = False

if PublishToROS:
    import rospy
    from geometry_msgs.msg import Point


def set_camera_focus():
    # Disable autofocus
    os.system('v4l2-ctl -d 1 -c focus_auto=0')

    # Set focus to a specific value. High values for nearby objects and
    # low values for distant objects.
    os.system('v4l2-ctl -d 1 -c focus_absolute=0')

    # sharpness (int)    : min=0 max=255 step=1 default=128 value=128
    os.system('v4l2-ctl -d 1 -c sharpness=200')


class CameraDriver:
    """
    Purpose: capture images from a camera and delegate procesing of the
    images to a different class.
    """

    def __init__(self, marker_orders=[6], default_kernel_size=21, scaling_parameter=2500, downscale_factor = 1):
        # Initialize camera driver.
        # Open output window.
        cv2.namedWindow('filterdemo', cv2.cv.CV_WINDOW_AUTOSIZE)

        # Select the camera where the images should be grabbed from.
        set_camera_focus()
        self.camera = cv2.VideoCapture(0)
        self.set_camera_resolution()

        # Storage for image processing.
        self.current_frame = None
        self.processed_frame = None
        self.running = True
        self.downscale_factor = downscale_factor

        # Storage for trackers.
        self.trackers = []
        self.old_locations = []

        # Initialize trackers.
        for marker_order in marker_orders:
            temp = MarkerTracker(marker_order, default_kernel_size, scaling_parameter)
            self.trackers.append(temp)
            self.old_locations.append(MarkerPose(None, None, None, None, None))

    def set_camera_resolution(self):
        self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 1920)
        self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 1080)

    def get_image(self):
        # Get image from camera.
        for k in range(5):
            self.current_frame = self.camera.read()[1]

    def process_frame(self):
        self.processed_frame = self.current_frame
        # Locate all markers in image.
        frame_gray = cv2.cvtColor(self.current_frame, cv2.cv.CV_RGB2GRAY)
        reduced_image = cv2.resize(frame_gray, (0,0), fx=1.0/self.downscale_factor, fy=1.0 / self.downscale_factor)
        for k in range(len(self.trackers)):
            # Previous marker location is unknown, search in the entire image.
            self.current_frame = self.trackers[k].locate_marker(reduced_image)
            self.old_locations[k] = self.trackers[k].pose
            self.old_locations[k].scale_position(self.downscale_factor)

    def draw_detected_markers(self):
        for k in range(len(self.trackers)):
            xm = self.old_locations[k].x
            ym = self.old_locations[k].y
            orientation = self.old_locations[k].theta
            cv2.circle(self.processed_frame, (xm, ym), 4, (55, 55, 255), 2)

            xm2 = int(xm + 50 * math.cos(orientation))
            ym2 = int(ym + 50 * math.sin(orientation))
            cv2.line(self.processed_frame, (xm, ym), (xm2, ym2), (255, 0, 0), 2)

    def show_processed_frame(self):
        cv2.imshow('filterdemo', self.processed_frame)

    def reset_all_locations(self):
        # Reset all markers locations, forcing a full search on the next iteration.
        for k in range(len(self.trackers)):
            self.old_locations[k] = MarkerPose(None, None, None, None, None)

    def handle_keyboard_events(self):
        # Listen for keyboard events and take relevant actions.
        key = cv2.waitKey(100)
        # Discard higher order bit, http://permalink.gmane.org/gmane.comp.lib.opencv.devel/410
        key = key & 0xff
        if key == 27:  # Esc
            self.running = False
        if key == 114:  # R
            print("Resetting")
            self.reset_all_locations()
        if key == 115:  # S
            # save image
            print("Saving image")
            filename = strftime("%Y-%m-%d %H-%M-%S")
            cv2.imwrite("output/%s.png" % filename, self.current_frame)

    def return_positions(self):
        # Return list of all marker locations.
        return self.old_locations


class ImageDriver(CameraDriver):
    """
    Purpose: Same as cameraDriver, but for single image
    """

    def __init__(self, marker_orders=[7, 8], default_kernel_size=21, scaling_parameter=2500):

        # Storage for image processing.
        self.current_frame = None
        self.processed_frame = None
        self.running = True

        # Storage for trackers.
        self.trackers = []
        self.old_locations = []

        # Initialize trackers.
        for markerOrder in marker_orders:
            temp = ImageAnalyzer(downscale_factor=1)
            temp.add_marker_to_track(markerOrder, default_kernel_size, scaling_parameter)
            self.trackers.append(temp)
            self.old_locations.append(MarkerPose(None, None, None, None))

        self.cnt = 0
        self.defaultOrientation = 0

    def get_image(self):
        # Get image from camera.
        self.current_frame = cv2.imread('/home/henrik/Dropbox/Camera Uploads/2015-11-12 10.06.20.jpg')
        if not isinstance(self.current_frame, (np.ndarray, np.generic)):
            raise TypeError('Your input type is not a numpy array')


class RosPublisher:
    def __init__(self, markers):
        # Instantiate ros publisher with information about the markers that
        # will be tracked.
        self.pub = []
        self.markers = markers
        for i in markers:
            self.pub.append(rospy.Publisher('positionPuplisher' + str(i), Point, queue_size=10))
        rospy.init_node('FrobitLocator')

    def publish_marker_locations(self, locations):
        j = 0
        for i in self.markers:
            print 'x%i %i  y%i %i  o%i %i' % (i, locations[j].x, i, locations[j].y, i, locations[j].theta)
            # ros function
            self.pub[j].publish(Point(locations[j].x, locations[j].y, locations[j].theta))
            j += 1


def main():
    list_of_markers_to_find = [4]

    if PublishToROS:
        ros_publisher = RosPublisher(list_of_markers_to_find)

    cd = CameraDriver(list_of_markers_to_find, default_kernel_size=55, scaling_parameter=1000, downscale_factor=2)  # Best in robolab.
    # cd = ImageDriver(list_of_markers_to_find, defaultKernelSize = 21)
    t0 = time()

    # Calibration of setup in robolab, so that the coordinates correspond to real world coordinates.
    reference_point_locations_in_image = [[1328, 340], [874, 346], [856, 756], [1300, 762]]
    reference_point_locations_in_world_coordinates = [[0, 0], [300, 0], [300, 250], [0, 250]]
    perspective_corrector = PerspectiveCorrecter(reference_point_locations_in_image,
                                                 reference_point_locations_in_world_coordinates)

    while cd.running:
        (t1, t0) = (t0, time())
        print "time for one iteration: %f" % (t0 - t1)
        cd.get_image()
        cd.process_frame()
        cd.draw_detected_markers()
        cd.show_processed_frame()
        cd.handle_keyboard_events()
        y = cd.return_positions()
        if PublishToROS:
            ros_publisher.publish_marker_locations(y)
        else:
            for k in range(len(y)):
                try:
                    pose_corrected = perspective_corrector.convertPose(y[k])
                    print("%8.3f %8.3f %8.3f %8.3f %s" % (pose_corrected.x,
                                                          pose_corrected.y,
                                                          pose_corrected.theta,
                                                          pose_corrected.quality,
                                                          pose_corrected.order))
                except Exception as e:
                    print("%s" % e)

    print("Stopping")


main()
