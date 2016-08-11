#!/usr/bin/env python
from time import time, strftime
import os

# sys.path.append('/opt/ros/indigo/lib/python2.7/dist-packages')
# need to run the following line before running the script in ros mode
# source /opt/ros/indigo/setup.bash

# python imports
import signal
import cv2
import math
import numpy as np
from sensor_msgs.msg import Image

# application imports
from PerspectiveTransform import PerspectiveCorrecter
from MarkerPose import MarkerPose
from MarkerTracker import MarkerTracker

# parameters
print_debug_messages = False
show_image = True
list_of_markers_to_find = [6,7]
get_images_to_flush_cam_buffer = 5
publish_to_ros = True
camera_device = 0

# global variables
stop_flag = False
ros_publisher = False

def on_new_image(msg):
    t0 = time()
    cd.current_frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    cd.process_frame()
    cd.draw_detected_markers()
    cd.show_processed_frame()
    cd.handle_keyboard_events()
    y = cd.return_positions()
    if publish_to_ros:
        ros_publisher.publish_marker_locations(y)
    if print_debug_messages is True:
        print "time for one iteration: %f" % (time() - t0)


# define ctrl-c handler
def signal_handler(signal, frame):
    global stop_flag
    stop_flag = True

# install ctrl-c handler
signal.signal(signal.SIGINT, signal_handler)

def set_camera_focus(device):
    # Disable autofocus
    os.system('v4l2-ctl -d device -c focus_auto=0')

    # Set focus to a specific value. High values for nearby objects and
    # low values for distant objects.
    os.system('v4l2-ctl -d device -c focus_absolute=0')

    # sharpness (int)    : min=0 max=255 step=1 default=128 value=128
    os.system('v4l2-ctl -d device -c sharpness=200')

class CameraDriver:
    """
    Purpose: capture images from a camera and delegate procesing of the
    images to a different class.
    """

    def __init__(self, marker_orders=[6], default_kernel_size=21, scaling_parameter=2500, downscale_factor=1):
        # Initialize camera driver.
        # Open output window.
        if show_image is True:
            cv2.namedWindow('filterdemo', cv2.cv.CV_WINDOW_AUTOSIZE)

        # Select the camera where the images should be grabbed from.
        set_camera_focus(camera_device)
        self.camera = cv2.VideoCapture(camera_device)
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
        for k in range(get_images_to_flush_cam_buffer):
            self.current_frame = self.camera.read()[1]

    def downscale_frame(self):
        self.current_frame = cv2.resize(self.current_frame, (0, 0), fx=1.0/self.downscale_factor, fy=1.0 / self.downscale_factor)

    def process_frame(self):
        self.processed_frame = self.current_frame
        frame_gray = cv2.cvtColor(self.current_frame, cv2.cv.CV_RGB2GRAY)
        # Locate all markers in image.
        for k in range(len(self.trackers)):
            # Previous marker location is unknown, search in the entire image.
            self.current_frame = self.trackers[k].locate_marker(frame_gray)
            self.old_locations[k] = self.trackers[k].pose
            #self.old_locations[k].scale_position(self.downscale_factor)

    def draw_detected_markers(self):
        for k in range(len(self.trackers)):
            xm = self.old_locations[k].x
            ym = self.old_locations[k].y
            orientation = self.old_locations[k].theta
            if self.old_locations[k].quality < 0.9:
                cv2.circle(self.processed_frame, (xm, ym), 4, (55, 55, 255), 1)
            else:
                cv2.circle(self.processed_frame, (xm, ym), 4, (55, 55, 255), 3)

            xm2 = int(xm + 50 * math.cos(orientation))
            ym2 = int(ym + 50 * math.sin(orientation))
            cv2.line(self.processed_frame, (xm, ym), (xm2, ym2), (255, 0, 0), 2)

    def show_processed_frame(self):
        if show_image is True:
            cv2.imshow('filterdemo', self.processed_frame)

    def reset_all_locations(self):
        # Reset all markers locations, forcing a full search on the next iteration.
        for k in range(len(self.trackers)):
            self.old_locations[k] = MarkerPose(None, None, None, None, None)

    def handle_keyboard_events(self):
        if show_image is True:
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

class RosPublisher:
    def __init__(self, markers, markerpose_pub_topic):
        # Instantiate ros publisher with information about the markers that
        # will be tracked.
        self.markers = markers
        self.markerpose_pub = rospy.Publisher(markerpose_pub_topic, markerpose, queue_size=0)
        rospy.init_node('MarkerLocator')
        self.perspective_corrector = PerspectiveCorrecter(reference_point_locations_in_image, reference_point_locations_in_world_coordinates)

    def publish_marker_locations(self, locations):
        markerpose_msg.header.stamp = rospy.get_rostime()
        j = 0
        for i in self.markers:
            pose_corrected = self.perspective_corrector.convertPose(locations[j])
            if print_debug_messages == True:
                print("%8.3f %8.3f %8.3f %8.3f %s" % (pose_corrected.x, pose_corrected.y, pose_corrected.theta, pose_corrected.quality, pose_corrected.order))

            markerpose_msg.order = locations[j].order
            markerpose_msg.x = locations[j].x
            markerpose_msg.y = locations[j].y
            markerpose_msg.theta = locations[j].theta
            markerpose_msg.quality = locations[j].quality
            self.markerpose_pub.publish(markerpose_msg)
            j += 1

# Calibration of setup in robolab, so that the coordinates correspond to real world coordinates.
reference_point_locations_in_image = [[1328, 340], [874, 346], [856, 756], [1300, 762]]
reference_point_locations_in_world_coordinates = [[0, 0], [300, 0], [300, 250], [0, 250]]
perspective_corrector = PerspectiveCorrecter(reference_point_locations_in_image,
                                                 reference_point_locations_in_world_coordinates)

if publish_to_ros:
    import rospy
    from cv_bridge import CvBridge, CvBridgeError
    from markerlocator.msg import markerpose
    rospy.init_node('MarkerLocator')
    bridge = CvBridge()
    markerpose_msg = markerpose()
    image_downscale_factor = rospy.get_param("/image_downscale_factor", 1.0)
    show_image = rospy.get_param("~show_image", False)
    print_debug_messages = rospy.get_param("~print_debug_messages", False)
    ros_marker_order = rospy.get_param("~marker_order", 0)
    if ros_marker_order != 0:
        list_of_markers_to_find = [ros_marker_order]
    markerimage_sub_topic = rospy.get_param("~markerimage_sub",'/markerlocator/image_raw')
    markerpose_pub_topic = rospy.get_param("~markerpose_pub",'/markerlocator/markerpose')

    ros_publisher = RosPublisher(list_of_markers_to_find, markerpose_pub_topic)
    rospy.Subscriber(markerimage_sub_topic, Image, on_new_image)

# instantiate camera driver
cd = CameraDriver(list_of_markers_to_find, default_kernel_size=55, scaling_parameter=1000, downscale_factor=image_downscale_factor)
# cd = ImageDriver(list_of_markers_to_find, defaultKernelSize = 21)



def main():
    t0 = time()

    while cd.running and stop_flag is False:
        if publish_to_ros == False: 
            (t1, t0) = (t0, time())
            if print_debug_messages is True:
                print "time for one iteration: %f" % (t0 - t1)
            cd.get_image()
            cd.downscale_frame()
            cd.process_frame()
            cd.draw_detected_markers()
            cd.show_processed_frame()
            cd.handle_keyboard_events()
            y = cd.return_positions()
            if publish_to_ros:
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
