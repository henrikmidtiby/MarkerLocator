#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import os

def set_camera_focus(device):
    # Disable autofocus
    os.system('v4l2-ctl -d %d -c focus_auto=0' % device)

    # Set focus to a specific value. High values for nearby objects and
    # low values for distant objects.
    os.system('v4l2-ctl -d %d -c focus_absolute=0' % device)

    # sharpness (int)    : min=0 max=255 step=1 default=128 value=128
    os.system('v4l2-ctl -d %d -c sharpness=200' % device)

def webcam_pub():
    rospy.init_node('webcam_pub', anonymous=True)
    markerimage_pub_topic = rospy.get_param("~markerimage_pub",'/markerlocator/image_raw')
    pub = rospy.Publisher(markerimage_pub_topic, Image, queue_size=0)
    camera_width = rospy.get_param("~camera_width", 1920)
    camera_height = rospy.get_param("~camera_height", 1080)
    update_rate = rospy.get_param("~update_rate", 30)
    skip_images = rospy.get_param("~skip_images", 5)
    image_downscale_factor = rospy.get_param("/image_downscale_factor", 1.0)
    rate = rospy.Rate(update_rate)

    camera_device = rospy.get_param("~camera_device", 0)
    set_camera_focus(camera_device)
    cam = cv2.VideoCapture(camera_device)

    # define camera
    # cam.set(cv2.CAP_PROP_FRAME_WIDTH, camera_width)
    # cam.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_height)
    cam.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, camera_width)
    cam.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, camera_height)

    bridge = CvBridge()

    if not cam.isOpened():
         print("Webcam is not available")
         return -1

    count = 0
    while not rospy.is_shutdown():
        (grabbed, frame) = cam.read()
        if count % skip_images == 0:
            time_captured = rospy.Time.now()
            frame = cv2.resize(frame, (0, 0), fx=1.0/image_downscale_factor, fy=1.0/image_downscale_factor)
            # frame_gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            frame_gray = cv2.cvtColor(frame, cv2.cv.CV_RGB2GRAY)
            msg = bridge.cv2_to_imgmsg(frame_gray, encoding="8UC1")
            msg.header.stamp = time_captured
            pub.publish(msg)
        count += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        webcam_pub()
    except rospy.ROSInterruptException:
        pass
