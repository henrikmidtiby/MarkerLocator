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
    update_rate = rospy.get_param("~update_rate", 30)
    skip_images = rospy.get_param("~skip_images", 5)
    image_downscale_factor = rospy.get_param("~image_downscale_factor", 1.0)
    rate = rospy.Rate(update_rate)

    camera_device = rospy.get_param("~camera_device", 0)
    set_camera_focus(camera_device)
    cam = cv2.VideoCapture(camera_device)

    # define camera
    cam.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 1920)
    cam.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 1080)

    bridge = CvBridge()

    if not cam.isOpened():
         print("Webcam is not available")
         return -1

    count = 0
    while not rospy.is_shutdown():
        (grabbed, frame) = cam.read()
        if count % skip_images == 0:
            frame = cv2.resize(frame, (0, 0), fx=1.0/image_downscale_factor, fy=1.0/image_downscale_factor)
            msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            pub.publish(msg)
        count += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        webcam_pub()
    except rospy.ROSInterruptException:
        pass
