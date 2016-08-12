#!/usr/bin/env python

# python imports
import signal
import cv2
import math
import numpy as np
from time import time, strftime
import os

# ros imports
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# app imports
from PerspectiveTransform import PerspectiveCorrecter
from MarkerPose import MarkerPose
from MarkerTracker import MarkerTracker
from markerlocator.msg import markerpose

# global variables
stop_flag = False

# define ctrl-c handler
def signal_handler(signal, frame):
	global stop_flag
	stop_flag = True

# install ctrl-c handler
signal.signal(signal.SIGINT, signal_handler)

class CameraDriver:
	def __init__(self, marker_order, marker_size, scaling_parameter):

		# Storage for image processing.
		self.current_frame = None
		self.processed_frame = None
		self.running = True

		# Initialize trackers.
		self.tracker = MarkerTracker(marker_order, marker_size, scaling_parameter)
		self.location = MarkerPose(None, None, None, None, None)

	def open_image_window(self):
		cv2.namedWindow('filterdemo', cv2.cv.CV_WINDOW_AUTOSIZE)

	def process_frame(self):
		self.processed_frame = self.current_frame
		frame_gray = self.current_frame
		self.processed_frame = cv2.cvtColor(self.current_frame, cv2.cv.CV_GRAY2BGR)
		
		# Previous marker location is unknown, search in the entire image.
		self.current_frame = self.tracker.locate_marker(frame_gray)
		self.location = self.tracker.pose

	def show_processed_frame(self):
		xm = self.location.x
		ym = self.location.y
		orientation = self.location.theta
		if self.location.quality < 0.9:
			cv2.circle(self.processed_frame, (xm, ym), 4, (55, 55, 255), 1)
		else:
			cv2.circle(self.processed_frame, (xm, ym), 4, (55, 55, 255), 3)
		xm2 = int(xm + 50 * math.cos(orientation))
		ym2 = int(ym + 50 * math.sin(orientation))
		cv2.line(self.processed_frame, (xm, ym), (xm2, ym2), (255, 0, 0), 2)
		cv2.imshow('filterdemo', self.processed_frame)

	def reset_location(self):
		# Reset all markers locations, forcing a full search on the next iteration.
		self.location = MarkerPose(None, None, None, None, None)

	def handle_keyboard_events(self):
		# Listen for keyboard events and take relevant actions.
		key = cv2.waitKey(100)
		# Discard higher order bit, http://permalink.gmane.org/gmane.comp.lib.opencv.devel/410
		key = key & 0xff
		if key == 27:  # Esc
			self.running = False
		if key == 114:  # R
			print("Resetting")
			self.reset_location()
		if key == 115:  # S
			# save image
			print("Saving image")
			filename = strftime("%Y-%m-%d %H-%M-%S")
			cv2.imwrite("output/%s.png" % filename, self.current_frame)


class marker_locator_node():
	def __init__(self):
		rospy.init_node('MarkerLocator')

		self.rate = rospy.Rate(10) # Hz

		# needed to convert image
		self.bridge = CvBridge()

		# read topic names
		markerimage_sub_topic = rospy.get_param("~markerimage_sub",'/markerlocator/image_raw')
		markerpose_pub_topic = rospy.get_param("~markerpose_pub",'/markerlocator/markerpose')

		# read parameters
		self.show_image = rospy.get_param("~show_image", False)
		self.print_debug_messages = rospy.get_param("~print_debug_messages", False)
		down_scale_factor = rospy.get_param("/image_downscale_factor", 1.0)
		self.marker_order = rospy.get_param("~marker_order", 0)
		marker_size = rospy.get_param("~marker_size", 0) / down_scale_factor
		calib_a_wld = [rospy.get_param("/calibrate_a_world_x", 0), rospy.get_param("/calibrate_a_world_y", 0)]
		calib_a_img = [rospy.get_param("/calibrate_a_image_x", 0) / down_scale_factor,
					   rospy.get_param("/calibrate_a_image_y", 0) / down_scale_factor]
		calib_b_wld = [rospy.get_param("/calibrate_b_world_x", 0), rospy.get_param("/calibrate_b_world_y", 0)]
		calib_b_img = [rospy.get_param("/calibrate_b_image_x", 0) / down_scale_factor,
					   rospy.get_param("/calibrate_b_image_y", 0) / down_scale_factor]
		calib_c_wld = [rospy.get_param("/calibrate_c_world_x", 0), rospy.get_param("/calibrate_c_world_y", 0)]
		calib_c_img = [rospy.get_param("/calibrate_c_image_x", 0) / down_scale_factor,
					   rospy.get_param("/calibrate_c_image_y", 0) / down_scale_factor]
		calib_d_wld = [rospy.get_param("/calibrate_d_world_x", 0), rospy.get_param("/calibrate_d_world_y", 0)]
		calib_d_img = [rospy.get_param("/calibrate_d_image_x", 0) / down_scale_factor,
					   rospy.get_param("/calibrate_d_image_y", 0) / down_scale_factor]

		# static parameters
		scaling_parameter = 1000 # only for (debug) plotting purposes

		# Calibration of setup, so that the coordinates correspond to real world coordinates.
		calib_pts_image = [calib_a_img, calib_b_img, calib_c_img, calib_d_img]
		calib_pts_world = [calib_a_wld, calib_b_wld, calib_c_wld, calib_d_wld]
		self.perspective_corrector = PerspectiveCorrecter(calib_pts_image, calib_pts_world)
		if self.print_debug_messages:
			print 'Calibration points image:', calib_pts_image
			print 'Calibration points world:', calib_pts_world

		# instantiate camera driver
		self.cd = CameraDriver(self.marker_order, marker_size, scaling_parameter)
		if self.show_image:
			self.cd.open_image_window()

		# instantiate markerpose publisher
		self.markerpose_msg = markerpose()
		self.markerpose_pub = rospy.Publisher(markerpose_pub_topic, markerpose, queue_size=0)

		# instantiate image subscribers
		self.time_prev_image = time()
		rospy.Subscriber(markerimage_sub_topic, Image, self.on_new_image)

		# call updater routine
		self.updater()  

	def on_new_image(self, msg):
		self.cd.current_frame = self.bridge.imgmsg_to_cv2(msg, "8UC1")
		self.cd.process_frame()
		if self.show_image:
			self.cd.show_processed_frame()
			self.cd.handle_keyboard_events()

		# publish the marker position
		self.markerpose_msg.header.stamp = rospy.get_rostime()
		pose_corrected = self.perspective_corrector.convertPose(self.cd.location)
		self.markerpose_msg.order = pose_corrected.order
		self.markerpose_msg.x = pose_corrected.x
		self.markerpose_msg.y = pose_corrected.y
		self.markerpose_msg.theta = pose_corrected.theta
		self.markerpose_msg.quality = pose_corrected.quality
		self.markerpose_msg.timestamp = msg.header.stamp
		self.markerpose_pub.publish(self.markerpose_msg)

		# print debug info
		if self.print_debug_messages == True:
			print("Time: %.3f  Marker: %s  x: %8.3f  y:%8.3f  theta: %8.3f  quality: %8.3f  interval: %.3f" % (msg.header.stamp.to_sec(), pose_corrected.order, pose_corrected.x, pose_corrected.y, pose_corrected.theta, pose_corrected.quality, (time() - self.time_prev_image)))
			self.time_prev_image = time()

	def updater(self):
		while not rospy.is_shutdown():
			self.rate.sleep()

if __name__ == '__main__':
	try:
		marker_locator_node()
	except rospy.ROSInterruptException:
		pass


