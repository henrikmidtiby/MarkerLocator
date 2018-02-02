# -*- coding: utf-8 -*-
"""
Python program for tracking certain markers in videos.

Written by: Henrik Skov Midtiby
Date: 2015-11-14
"""

import argparse
import cv2
import MarkerTracker
import math


def track_marker_in_video(video_file_to_analyze_filename, output_filename_input, order_of_marker_input,
                          size_of_kernel_input, track_orientation, scale_factor, video_output_filename):
    # Open video file for reading and output file for writing.
    cap = cv2.VideoCapture()
    cap.open(video_file_to_analyze_filename)
    output_file = open(output_filename_input, 'w')

    # Initialize the marker tracker.
    tracker = MarkerTracker.MarkerTracker(order_of_marker_input, size_of_kernel_input, 1.0)
    tracker.track_marker_with_missing_black_leg = track_orientation

    # Add headers to the output
    string_to_file = "frame\tx\ty\ttheta\tquality\n"
    print(string_to_file[0:-1])
    output_file.write(string_to_file)

    # Main loop
    counter = 0
    while cap.isOpened():
        counter += 1
        # Read a new image from the file.
        ret, frame = cap.read()

        # Halt if reading failed.
        if not ret:
            break

        frame = cv2.resize(frame, (0, 0), fx=1/scale_factor, fy=1/scale_factor)

        if counter == 1 and video_output_filename is not None:
            frames_per_second = 20
            fourcc = cv2.VideoWriter_fourcc(*'MJPG')
            shape_of_frame = (frame.shape[1], frame.shape[0])
            print(shape_of_frame)
            video_output_file = cv2.VideoWriter(video_output_filename,
                                                fourcc,
                                                frames_per_second,
                                                shape_of_frame,
                                                isColor=True)
            print(video_output_file.isOpened())

        # Convert image to grayscale.
        gray_scale_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Locate marker in image.
        marker_pose = tracker.locate_marker(gray_scale_image)

        # Show and write to file information about the detected marker.
        show_and_store_marker_location(counter, marker_pose, output_file)

        # Mark the center of the marker
        annotate_frame_with_detected_marker(frame, marker_pose, order_of_marker_input, size_of_kernel_input,
                                            track_orientation)

        # Show the annotated image.
        norm_image = cv2.normalize(tracker.frame_sum_squared, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
        cv2.imshow('marker_response', norm_image)
        cv2.imshow('frame', frame)

        # Break the look if the key 'q' was pressed.
        key_value = cv2.waitKey(10)
        if key_value & 0xFF == ord('q'):
            break
        # Wait if 'p' was pressed.
        if key_value & 0xFF == ord('p'):
            cv2.waitKey(1000000)

        if video_output_filename is not None:
            video_output_file.write(frame)

    output_file.close()
    if video_output_filename is not None:
        video_output_file.release()
    return


def show_and_store_marker_location(counter, marker_pose, output_file):
    string_to_file = "%3d\t%3d\t%3d\t%.2f\t%.2f\n" % (
        counter, marker_pose.x, marker_pose.y, marker_pose.theta, marker_pose.quality)
    print(string_to_file[0:-1])
    output_file.write(string_to_file)


def annotate_frame_with_detected_marker(frame, marker_pose, order_of_marker_input, size_of_kernel_input,
                                        track_orientation):
    line_width_of_circle = 2
    if marker_pose.quality > 0.5:
        marker_color = (0, 255, 0)
    else:
        marker_color = (255, 0, 255)
    cv2.circle(frame, (marker_pose.x, marker_pose.y), int(size_of_kernel_input / 2), marker_color, line_width_of_circle)
    dist = 50
    direction_line_width = 1
    if track_orientation:
        # Mark the orientation of the detected marker
        point1 = (marker_pose.x, marker_pose.y)
        point2 = (math.trunc(marker_pose.x + dist * math.cos(marker_pose.theta)),
                  math.trunc(marker_pose.y + dist * math.sin(marker_pose.theta)))

        cv2.line(frame, point1, point2, marker_color, direction_line_width)
    else:
        point1 = (marker_pose.x, marker_pose.y)
        theta = marker_pose.theta
        for k in range(order_of_marker_input):
            theta += 2 * math.pi / order_of_marker_input
            point2 = (math.trunc(marker_pose.x + dist * math.cos(theta)),
                      math.trunc(marker_pose.y + dist * math.sin(theta)))

            cv2.line(frame, point1, point2, marker_color, direction_line_width)


# Launch the program.
# video_file_to_analyze = 'input/2015-11-12 09.06.21.mp4'
# video_file_to_analyze = 'input/2015-11-12 08.58.29.mp4'
# video_file_to_analyze = '/home/henrik/Dropbox/Camera Uploads/2016-07-08 06.22.55.mp4'
# video_file_to_analyze = '/home/henrik/Dropbox/Camera Uploads/2016-09-16 07.54.48.mp4'
# output_filename = 'output/positions3.txt'
# order_of_marker = 5
# size_of_kernel = 201
# track_marker_in_video(video_file_to_analyze, output_filename, order_of_marker, size_of_kernel)


parser = argparse.ArgumentParser(description='Track a n-fold marker in a video. The location, orientation '
                                             'and a quality estimate of the tracked marker is stored in '
                                             'OUTPUT_FILENAME.')
parser.add_argument('input_video_filename',
                    type=str,
                    help='Name of video file to analyse.')
parser.add_argument('--order', dest='marker_order', default=5,
                    type=int,
                    help='Order of the n-fold marker to track. The order is the number of \'black legs\' '
                         'in the marker. Remember to include the removed leg if using an oriented marker.')
parser.add_argument('--scalefactor', dest='scale_factor', default=1,
                    type=int,
                    help='Size reduction factor.')
parser.add_argument('--kernelsize', dest='kernelsize', default=51,
                    type=int,
                    help='Size of the kernel that is used to track the marker. This value should be '
                         'less than the diameter of the marker. Larger values gives better tracking '
                         'but requires more calculations.')
parser.add_argument('--trackorientation', dest='marker_is_oriented', default=False,
                    action="store_true",
                    help='Specify if the marker is oriented (missing a black leg) or not. This only '
                         'affects the quality estimator of the detected marker.')
parser.add_argument('--output', dest='output_filename', default='temppositions.txt',
                    type=str,
                    help='Location of text file in which to store the tracked locations.')
parser.add_argument('--videooutput', dest='video_output_filename', default=None,
                    type=str,
                    help='Location of video file in which to store the analysed wideo.')

args = parser.parse_args()

track_marker_in_video(args.input_video_filename, args.output_filename, args.marker_order, args.kernelsize,
                      args.marker_is_oriented, args.scale_factor, args.video_output_filename)

# Some example command line
# python VideoObjectTracker.py input/2015-11-12\ 08.58.29.mp4 --order 2 --kernelsize=151
# python VideoObjectTracker.py input/2015-11-12\ 07.58.26.mp4 --order 6 --kernelsize 101
# python VideoObjectTracker.py input/2015-11-12\ 09.06.21.mp4 --order 4 --kernelsize 101
# python VideoObjectTracker.py input/2015-11-11\ 12.50.19.mp4 --order 6 --kernelsize=101 --trackorientation
#
#
