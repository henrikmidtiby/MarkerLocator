# -*- coding: utf-8 -*-
"""
Created on Wed Feb 13 20:35:04 2013

@author: Henrik Skov Midtiby
"""

import cv2
import numpy as np
import math
from MarkerPose import MarkerPose


# Inspired by this post on stackoverflow
# http://stackoverflow.com/questions/9808601/is-getperspectivetransform-broken-in-opencv-python2-wrapper
class PerspectiveCorrecter:
    def __init__(self, imageCoordinates, worldCoordinates):
        src = np.array(imageCoordinates, np.float32)
        dst = np.array(worldCoordinates, np.float32)
        self.transformMatrix = cv2.getPerspectiveTransform(src,dst)
        
    def convert(self, coordinate):
        newcoordinate = np.array([coordinate[0], coordinate[1], 1], np.float32)
        temp = np.dot(self.transformMatrix, newcoordinate)
        temp = temp * 1/temp[2]
        return [temp[0], temp[1]]
        
    def convertPose(self, pose):
        # Idea is to take the input pose and convert it to two points, the 
        # location and a director. These two points are then perspective 
        # corrected and the transformed orientation can then be determined 
        # from the two points.
        location = [pose.x, pose.y]      
        orientation = pose.theta
        dist = 10
        dx = dist * math.cos(orientation)
        dy = dist * math.sin(orientation)
        pointTwo = [location[0] + dx, location[1] + dy]
        loc1 = self.convert(location)
        loc2 = self.convert(pointTwo)
        dx = loc2[0] - loc1[0]
        dy = loc2[1] - loc1[1]
        orient = math.atan2(dy, dx)
        return MarkerPose(loc1[0], loc1[1], orient, pose.quality, pose.order)

def main():
    pointLocationsInImage = [[197, 136], [168, 403], [449, 169], [420, 347]]
    realCoordinates = [[0, 0], [0, 4], [6, 0], [6, 4]]
    perspectiveConverter = PerspectiveCorrecter(pointLocationsInImage, realCoordinates)
    print(perspectiveConverter.convert([294, 269]))

    pointLocationsInImage = [[0, 0], [0, 4], [6, 0], [6, 4]]
    realCoordinates = [[0, 0], [0, 4], [6, 0], [6, 4]]
    perspectiveConverter = PerspectiveCorrecter(pointLocationsInImage, realCoordinates)
    pose = perspectiveConverter.convertPose(MarkerPose(50, 120, 10, 0.2))
    print("%8.3f %8.3f %8.3f %8.3f" % (pose.x, pose.y, pose.theta, pose.quality))
        
if __name__ == '__main__':
    main()
