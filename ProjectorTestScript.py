# -*- coding: utf-8 -*-
"""
Created on Thu Aug 14 13:36:47 2014

@author: henrik
"""

import sys

sys.path.append('/opt/ros/hydro/lib/python2.7/dist-packages')
import numpy as np
import cv2
import math

def getPointOnEdge(radius, orientation, k, n):
    angle = orientation + 2 * math.pi * k / n
    return np.array([radius*math.cos(angle), radius*math.sin(angle)])
    
def getPolygonForOneSegmentOfMarker(radius, orientation, offset, k, n):
    origo = getPointOnEdge(0, orientation, 1, 1) + offset
    point1 = getPointOnEdge(radius, orientation, k, n) + offset
    point2 = getPointOnEdge(radius, orientation, k + 1, n) + offset
    point3 = getPointOnEdge(radius, orientation, k + 2, n) + offset
    polygon = np.array([origo, point1, point2, point3], np.int32)
    return polygon
    
def drawMarker(canvas, offset, orientation, order, radius):
    drawingOrder = order * 4
    for k in range(3, drawingOrder - 4, 4):
        triangle = getPolygonForOneSegmentOfMarker(radius, orientation, offset, k, drawingOrder)
        cv2.fillConvexPoly(canvas, triangle, 0)
    return canvas
    
def main():
    canvas = np.ones((600, 600))
    
    order = 7
    offset = np.array([200, 300])
    radius = 100
    orientation = 0.3

    while True:
        for orient in np.linspace(0, 2*math.pi, 110, endpoint=False):
            canvas = np.ones((600, 600))
            canvas = drawMarker(canvas, offset, orient, order, radius)
            cv2.imshow('frame', canvas)
            key = cv2.waitKey(100)
            if (key & 0xFF) == ord('q'):
                return

    cv2.waitKey(0)
    pass


main()