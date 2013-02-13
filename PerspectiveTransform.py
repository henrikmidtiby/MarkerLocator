# -*- coding: utf-8 -*-
"""
Created on Wed Feb 13 20:35:04 2013

@author: hemi
"""

import cv2
import numpy as np

def calculatePerspectiveTransform():
    # Specify calibration points
    pointLocationsInImage = [[197, 136], [168, 403], [449, 169], [420, 347]]
    realCoordinates = [[0, 0], [0, 4], [6, 0], [6, 4]]
    src = np.array(pointLocationsInImage, np.float32)
    dst = np.array(realCoordinates, np.float32)

    return cv2.getPerspectiveTransform(src,dst)

def convertCoordinates(matrix, coordinate):
    newcoordinate = np.array([coordinate[0], coordinate[1], 1], np.float32)
    temp = np.dot(matrix, newcoordinate)
    temp = temp * 1/temp[2]
    return [temp[0], temp[1]]
    
matrix = calculatePerspectiveTransform()
print convertCoordinates(matrix, [294, 269])