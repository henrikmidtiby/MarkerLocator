# -*- coding: utf-8 -*-
"""
Created on Wed Feb 13 20:35:04 2013

@author: hemi
"""

import cv2
import numpy as np


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
        
     
pointLocationsInImage = [[197, 136], [168, 403], [449, 169], [420, 347]]
realCoordinates = [[0, 0], [0, 4], [6, 0], [6, 4]]
perspectiveConverter = PerspectiveCorrecter(pointLocationsInImage, realCoordinates)
print perspectiveConverter.convert([294, 269])
        
        