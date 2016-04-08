# -*- coding: utf-8 -*-
"""
Marker tracker for locating n-fold edges in images using convolution.

@author: Henrik Skov Midtiby
"""
import cv2
import numpy as np
import math


class MarkerTracker:
    '''
    Purpose: Locate a certain marker in an image.
    '''
    def __init__(self, order, kernelSize, scaleFactor):
        self.kernelSize = kernelSize
        (kernelReal, kernelImag) = self.generateSymmetryDetectorKernel(order, kernelSize)
        self.order = order
        self.matReal = np.zeros((kernelSize, kernelSize), dtype=np.float32)
        self.matImag = np.zeros((kernelSize, kernelSize), dtype=np.float32)
        for i in range(kernelSize):
            for j in range(kernelSize):
                self.matReal[i, j] = kernelReal[i][j] / scaleFactor
                self.matImag[i, j] = kernelImag[i][j] / scaleFactor
        self.lastMarkerLocation = (None, None)
        self.orientation = None

        (kernelRealThirdHarmonics, kernelImagThirdHarmonics) = self.generateSymmetryDetectorKernel(3*order, kernelSize)
        self.matRealThirdHarmonics = np.zeros((kernelSize, kernelSize), np.float32)
        self.matImagThirdHarmonics = np.zeros((kernelSize, kernelSize), np.float32)
        for i in range(kernelSize):
            for j in range(kernelSize):
                self.matRealThirdHarmonics[i, j] = kernelRealThirdHarmonics[i][j] / scaleFactor
                self.matImagThirdHarmonics[i, j] = kernelImagThirdHarmonics[i][j] / scaleFactor

        self.quality = 0
                  
    def generateSymmetryDetectorKernel(self, order, kernelsize):
        valueRange = np.linspace(-1, 1, kernelsize);
        temp1 = np.meshgrid(valueRange, valueRange)
        kernel = temp1[0] + 1j*temp1[1];

        magni = abs(kernel);
        kernel = kernel**order;
        kernel = kernel*np.exp(-8*magni**2);
         
        return (np.real(kernel), np.imag(kernel))

    def allocateSpaceGivenFirstFrame(self, frame):
        framewidth=frame.shape[1]
        frameheight=frame.shape[0]
        self.newFrameImage32F = np.zeros((frameheight, framewidth,3), dtype=np.float32)
        self.newFrameImage32F = np.zeros((frameheight, framewidth,3), dtype=np.float32)
        self.frameReal = np.zeros((frameheight,framewidth,1), dtype=np.float32)
        self.frameImag = np.zeros((frameheight,framewidth,1), dtype=np.float32)
        self.frameRealThirdHarmonics = np.zeros((frameheight,framewidth,1), dtype=np.float32)
        self.frameImagThirdHarmonics = np.zeros((frameheight,framewidth,1), dtype=np.float32)
        self.frameRealSq = np.zeros((frameheight,framewidth,1), dtype=np.float32)
        self.frameImagSq = np.zeros((frameheight,framewidth,1), dtype=np.float32)
        self.frameSumSq = np.zeros((frameheight,framewidth,1), dtype=np.float32)


    def locateMarker(self, frame):
        self.frameReal = frame
        self.frameImag = frame
        self.frameRealThirdHarmonics = frame
        self.frameImagThirdHarmonics = frame

        # Calculate convolution and determine response strength.
        self.frameReal = cv2.filter2D(self.frameReal, cv2.CV_32F, self.matReal)
        self.frameImag = cv2.filter2D(self.frameImag, cv2.CV_32F, self.matImag)
        
        
        
        self.frameRealSq = np.multiply(self.frameReal, self.frameReal)
        self.frameImagSq = np.multiply(self.frameImag, self.frameImag)
        self.frameSumSq = self.frameRealSq + self.frameImagSq

        # Calculate convolution of third harmonics for quality estimation.
        self.frameRealThirdHarmonics = cv2.filter2D(self.frameRealThirdHarmonics, cv2.CV_32F, self.matRealThirdHarmonics)
        self.frameImagThirdHarmonics = cv2.filter2D(self.frameImagThirdHarmonics, cv2.CV_32F, self.matImagThirdHarmonics)
        
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(self.frameSumSq)
        self.lastMarkerLocation = max_loc
        (xm, ym) = max_loc
        self.determineMarkerOrientation(frame)
        self.determineMarkerQuality()
        return max_loc

    def determineMarkerOrientation(self, frame):    
        (xm, ym) = self.lastMarkerLocation
        #realval = cv.Get2D(self.frameReal, ym, xm)[0]
        #imagval = cv.Get2D(self.frameImag, ym, xm)[0]
        realval=self.frameReal[ym, xm]
        imagval = self.frameImag[ym, xm]
        
        self.orientation = (math.atan2(-realval, imagval) - math.pi / 2) / self.order

        maxValue = 0
        maxOrient = 0
        searchDist = self.kernelSize / 3
        for k in range(self.order):
            orient = self.orientation + 2 * k * math.pi / self.order
            xm2 = int(xm + searchDist*math.cos(orient))
            ym2 = int(ym + searchDist*math.sin(orient))
            if(xm2 > 0 and ym2 > 0 and xm2 < frame.shape[1] and ym2 < frame.shape[0]):
                try:
                    intensity = frame[ym2,xm2]
                    if(intensity[0] > maxValue):
                        maxValue = intensity[0]
                        maxOrient = orient
                except:
                    print("determineMarkerOrientation: error: %d %d %d %d" % (ym2, xm2, frame.shape[1], frame.shape[0]))
                    pass

        self.orientation = self.limitAngleToRange(maxOrient)

    def determineMarkerQuality(self):
        (xm, ym) = self.lastMarkerLocation
#        realval = cv.Get2D(self.frameReal, ym, xm)[0]
#        imagval = cv.Get2D(self.frameImag, ym, xm)[0]

        realval=self.frameReal[ym, xm]
        imagval = self.frameImag[ym, xm]        

        realvalThirdHarmonics = self.frameRealThirdHarmonics[ym, xm]
        imagvalThirdHarmonics = self.frameImagThirdHarmonics[ym, xm]
        
        
        
        argumentPredicted = 3*math.atan2(-realval, imagval)
        argumentThirdHarmonics = math.atan2(-realvalThirdHarmonics, imagvalThirdHarmonics)
        argumentPredicted = self.limitAngleToRange(argumentPredicted)
        argumentThirdHarmonics = self.limitAngleToRange(argumentThirdHarmonics)
        difference = self.limitAngleToRange(argumentPredicted - argumentThirdHarmonics)
        strength = math.sqrt(realval*realval + imagval*imagval)
        strengthThirdHarmonics = math.sqrt(realvalThirdHarmonics*realvalThirdHarmonics + imagvalThirdHarmonics*imagvalThirdHarmonics)
        #print("Arg predicted: %5.2f  Arg found: %5.2f  Difference: %5.2f" % (argumentPredicted, argumentThirdHarmonics, difference))        
        #print("angdifferenge: %5.2f  strengthRatio: %8.5f" % (difference, strengthThirdHarmonics / strength))
        # angdifference \in [-0.2; 0.2]
        # strengthRatio \in [0.03; 0.055]
        self.quality = math.exp(-math.pow(difference/0.3, 2))
        #self.printMarkerQuality(self.quality)
        
    def printMarkerQuality(self, quality):
        stars = ""        
        if(quality > 0.5):
            stars = "**"
        if(quality > 0.7):
            stars = "***"
        if(quality > 0.9):
            stars = "****"
        print("quality = %d): %5.2f %s" % (self.order, quality, stars))
        
    def limitAngleToRange(self, angle):
        while(angle < math.pi):
            angle += 2*math.pi
        while(angle > math.pi):
            angle -= 2*math.pi
        return angle