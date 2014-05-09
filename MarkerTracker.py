# -*- coding: utf-8 -*-
"""
Marker tracker for locating n-fold edges in images using convolution.

@author: Henrik Skov Midtiby
"""
import cv
import numpy as np
import math


class MarkerTracker:
    '''
    Purpose: Locate a certain marker in an image.
    '''
    def __init__(self, order, kernelSize, scaleFactor):
        (kernelReal, kernelImag) = self.generateSymmetryDetectorKernel(order, kernelSize)
        self.order = order
        self.matReal = cv.CreateMat(kernelSize, kernelSize, cv.CV_32FC1)
        self.matImag = cv.CreateMat(kernelSize, kernelSize, cv.CV_32FC1)
        for i in range(kernelSize):
            for j in range(kernelSize):
                self.matReal[i, j] = kernelReal[i][j] / scaleFactor
                self.matImag[i, j] = kernelImag[i][j] / scaleFactor
        self.lastMarkerLocation = (None, None)
        self.orientation = None

        (kernelRealThirdHarmonics, kernelImagThirdHarmonics) = self.generateSymmetryDetectorKernel(3*order, kernelSize)
        self.matRealThirdHarmonics = cv.CreateMat(kernelSize, kernelSize, cv.CV_32FC1)
        self.matImagThirdHarmonics = cv.CreateMat(kernelSize, kernelSize, cv.CV_32FC1)
        for i in range(kernelSize):
            for j in range(kernelSize):
                self.matRealThirdHarmonics[i, j] = kernelRealThirdHarmonics[i][j] / scaleFactor
                self.matImagThirdHarmonics[i, j] = kernelImagThirdHarmonics[i][j] / scaleFactor

                  
    def generateSymmetryDetectorKernel(self, order, kernelsize):
        valueRange = np.linspace(-1, 1, kernelsize);
        temp1 = np.meshgrid(valueRange, valueRange)
        kernel = temp1[0] + 1j*temp1[1];
            
        magni = abs(kernel);
        kernel = kernel**order;
        kernel = kernel*np.exp(-8*magni**2);
         
        return (np.real(kernel), np.imag(kernel))

    def allocateSpaceGivenFirstFrame(self, frame):
        self.newFrameImage32F = cv.CreateImage((frame.width, frame.height), cv.IPL_DEPTH_32F, 3)
        self.frameReal = cv.CreateImage ((frame.width, frame.height), cv.IPL_DEPTH_32F, 1)
        self.frameImag = cv.CreateImage ((frame.width, frame.height), cv.IPL_DEPTH_32F, 1)
        self.frameRealThirdHarmonics = cv.CreateImage ((frame.width, frame.height), cv.IPL_DEPTH_32F, 1)
        self.frameImagThirdHarmonics = cv.CreateImage ((frame.width, frame.height), cv.IPL_DEPTH_32F, 1)
        self.frameRealSq = cv.CreateImage ((frame.width, frame.height), cv.IPL_DEPTH_32F, 1)
        self.frameImagSq = cv.CreateImage ((frame.width, frame.height), cv.IPL_DEPTH_32F, 1)
        self.frameSumSq = cv.CreateImage ((frame.width, frame.height), cv.IPL_DEPTH_32F, 1)

    
    def locateMarker(self, frame):
        self.frameReal = cv.CloneImage(frame)
        self.frameImag = cv.CloneImage(frame)
        self.frameRealThirdHarmonics = cv.CloneImage(frame)
        self.frameImagThirdHarmonics = cv.CloneImage(frame)

        # Calculate convolution and determine response strength.
        cv.Filter2D(self.frameReal, self.frameReal, self.matReal)
        cv.Filter2D(self.frameImag, self.frameImag, self.matImag)
        cv.Mul(self.frameReal, self.frameReal, self.frameRealSq)
        cv.Mul(self.frameImag, self.frameImag, self.frameImagSq)
        cv.Add(self.frameRealSq, self.frameImagSq, self.frameSumSq)

        # Calculate convolution of third harmonics for quality estimation.
        cv.Filter2D(self.frameRealThirdHarmonics, self.frameRealThirdHarmonics, self.matRealThirdHarmonics)
        cv.Filter2D(self.frameImagThirdHarmonics, self.frameImagThirdHarmonics, self.matImagThirdHarmonics)
        
        min_val, max_val, min_loc, max_loc = cv.MinMaxLoc(self.frameSumSq)
        self.lastMarkerLocation = max_loc
        (xm, ym) = max_loc
        self.determineMarkerOrientation(frame)
        self.determineMarkerQuality()
        return max_loc

    def determineMarkerOrientation(self, frame):    
        (xm, ym) = self.lastMarkerLocation
        realval = cv.Get2D(self.frameReal, ym, xm)[0]
        imagval = cv.Get2D(self.frameImag, ym, xm)[0]
        self.orientation = (math.atan2(-realval, imagval) - math.pi / 2) / self.order

        maxValue = 0
        maxOrient = 0
        searchDist = 10
        for k in range(self.order):
            orient = self.orientation + 2 * k * math.pi / self.order
            xm2 = int(xm + searchDist*math.cos(orient))
            ym2 = int(ym + searchDist*math.sin(orient))
            if(xm2 > 0 and ym2 > 0 and xm2 < frame.width and ym2 < frame.height):
                try:
                    intensity = cv.Get2D(frame, ym2, xm2)
                    if(intensity[0] > maxValue):
                        maxValue = intensity[0]
                        maxOrient = orient
                except:
                    print("determineMarkerOrientation: error: %d %d %d %d" % (ym2, xm2, frame.width, frame.height))
                    pass

        self.orientation = self.limitAngleToRange(maxOrient)

    def determineMarkerQuality(self):
        (xm, ym) = self.lastMarkerLocation
        realval = cv.Get2D(self.frameReal, ym, xm)[0]
        imagval = cv.Get2D(self.frameImag, ym, xm)[0]
        realvalThirdHarmonics = cv.Get2D(self.frameRealThirdHarmonics, ym, xm)[0]
        imagvalThirdHarmonics = cv.Get2D(self.frameImagThirdHarmonics, ym, xm)[0]
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
        quality = math.exp(-math.pow(difference/0.3, 2))
        self.printMarkerQuality(quality)
        
    def printMarkerQuality(self, quality):
        stars = ""        
        if(quality > 0.5):
            stars = "**"
        if(quality > 0.7):
            stars = "***"
        if(quality > 0.9):
            stars = "****"
        print("quality: %5.2f %s" % (quality, stars))
        
    def limitAngleToRange(self, angle):
        while(angle < math.pi):
            angle += 2*math.pi
        while(angle > math.pi):
            angle -= 2*math.pi
        return angle