#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#

import numpy as np
import pandas as pd
import os
import time
from datetime import datetime
from matplotlib import pyplot as plt
from scipy.optimize import curve_fit
from shutil import copyfile
import re
import re
import itertools as it
import seaborn as sns
import logging
import cv2
import json
import logging
import random

class intrinsic_calibration:

    # Coordinates list
    rect_list = []
    
    # Click and crop variables
    refPt2 = []
    cropping = False
    imgNum = 0
    height = 480
    width = 640
    irClone = []
    irCrop = []
    cb_width = 9
    cb_height = 6
    
    # Calib output params
    ret = 0
    mtx = []
    dist = []
    rvecs = []
    tvecs = []
    
    # Data output options
    roi_storage = []
    
    def __init__(self, height=480, width=640, cb_width = 9, cb_height = 6):
        self.height = height
        self.width = width
        self.cb_width = cb_width
        self.cb_height = cb_height
        self.irClone = np.zeros((self.height, self.width), np.uint8)
        self.irCrop = np.zeros((self.height, self.width), np.uint8)
        roi_storage = []
        self.rect_list = []
        self.refPt2 = []
        self.objpoints = []
        self.imgpoints = []
        self.subpix_criteria = (cv2.TERM_CRITERIA_EPS, 30, 0.001)
        self.c_size = []

    def clear(self):
        """
        clears variables
        Inputs:
            none

        Outputs:
            none

        """
        self.rect_list = []
        self.c_size = []
        self.objpoints = []
        self.imgpoints = []
        
    def click_and_crop(self, event, x, y, flags, param):
        """
        function to click and crop to find corners
        Inputs:
            event: click event
            x: x coordinate
            y: y coordinate
            flags: opencv flags
            param: imput parameters

        Outputs:
            none

        """
        if event == cv2.EVENT_LBUTTONDOWN:
            self.refPt2.append((x, y))
            cropping = True
            
        if event == cv2.EVENT_RBUTTONDOWN:
            self.refPt2.append((x, y))
            cropping = True        
        
        elif event == cv2.EVENT_LBUTTONUP:
            print('Capture ' + str(self.cb_width) + ' by ' + str(self.cb_height) + ' checkerboard.') 
            refPt = self.refPt2
            refPt.append((x, y))
            print(refPt)
            cropping = False
            
            # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
            objp = np.zeros((self.cb_width*self.cb_height,3), np.float32)
            objp[:,:2] = np.mgrid[0:self.cb_width,0:self.cb_height].T.reshape(-1,2)
                
            temp = np.zeros((self.height, self.width), np.uint8)
            temp[refPt[0][1]:refPt[1][1], refPt[0][0]:refPt[1][0]] = self.irClone[refPt[0][1]:refPt[1][1], refPt[0][0]:refPt[1][0]]
            ret, centers = cv2.findChessboardCorners(temp, (self.cb_width,self.cb_height), flags=cv2.CALIB_CB_ADAPTIVE_THRESH)
            if ret == True:
                print("Grid Found")
                #Append roi coordinates
                self.rect_list.append((refPt[0], refPt[1]))
                self.c_size.append((self.cb_width,self.cb_height))

                # Add corner points to obj and img points list 
                self.objpoints.append(objp)
                corners = cv2.cornerSubPix(temp,centers,(self.cb_width,self.cb_height),(-1,-1),self.subpix_criteria)
                self.imgpoints.append(corners)
                
                # draw a rectangles around the region of interests
                self.irCrop = np.ascontiguousarray(self.irCrop, dtype=np.uint8)
                cv2.rectangle(self.irCrop, refPt[0], refPt[1], (255, 255, 255), 2)
                #cv2.rectangle(self.irCrop, refPt[0], refPt[1], (0, 255, 0), 2)
                cv2.imshow("cropIR", self.irCrop)
                self.imgNum = self.imgNum + 1
                
            self.refPt2 = []
        
        #Switch checkerboard width and height
        elif event == cv2.EVENT_RBUTTONUP:
            refPt = self.refPt2
            refPt.append((x, y))
            #print(refPt)
            cropping = False
            
            temp = np.zeros((self.height, self.width), np.uint8)
            temp[refPt[0][1]:refPt[1][1], refPt[0][0]:refPt[1][0]] = self.irClone[refPt[0][1]:refPt[1][1], refPt[0][0]:refPt[1][0]]
            ret, centers = cv2.findChessboardCorners(temp, (self.cb_width,self.cb_height), flags=cv2.CALIB_CB_ADAPTIVE_THRESH)
            if ret == True:
                print("Grid Found")
                self.rect_list.append((refPt[0], refPt[1]))
                self.c_size.append((self.cb_width,self.cb_height))
                # draw a rectangles around the region of interests
                self.irCrop = np.ascontiguousarray(self.irCrop, dtype=np.uint8)
                cv2.rectangle(self.irCrop, refPt[0], refPt[1], (255, 255, 255), 2)
                #cv2.rectangle(self.irCrop, refPt[0], refPt[1], (0, 255, 0), 2)
                cv2.imshow("cropIR", self.irCrop)
                self.imgNum = self.imgNum + 1
                
            self.refPt2 = []

    def get_coordinates(self, irImage):
        """
        Get coordinates list
        Inputs:
            irImage: passiveIR frames

        Outputs:
            rect_list: list of coordinates

        """
        self.irClone = irImage.copy()
        self.irCrop = irImage.copy()
        
        cv2.namedWindow("cropIR")
        cv2.setMouseCallback("cropIR", self.click_and_crop)
        while True:
            # display the image and wait for a keypress
            cv2.imshow("cropIR", self.irCrop)
            key = cv2.waitKey(1) & 0xFF

            # if the 'r' key is pressed, reset the cropping region
            if key == ord("r"):
                self.irCrop = self.irClone.copy()
                self.rect_list = []

            # if the 'c' key is pressed, break from the loop
            elif key == ord("c"):
                cv2.destroyAllWindows()
                break
                
        return self.rect_list
                
    def output_coordinates(self, path):
        """
        Outputs coordinates to a json file
        Inputs:
            path: filename

        Outputs:
            none

        """
        dict = {}
        dict['coordinates'] = self.rect_list
        dict['checkerboard_size'] = self.c_size
        
        with open(os.path.join(path, 'intrinsic_coordinates.json'), 'w') as fp:
            json.dump(dict, fp)
            
    def load_coordinates(self, fname):
        """
        Loads coordinates from a json file
        Inputs:
            fname: json file name

        Outputs:
            none

        """
        with open(fname, 'r') as fp:
            dict = json.loads(fp.read())
            self.rect_list = dict['coordinates']
            self.c_size = dict['checkerboard_size']
        return self.rect_list, self.c_size

    def jiggle(self, coor, x, y):
        """
        add a small jiggle to coordinates
        Inputs:
            coor: coordinate list
            x: jiggle in x
            y: jiggle in y

        Outputs:
            coor: updated coordinate list

        """
        xj = random.randrange(-x,x+1,1)
        yj = random.randrange(-y,y+1,1)
        coor[0][0] = coor[0][0] + xj 
        coor[1][0] = coor[1][0] + xj
        coor[0][1] = coor[0][1] + yj
        coor[1][1] = coor[1][1] + yj
        
        if coor[0][0] < 0:
            coor[0][0] = 0
        if coor[0][1] < 0:
            coor[0][1] = 0
        if coor[1][0] > self.width:
            coor[1][0] = self.width-1
        if coor[1][1] > self.height:
            coor[1][1] = self.height

        return coor

    def get_corners(self, irImage, attempts = 5):
        """
        Find corners 
        Inputs:
            irImage: passiveIR image
            attempts: attempt count

        Outputs:
            cb_found: Whether checkerboard pattern is found

        """
        self.irClone = irImage.copy()
        cb_found = 0

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        for i, j in zip(self.c_size, self.rect_list):
            self.cb_width = i[0]
            self.cb_height = i[1]

            coor = j

            objp = np.zeros((self.cb_width*self.cb_height,3), np.float32)
            objp[:,:2] = np.mgrid[0:self.cb_width,0:self.cb_height].T.reshape(-1,2)
            
            tries = 0
            while(tries < attempts):
                temp = np.zeros((self.height, self.width), np.uint8)
                temp[coor[0][1]:coor[1][1], coor[0][0]:coor[1][0]] = self.irClone[coor[0][1]:coor[1][1], coor[0][0]:coor[1][0]]
                ret, centers = cv2.findChessboardCorners(temp, (self.cb_width,self.cb_height), flags=cv2.CALIB_CB_ADAPTIVE_THRESH)
                if ret == True:
                    cb_found = cb_found + 1
                    # Add corner points to obj and img points list 
                    self.objpoints.append(objp)
                    corners = cv2.cornerSubPix(temp,centers,(self.cb_width,self.cb_height),(-1,-1),self.subpix_criteria)
                    self.imgpoints.append(corners)
                    print('Chessboard Found : Tries - ' + str(tries))
                    tries = attempts
                else:
                    plt.figure(figsize=(12,12))
                    plt.imshow(temp)
                    print('Jiggling')
                    print(coor)                    
                    coor = self.jiggle(j, 20, 20)
                    tries = tries + 1      

        print('Checkerboards Found: ' + str(cb_found))

        return cb_found

    def get_roi_images(self, irImage):
        """
        Get ROI images
        Inputs:
            irImage: passive IR image

        Outputs:
            roi_storage: store ROI 

        """
        #logger = logging.getLogger(__name__)
        self.irClone = irImage.copy()
        
        # Store rois and cornerpoints
        self.roi_storage = np.zeros((self.height, self.width, len(self.rect_list)))
        self.center_points = []
        for idx, i in enumerate(self.rect_list): 
            roi = np.zeros((self.height, self.width), np.uint8)
            roi[i[0][1]:i[1][1], i[0][0]:i[1][0]] = self.irClone[i[0][1]:i[1][1], i[0][0]:i[1][0]]
            self.roi_storage[:,:,idx] = roi
        
        return self.roi_storage
        
    
    def calibrate_intrinsic(self, iterate=0, mtx_guess = None, rational_model=True, imgpoints =None, objpoints=None, shape=None):
        """
        run instrinsic calibration/ geometric calibration 
        Inputs:
            iterate: number of iterations
            mtx_guess: initial mtx value
            rational_model: opencv model
            imgpoints: imagepoints
            objpoins: objectpoints
            shape: shape of image

        Outputs:
            0
            roi_storage: roi values
            center_points: coordinates of center of checkerboard

        """
        if imgpoints is None:
            imgpoints = self.imgpoints
        if objpoints is None:
            objpoints = self.objpoints
        if shape is None:
            shape = self.irClone.shape[::-1]
        # Calibrate Camera
        criteria2 = (cv2.TERM_CRITERIA_EPS, 1000, 0.001)
        flags = 0
        if rational_model:
            flags = flags + cv2.CALIB_RATIONAL_MODEL
        if mtx_guess is None:
            mtx_guess = np.array([[0.,0.,0.],[0.,0.,0.],[0.,0.,0.]])
        else:
            flags = flags + cv2.CALIB_USE_INTRINSIC_GUESS
        if True:
            self.ret, self.mtx, self.dist, self.rvecs, self.tvecs = cv2.calibrateCamera(objpoints, imgpoints, shape, mtx_guess, None, None, None, flags, criteria=criteria2)
            
            # Run calibration again if iterate is specified
            flags = (cv2.CALIB_RATIONAL_MODEL+cv2.CALIB_USE_INTRINSIC_GUESS)
            for i in range(iterate):
                self.ret, self.mtx, self.dist, self.rvecs, self.tvecs = cv2.calibrateCamera(objpoints, imgpoints, shape, mtx_guess, None, None, None, flags, criteria=criteria2)
            
            return len(objpoints), [self.ret, self.mtx, self.dist, self.rvecs, self.tvecs], [objpoints, imgpoints]
        
        
        return 0, [], self.roi_storage, self.center_points
        
    def output_intrinsic(self, path, sn):
        """
        Outputs instricsics to a json file
        Inputs:
            path: filename
            sn: calibration station name

        Outputs:
            none

        """
        dict = {}
        mtx = np.append(self.mtx[0], np.append(self.mtx[1], self.mtx[2])).tolist()
        print(mtx)
        
        dict["Calibration_SN"] = {"2" : sn}
        dict["Calibration_Date_stamp"] = {"3" : datetime.now().strftime('%m%d%Y')}
        dict["Intrinsic"] = {"5": mtx}
        dict["Distortion_Coeff"] = {"6": self.dist[0].tolist()}
        

        os.makedirs(path, exist_ok=True)

        with open(os.path.join(path, 'Camera_Intrinsic.json'), 'w') as fp:
            json.dump(dict, fp)
    
    # Must be called right after calibrate_intrinsic for accurate info
    def output_intrinsic_data(self, output_path):
        """
        Outputs instinsic data
        Inputs:
            output_path: path of filename

        Outputs:
            none

        """
        
        folder_name = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        path = os.path.join(output_path, 'intrinsic_data', folder_name)
        
        irImage_path = os.path.join(path, 'irImage.png') 
        cv2.imwrite(irImage_path, self.irClone)
        
        for idx, data in enumerate(range(self.roi_storage[-1])):
            roi_path = os.path.join(path, 'roi' + str(idx) + '.png') 
            cv2.imwrite(roi_path, data)
            
        self.output_intrinsic(path)
        self.output_coordinates(path)
