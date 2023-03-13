#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#

import tof_calib.intrinsic_calibration as ic
import glob
import cv2
import numpy as np
import os
import csv
from matplotlib import pyplot as plt

def GeometricCal(irImage, coords_file, folderName, intrinsics_dict):
    """
    Performs geometric calibration
    Inputs:
        yaml_file: input .yaml configuration file name
        folderName: foldername to store output
        intrinsics_dict: camera intrinsics dictionary

    Outputs:
        intrinsics_dict: updated camera intrinsics dictionaryy

    """
    #Initalize intrinsic calib class
    ic_handle = ic.intrinsic_calibration(height=1024,width=1024, cb_height=10, cb_width=10)

    imax = np.amax(irImage)
    irImage = np.uint8(255*(irImage/imax))
    print(irImage.shape)

    #Sharpen mask for clear image
    #filter=np.array([[-1,-1,-1],[-1,9,-1],[-1,-1,-1]])
    filter = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
    irImage = cv2.filter2D(irImage, -1, filter)
    cv2.imwrite(os.path.join(folderName, 'checkerboard_sharpened.png'), irImage)

    # Load saved coordinates from JSON file
    ic_handle.load_coordinates(coords_file)
    
    # Find checkerboards and corners
    cb_found = ic_handle.get_corners(irImage)
    if cb_found < 9:
        print("Number of checkerboards found is " + str(cb_found) + '/9')
        print("Please locate checkerboards manually")

        # Manually identify checkerboards and save new coordinates to file
        # Find checkerboard coordinates - Drag Rectangles on popup window

        #Reset coordinates
        ic_handle.clear()
    
        # Find 10x10 checkerboards
        ic_handle.cb_height = 10
        ic_handle.cb_width = 10
        rect_coordinates = ic_handle.get_coordinates(irImage)

        # Find 6x6 checkerboards
        ic_handle.cb_height = 6
        ic_handle.cb_width = 6
        rect_coordinates = ic_handle.get_coordinates(irImage)

        # Store coordinates in json
        #ic_handle.output_coordinates(folderName)
        ic_handle.output_coordinates("./config")

    #Provide initial guesses for algorithm
    guess_pp = 512 #center of pixel array
    # if yaml_file['module_class'] == 'Crosby':
    #     guess_fl = 785
    # else:
    #     guess_fl = 1173 #1173 corresponds to JCD focal length of 4.1mm
    guess_fl = 1173 #Crosby seems to work better with this initial guess also
    mtx_guess_in = np.array([[guess_fl,0.,guess_pp],[0.,guess_fl,guess_pp],[0.,0.,1]])
    for i in range(1):
        if i == 0:
            print("Intrinsics calc with guess")
            checkerboards_found, intrinsics, int_points= ic_handle.calibrate_intrinsic(mtx_guess=mtx_guess_in)
        if i == 1:
            print("Intrinsics calc with guess and 3 iterations")
            checkerboards_found, intrinsics, int_points= ic_handle.calibrate_intrinsic(3, mtx_guess=mtx_guess_in)
        
        print('Checkerboards Found:' + str(checkerboards_found))
        print('Reprojection Error:' + str(intrinsics[0]))
        
        print("fx: " + str(intrinsics[1][0][0]))
        print("fy: " + str(intrinsics[1][1][1]))
        print("cx: " + str(intrinsics[1][0][2]))
        print("cy: " + str(intrinsics[1][1][2]))
        print("k1: " + str(intrinsics[2][0][0]))
        print("k2: " + str(intrinsics[2][0][1]))
        print("k3: " + str(intrinsics[2][0][4]))
        if len(intrinsics[2][0]) > 5:
            print("k4: " + str(intrinsics[2][0][5]))
            print("k5: " + str(intrinsics[2][0][6]))
            print("k6: " + str(intrinsics[2][0][7]))
        else:
            print('N/A')
            print('N/A')
            print('N/A')
        print("p1: " + str(intrinsics[2][0][2]))
        print("p2: " + str(intrinsics[2][0][3]))
        
        print('')
        
    intrinsics_dict['Rerr'] = intrinsics[0]
    intrinsics_dict['Fc1'] = intrinsics[1][0][0]
    intrinsics_dict['Fc2'] = intrinsics[1][1][1]
    intrinsics_dict['cc1'] = intrinsics[1][0][2]
    intrinsics_dict['cc2'] = intrinsics[1][1][2]
    intrinsics_dict['Kc1'] = intrinsics[2][0][0]
    intrinsics_dict['Kc2'] = intrinsics[2][0][1]
    intrinsics_dict['Kc3'] = intrinsics[2][0][4]
    if len(intrinsics[2][0]) > 5:
        intrinsics_dict['Kc4'] = intrinsics[2][0][5]
        intrinsics_dict['Kc5'] = intrinsics[2][0][6]
        intrinsics_dict['Kc6'] = intrinsics[2][0][7]
    else:
        intrinsics_dict['Kc4'] = 0
        intrinsics_dict['Kc5'] = 0
        intrinsics_dict['Kc6'] = 0
    intrinsics_dict['Ty'] = intrinsics[2][0][2]
    intrinsics_dict['Tx'] = intrinsics[2][0][3]
    intrinsics_dict['Cy'] = 0
    intrinsics_dict['Cx'] = 0

    return intrinsics_dict