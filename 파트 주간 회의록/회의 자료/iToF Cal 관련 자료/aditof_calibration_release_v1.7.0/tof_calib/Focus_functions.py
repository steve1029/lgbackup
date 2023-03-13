#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#

import cv2
import numpy as np
import sys
import time
import os
import json
from imatest.it import ImatestLibrary, ImatestException
import json

# Initialize Imatest Library
imatestLib = ImatestLibrary()

def FocusMeasure(avgFrames, adjust, root_dir, ini_file, raw_file_path):
    '''
    Function to measure focus. Requires Imatest to be installed
        
    Inputs:
        avgFrames: Averaged passive IR frame taken of the blemish setup
        adjust: if to adjust lens
        root_dir: root directory of .ini files
        ini_file: location of CenterMTF.ini/ Full_ROI_3D-plots.ini file
        raw_file_path: location on where to store results

    Outputs:
        json_result: result of blemish test in a dictionary
        
    '''

    raw_data = None

    if '_tempCalFiles' in os.listdir():
        pass
    else:
        os.mkdir('_tempCalFiles')

    BaseFile = os.path.join('_tempCalFiles', 'tmp_intermediateFile_focus')

    np.save(BaseFile,avgFrames) #Passing data directly
    with open(BaseFile+'.npy', 'rb') as fsock:  #Passing data directly
        raw_data = fsock.read()

    try:
        json_args = imatestLib.build_json_args(width=1024,
                                        height=1024,
                                        ncolors=1,
                                        extension='npy',
                                        filename=raw_file_path)   
    
        result = imatestLib.sfrplus(input_file=None,
                                    root_dir=root_dir,
                                    op_mode=ImatestLibrary.OP_MODE_DIRECT_READ,
                                    ini_file=ini_file,
                                    raw_data=raw_data[128:],
                                    json_args=json_args)

        json_result = json.loads(result)

        if adjust == True:
            print('MTF30 = ' + str(json_result['sfrplusResults']['mtf30']))
            multiMTF = json_result['sfrplusResults']['mtf30']
            centerMTF = multiMTF[0]
            averagePrtWay = sum(multiMTF[1:5])/4
            WeightCenter = 1
            WeightPartWay = 0.75
            wtdMean = ((centerMTF*WeightCenter) + (averagePrtWay*WeightPartWay))/(WeightCenter+WeightPartWay)
            print('center MTF30 = ' + str(centerMTF))
            print('Avg Part-way = ' + str(averagePrtWay))
            print('Wtd mean = ' + str(wtdMean))

    except ImatestException as iex:
        json_result = None
        if iex.error_id == ImatestException.FloatingLicenseException:
            print("All floating license seats are in use.  Exit Imatest on another computer and try again.")
        elif iex.error_id == ImatestException.LicenseException:
            print("License Exception: " + iex.message)
        else:
            print(iex.message)
    except Exception as ex:
        json_result = None
        print(str(ex))

    #imatestLib.terminate_library()

    return json_result

  

