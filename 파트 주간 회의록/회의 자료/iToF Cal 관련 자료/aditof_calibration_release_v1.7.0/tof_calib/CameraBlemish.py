#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#

import numpy as np
import sys
from . import Blemish_functions as blemish
from . import CalibAPI as tof
import time
import matplotlib.pyplot as plt
import pickle
import os
from datetime import datetime
import logging
import yaml
import cv2
import pandas as pd


def blemish_test(config_file=None, CalibfolderName = ""):
	'''
		Top function for Geometric calibration
		
		Inputs:
		  config_file           - name of configuration (.yaml) file
		  CalibfolderName       - name of calbration folder
		  
		Outputs:
		  None
		
	'''
	with open(config_file) as y:
		file = yaml.load(y, Loader=yaml.SafeLoader)

	if CalibfolderName == "":
		now = datetime.now()
		folderName = file['Blemish']['output_directory'] + '_' + str(now.strftime('%Y%m%d_%H%M%S'))
	else:
		folderName = os.path.join(CalibfolderName, file['Blemish']['output_directory'][:-1])
	
	if not os.path.exists(folderName):
		os.mkdir(folderName)
	else:
		now = datetime.now()
		print('\nALERT: ' + folderName + ' already exists')
		old_folderName = folderName + str(now.strftime('_old_%Y%m%d_%H%M%S'))
		os.rename(folderName, old_folderName)
		print('ALERT: RENAMING OLD FOLDER: ' + folderName + ' as '+ old_folderName +'\n')
		os.mkdir(folderName)


	cfg_file = file['Blemish']['configuration_file']
	ini_file = file['Blemish']['ini_file']
	root_dir = file['Blemish']['root_dir']
	Nframes = file['Blemish']['Nframes']
	NWarmupFrames = file['Blemish']['NWarmupFrames']
	output_file_pkl = file['Blemish']['output_file_pkl']
	also_save_image_as_bin = file['Blemish']['also_save_image_as_bin']
	output_file_csv_report = file['Blemish']['output_file_csv_report']

	try:
		calibration_dir_path = file['calibration_directory']
	except:
		calibration_dir_path = ""
	
	print("\nTurning on camera...\n")
	module_class = file['module_class'] # MZ: Type of Imager Module (Walden_R1, Legacy, Ofilm)
	cam = tof.CalibAPI(module_class = module_class)
	cam.ConnectSensor(Mode=3, fps=10, cfg = os.path.join(calibration_dir_path, file['Blemish']['configuration_file']))

	ini_file = os.path.join(calibration_dir_path, ini_file)
	root_dir = os.path.join(calibration_dir_path, root_dir)

	# cam.LoadGainOffsetCalib(file['Blemish']['gainOffsetCorrection'])
	# Check if gain and offset calibration file is found
	gainOffsetFileName = str(os.path.join(CalibfolderName, 
										  file['GainOffsetCalibration']['output_directory'],
										  file['GainOffsetCalibration']['output_file_csv'])) + '.csv'
	if os.path.exists(gainOffsetFileName):
		print("\nGain and Offset file found\n")
		cam.LoadGainOffsetCalib(gainOffsetFileName)
	else:
		raise Exception('Gain and Offset calibration must be done before P0 Calibration')

	blemish_file = os.path.join(folderName, file['Blemish']['output_file_frames_pkl'])

	## First iteration
	hdrTemp = dict(
					mode = 3,
					nFrames = Nframes,
					nWarmupFrames = NWarmupFrames,
					OVERRIDE_LSDACS = False,
					LSDACValue = 108,
					enableSweep = 0,
					outputFormat = 'pkl',
					also_save_image_as_bin = also_save_image_as_bin,
					fileName = blemish_file + '.pkl'
					)

	_, frames = cam.CaptureFramesP0(hdrTemp)
	Img = frames['Image']
	Hdr = frames['Header']
	print('Frames Captured')
	avg_frame = np.mean(Img, axis = 3)
	avg_frame = np.int16(avg_frame - np.min(avg_frame))
	avg_frame2 = np.squeeze(avg_frame)

	#avg_frame2 = avg_frame2 / np.max(avg_frame2)
	#avg_frame2 = np.uint8(avg_frame * 255)
	avg_frame2 = np.uint8( (avg_frame2/np.amax(avg_frame2)*(255)))
	
	cv2.imwrite(os.path.join(folderName, 'Blemish_original.png'), avg_frame2)

	# blemish_ver_dict = blemish.BlemishProcess(avg_frame, root_dir, ini_file, folderName)
	results_folder = folderName + '/'
	blemish_ver_dict = blemish.BlemishProcess(avg_frame, root_dir, ini_file, results_folder)

	now = datetime.now()
	blemish_ver_dict['info'] = dict()
	blemish_ver_dict['info']['CheckSum'] = 0
	blemish_ver_dict['info']['Year'] = int(now.strftime('%Y'))
	blemish_ver_dict['info']['Month'] = int(now.strftime('%m'))
	blemish_ver_dict['info']['Day'] = int(now.strftime('%d'))
	blemish_ver_dict['info']['Hour'] = int(now.strftime('%H'))
	blemish_ver_dict['info']['Minute'] = int(now.strftime('%M'))
	blemish_ver_dict['info']['Second'] = int(now.strftime('%S'))
	blemish_ver_dict['info']['Pass'] = 1

	limits_file = file['limits_file']
	if os.path.abspath(limits_file) != limits_file:
		limits_file = os.path.join(calibration_dir_path, limits_file)
	try:
		with open(limits_file) as ly:
			limits = yaml.load(ly, Loader=yaml.SafeLoader)['BlemishCalibration']
	except:
		raise Exception("\n\nWARNING: Incorrect/non-existing limits file\n\n")

	blemish_ver_dict['Blemish_pass'] = True #Initialize to True

	nHotPixels = blemish_ver_dict['blemishResults']['nHotPixels'][0]
	if nHotPixels < limits['nHotPixels'][1]:
		nHotPixels_status = "PASSED"
	else:
		nHotPixels_status = "FAILED"
		print('\nHot pixels FAIL\n')
		blemish_ver_dict['Blemish_pass'] = False
	blemish_ver_dict['totalHotPixels'] = [nHotPixels, nHotPixels_status, limits['nHotPixels'][0], limits['nHotPixels'][1]]

	nDeadPixels = blemish_ver_dict['blemishResults']['nDeadPixels'][0]
	if nDeadPixels < limits['nDeadPixels'][1]:
		nDeadPixels_status = "PASSED"
	else:
		nDeadPixels_status = "FAILED"
		print('\nDead pixels FAIL\n')
		blemish_ver_dict['Blemish_pass'] = False
	blemish_ver_dict['totalDeadPixels'] = [nDeadPixels, nDeadPixels_status, limits['nDeadPixels'][0], limits['nDeadPixels'][1]]

	if blemish_ver_dict['blemishResults']['N_blemish_count'][0] > 0:
		totalBadPixels = sum(blemish_ver_dict['blemishResults']['blemishSizePxls'])
	else: 
		totalBadPixels = 0
	if totalBadPixels < limits['totalBadPixels'][1]:
		totalBadPixels_status = "PASSED"
	else:
		totalBadPixels_status = "FAILED"
		print('\nExceeded max number bad pixels - FAIL\n')
		blemish_ver_dict['Blemish_pass'] = False
	blemish_ver_dict['totalBadPixels'] = [totalBadPixels, totalBadPixels_status, limits['totalBadPixels'][0], limits['totalBadPixels'][1]]

	badPixelsCenter = blemish_ver_dict['blemishResults']['pixelErrors'][4]
	if badPixelsCenter < limits['badPixelsCenter'][1]:
		badPixelsCenter_status = "PASSED"
	else:
		badPixelsCenter_status = "FAILED"
		print('\nExceeded max number bad pixels in main portion of image - FAIL\n')
		blemish_ver_dict['Blemish_pass'] = False
	blemish_ver_dict['badPixelsCenter'] = [badPixelsCenter, badPixelsCenter_status, limits['badPixelsCenter'][0], limits['badPixelsCenter'][1]]

	badPixelsSides = sum(blemish_ver_dict['blemishResults']['pixelErrors'][1::2]) # element 1=Top, 3=Left, 5=Right, 7=Bottom
	if badPixelsSides < limits['badPixelsSides'][1]:
		badPixelsSides_status = "PASSED"
	else:
		badPixelsSides_status = "FAILED"
		print('\nExceeded max number bad pixels at periphery of image - FAIL\n')
		blemish_ver_dict['Blemish_pass'] = False
	blemish_ver_dict['badPixelsSides'] = [badPixelsSides, badPixelsSides_status, limits['badPixelsSides'][0], limits['badPixelsSides'][1]]

	blemish_ver_dict['info']['Pass'] = blemish_ver_dict['Blemish_pass']

	if blemish_ver_dict['info']['Pass'] == True:
		device_passed = "PASSED"
		print("\n====================")
		print('BLEMISH TEST PASSED')
		print("====================\n")
	else:
		device_passed = "FAILED"
		print("\n====================")
		print('BLEMISH TEST FAILED')
		print("====================\n")

	f = open(os.path.join(folderName, file['Blemish']['output_file_pkl']+'.pkl'), 'wb')
	pickle.dump(blemish_ver_dict, f)
	f.close()

	# Save results to CSV file 
	Blemish_status = dict()
	Blemish_status['Pass/Fail'] = [blemish_ver_dict['Blemish_pass'], device_passed]
	for key, value in blemish_ver_dict.items():
		if isinstance(blemish_ver_dict[key], (list,tuple,dict, str)):
			if len(blemish_ver_dict[key]) <= 4:
				Blemish_status[key] = value
	df = pd.DataFrame(dict([ (k,pd.Series(v)) for k,v in Blemish_status.items() ]))
	df = df.T
	df.columns = ['Value', 'Result', 'Min Limit', 'Max Limit']
	df.to_csv(os.path.join(folderName, output_file_csv_report + '.csv'))
