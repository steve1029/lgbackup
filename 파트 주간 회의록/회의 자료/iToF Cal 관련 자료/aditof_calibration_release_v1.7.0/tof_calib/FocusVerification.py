#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#

import numpy as np
import sys
from . import Focus_functions as focus
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

#import ftd2xx as ftd
import lamp_control.IR_Lamp_Control as ir_lamp_control

def focus_verify(config_file=None, CalibfolderName = ""):
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

	#ADI IR lamp setup
	if file['FocusVerification']['lamp_control'] == True:
		irl, id_check = ir_lamp_control.lamp_setup()

	if CalibfolderName == "":
		now = datetime.now()
		folderName = file['FocusVerification']['output_directory'][:-1] + '_' + str(now.strftime('%Y%m%d_%H%M%S'))
	else:
		folderName = os.path.join(CalibfolderName, file['FocusVerification']['output_directory'][:-1])
	
	if not os.path.exists(folderName):
		os.mkdir(folderName)
	else:
		now = datetime.now()
		print('\nALERT: ' + folderName + ' already exists')
		old_folderName = folderName + str(now.strftime('_old_%Y%m%d_%H%M%S'))
		os.rename(folderName, old_folderName)
		print('ALERT: RENAMING OLD FOLDER: ' + folderName + ' as '+ old_folderName +'\n')
		os.mkdir(folderName)


	cfg_file = file['FocusVerification']['configuration_file']
	ini_file = file['FocusVerification']['ini_file']
	root_dir = file['FocusVerification']['root_dir']
	Nframes = file['FocusVerification']['Nframes']
	NWarmupFrames = file['FocusVerification']['NWarmupFrames']
	output_file_pkl = file['FocusVerification']['output_file_pkl']
	also_save_image_as_bin = file['FocusVerification']['also_save_image_as_bin']
	output_file_csv_report = file['FocusVerification']['output_file_csv_report']

	try:
		calibration_dir_path = file['calibration_directory']
	except:
		calibration_dir_path = ""
	
	print("\nTurning on camera...\n")
	module_class = file['module_class'] # MZ: Type of Imager Module (Walden_R1, Toro, Ofilm)
	cam = tof.CalibAPI(module_class = module_class)
	cam.ConnectSensor(Mode=3, fps=10, cfg = os.path.join(calibration_dir_path,file['FocusVerification']['configuration_file']))

	ini_file = os.path.join(calibration_dir_path, ini_file)
	root_dir = os.path.join(calibration_dir_path, root_dir)

	# cam.LoadGainOffsetCalib(file['FocusVerification']['gainOffsetCorrection'])
	# Check if gain and offset calibration file is found
	gainOffsetFileName = str(os.path.join(CalibfolderName, 
										  file['GainOffsetCalibration']['output_directory'],
										  file['GainOffsetCalibration']['output_file_csv'])) + '.csv'
	if os.path.exists(gainOffsetFileName):
		print("\nGain and Offset file found\n")
		cam.LoadGainOffsetCalib(gainOffsetFileName)
	else:
		raise Exception('Gain and Offset calibration must be done before P0 Calibration')

	focus_file = os.path.join(folderName, file['FocusVerification']['output_file_frames_pkl'])

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
					fileName = focus_file + '.pkl'
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
	cv2.imwrite(os.path.join(folderName, 'focus_verification.png'), avg_frame2)

	results_folder = folderName + '/'

	try:
		focus_ver_dict = focus.FocusMeasure(avg_frame, False, root_dir, ini_file, results_folder)

		now = datetime.now()
		focus_ver_dict['info'] = dict()
		focus_ver_dict['info']['CheckSum'] = 0
		focus_ver_dict['info']['Year'] = int(now.strftime('%Y'))
		focus_ver_dict['info']['Month'] = int(now.strftime('%m'))
		focus_ver_dict['info']['Day'] = int(now.strftime('%d'))
		focus_ver_dict['info']['Hour'] = int(now.strftime('%H'))
		focus_ver_dict['info']['Minute'] = int(now.strftime('%M'))
		focus_ver_dict['info']['Second'] = int(now.strftime('%S'))
		focus_ver_dict['info']['configuration_file'] = cfg_file


		limits_file = file['limits_file']
		if os.path.abspath(limits_file) != limits_file:
			limits_file = os.path.join(calibration_dir_path, limits_file)
		try:
			with open(limits_file) as ly:
				limits = yaml.load(ly, Loader=yaml.SafeLoader)['FocusVerification']
		except:
			raise Exception("\n\nWARNING: Incorrect/non-existing limits file\n\n")

		focus_ver_dict['Focus_pass'] = True #Initialize to True

		sfrplusResults = focus_ver_dict['sfrplusResults']
		if sfrplusResults['mtf50'][0] > limits['MTF50Center'][0]:
			MTF50Center_status = "PASSED"
		else:
			MTF50Center_status = "FAILED"
			print('\nMTF50 at center of chart FAIL\n')
			focus_ver_dict['Focus_pass'] = False
		focus_ver_dict['MTF50Center'] = [sfrplusResults['mtf50'][0], MTF50Center_status, limits['MTF50Center'][0], limits['MTF50Center'][1]]

		if sfrplusResults['mtf30'][0] > limits['MTF30Center'][0]:
			MTF30Center_status = "PASSED"
		else:
			MTF30Center_status = "FAILED"
			print('\nMTF30 at center of chart FAIL\n')
			focus_ver_dict['Focus_pass'] = False
		focus_ver_dict['MTF30Center'] = [sfrplusResults['mtf30'][0], MTF30Center_status, limits['MTF30Center'][0], limits['MTF30Center'][1]]

		if sfrplusResults['secondary_2_results'][0] > limits['MTF0p5NyqCenter'][0]:
			MTF0p5NyqCenter_status = "PASSED"
		else:
			MTF0p5NyqCenter_status = "FAILED"
			print('\nMTF Nyq/2 at center of chart FAIL\n')
			focus_ver_dict['Focus_pass'] = False
		focus_ver_dict['MTF0p5NyqCenter'] = [sfrplusResults['secondary_2_results'][0], MTF0p5NyqCenter_status, limits['MTF0p5NyqCenter'][0], limits['MTF0p5NyqCenter'][1]]

		if sfrplusResults['secondary_1_summary'][5] > limits['MTF30WtdMean'][0]:
			MTF30WtdMean_status = "PASSED"
		else:
			MTF30WtdMean_status = "FAILED"
			print('\nWeighted mean MTF30 FAIL\n')
			focus_ver_dict['Focus_pass'] = False
		focus_ver_dict['MTF30WtdMean'] = [sfrplusResults['secondary_1_summary'][5], MTF30WtdMean_status, limits['MTF30WtdMean'][0], limits['MTF30WtdMean'][1]]

		focus_ver_dict['info']['Pass'] = focus_ver_dict['Focus_pass']


	except:
		now = datetime.now()
		focus_ver_dict['info'] = dict()
		focus_ver_dict['info']['CheckSum'] = 0
		focus_ver_dict['info']['Year'] = int(now.strftime('%Y'))
		focus_ver_dict['info']['Month'] = int(now.strftime('%m'))
		focus_ver_dict['info']['Day'] = int(now.strftime('%d'))
		focus_ver_dict['info']['Hour'] = int(now.strftime('%H'))
		focus_ver_dict['info']['Minute'] = int(now.strftime('%M'))
		focus_ver_dict['info']['Second'] = int(now.strftime('%S'))
		focus_ver_dict['info']['configuration_file'] = cfg_file
		focus_ver_dict['info']['Pass'] = 0
		raise Exception("\n\nERROR: Focus verification failed\n\n")

	if focus_ver_dict['info']['Pass'] == True:
		device_passed = "PASSED"
		print("\n=================")
		print('FOCUS VER PASSED')
		print("=================\n")
	else:
		device_passed = "FAILED"
		print("\n=================")
		print('FOCUS VER FAILED')
		print("=================\n")

	f = open(os.path.join(folderName, file['FocusVerification']['output_file_pkl']+'.pkl'), 'wb')
	pickle.dump(focus_ver_dict, f)
	f.close()

	#ADI IR lamp setup
	if file['FocusVerification']['lamp_control'] == True:
		ir_lamp_control.lamp_shutdown(irl, id_check)

	# Save results to CSV file 
	Focus_status = dict()
	Focus_status['Pass/Fail'] = [focus_ver_dict['Focus_pass'], device_passed]
	for key, value in focus_ver_dict.items():
		if isinstance(focus_ver_dict[key], (list,tuple,dict, str)):
			if len(focus_ver_dict[key]) <= 4:
				Focus_status[key] = value
	df = pd.DataFrame(dict([ (k,pd.Series(v)) for k,v in Focus_status.items() ]))
	df = df.T
	df.columns = ['Value', 'Result', 'Min Limit', 'Max Limit']
	df.to_csv(os.path.join(folderName, output_file_csv_report + '.csv'))