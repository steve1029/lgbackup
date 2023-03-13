#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#

import cv2
import numpy as np
import sys
# try:
from . import CalibAPI as tof
# except:
# 	import CalibAPI as tof
# try:
# from . import P0_functions as P0

# except:
# 	import P0_functions as P0
import time
import matplotlib.pyplot as plt
import pickle
import os
from datetime import datetime
import logging
import yaml
import shutil

def AB_stream(config_file=None, CalibfolderName = ""):
	'''
		Top function for AB stream
		
		Inputs:
		  config_file           - name of configuration (.yaml) file
		  CalibfolderName       - name of calbration folder
		  
		Outputs:
		  None
		
	'''

	with open(config_file) as y:
		file = yaml.load(y, Loader=yaml.SafeLoader)

	cal_settings = file['P0Calibration']['calibration']
	verification_settings = file['P0Calibration']['verification']

	if CalibfolderName == "":
		now = datetime.now()
		folderName = 'AB_stream_' + str(now.strftime('%Y%m%d_%H%M%S'))
	else:
		folderName = os.path.join(CalibfolderName, 'AB_stream')
	
	if not os.path.exists(folderName):
		os.mkdir(folderName)
	else:
		now = datetime.now()
		print('\nALERT: ' + folderName + ' already exists')
		old_folderName = folderName + str(now.strftime('_old_%Y%m%d_%H%M%S'))
		os.rename(folderName, old_folderName)
		print('ALERT: RENAMING OLD FOLDER: ' + folderName + ' as '+ old_folderName +'\n')
		os.mkdir(folderName)

	#mode = cal_settings['mode']
	mode = [5]
	fps = cal_settings['fps']

	try:
		n_modes_cfg = file['num_modes_cfg']
	except:
		print("\n\nWARNING: n_modes_cfg not found in yaml file, update to latest version of yaml file")
		print("Assuming n_modes_cfg = 10\n\n")
		n_modes_cfg = 10

	verify_OVERRIDE_LSDAC = verification_settings['verify_OVERRIDE_LSDACS']
	verify_LSDAC_VALUE = verification_settings['verify_LSDAC_VALUE']
	module_class = file['module_class'] # MZ: Type of Imager Module (Walden_R1, Legacy, Ofilm)

	also_save_image_as_bin = file['P0Calibration']['also_save_image_as_bin']

	y.close()

	if (5 in mode) and (10 in mode):
		#mode = mode[mode != 10]
		mode.remove(10)
		mode10_no_capture = True # Not capturing mode 10 to save time
	else:
		mode10_no_capture = False

	prelimTestResults = dict()
	prelimTestResults['All_pass'] = True

	try:
		calibration_dir_path = file['calibration_directory']
	except:
		calibration_dir_path = ""

	for m in range(len(mode)):
		cam = tof.CalibAPI(module_class = module_class, n_modes_cfg = n_modes_cfg)
		cam.ConnectSensor(Mode=mode[m], fps=fps, cfg = os.path.join(calibration_dir_path,file['P0Calibration']['configuration_file']))

		print("USING CFG FILE :", file['P0Calibration']['configuration_file'])
		
		# Check if gain and offset calibration file is found
		gainOffsetFileName = str(os.path.join(CalibfolderName, 
											  file['GainOffsetCalibration']['output_directory'],
											  file['GainOffsetCalibration']['output_file_csv'])) + '.csv'
		if os.path.exists(gainOffsetFileName):
			print("\nGain and Offset file found\n")
			cam.LoadGainOffsetCalib(gainOffsetFileName)
		else:
			raise Exception('Gain and Offset calibration must be done before P0 Calibration')
		
		hdrTemp = dict(
					mode = mode[m],
					# nFrames = verify_NFrames[m],
					# nWarmupFrames = verify_NFrames_warmup[m],
					OVERRIDE_LSDACS = verify_OVERRIDE_LSDAC[m],
					LSDACValue = verify_LSDAC_VALUE[m],
					enableSweep = 0, # False
					outputFormat = 'pkl',
					also_save_image_as_bin = also_save_image_as_bin,
					folderName = folderName
					)

		_, test_frames = cam.CaptureFramesAB(hdrTemp)

		print('AB Frames Captured')

	cam.DisconnectSensor()
	del cam

if __name__ == "__main__":
	AB_stream("./config/CalibrationConfig.yaml" , CalibfolderName = "")


def passiveIR_stream(config_file=None, CalibfolderName = ""):
	'''
		Top function for passive IR stream
		
		Inputs:
		  config_file           - name of configuration (.yaml) file
		  CalibfolderName       - name of calbration folder
		  
		Outputs:
		  None
		
	'''

	with open(config_file) as y:
		file = yaml.load(y, Loader=yaml.SafeLoader)

	calibration_mode = file['GeometricCalibration']['calibration_mode']

	try:
		calibration_dir_path = file['calibration_directory']
	except:
		calibration_dir_path = ""

	if CalibfolderName == "":
		now = datetime.now()
		folderName = 'passiveIR_stream_' + str(now.strftime('%Y%m%d_%H%M%S'))
	else:
		folderName = os.path.join(CalibfolderName, 'passiveIR_stream')
	
	if not os.path.exists(folderName):
		os.mkdir(folderName)
	else:
		now = datetime.now()
		print('\nALERT: ' + folderName + ' already exists')
		old_folderName = folderName + str(now.strftime('_old_%Y%m%d_%H%M%S'))
		os.rename(folderName, old_folderName)
		print('ALERT: RENAMING OLD FOLDER: ' + folderName + ' as '+ old_folderName +'\n')
		os.mkdir(folderName)


	print("\nTurning on camera...\n")
	module_class = file['module_class'] # MZ: Type of Imager Module (Walden_R1, Toro, Ofilm)
	now = datetime.now()
	geo_dict = dict()
	geo_dict['info'] = dict()
	geo_dict['info']['CheckSum'] = 0
	geo_dict['info']['Year'] = int(now.strftime('%Y'))
	geo_dict['info']['Month'] = int(now.strftime('%m'))
	geo_dict['info']['Day'] = int(now.strftime('%d'))
	geo_dict['info']['Hour'] = int(now.strftime('%H'))
	geo_dict['info']['Minute'] = int(now.strftime('%M'))
	geo_dict['info']['Second'] = int(now.strftime('%S'))
	geo_dict['info']['configuration_file'] = file['GeometricCalibration']['configuration_file']
	geo_dict['info']['Pass'] = 0

	cam = tof.CalibAPI(module_class = module_class)
	cam.ConnectSensor(Mode=3, fps=10, cfg = os.path.join(calibration_dir_path, file['GeometricCalibration']['configuration_file']))

	# cam.LoadGainOffsetCalib(file['GeometricCalibration']['gainOffsetCorrection'])
	# Check if gain and offset calibration file is found
	gainOffsetFileName = str(os.path.join(CalibfolderName, 
											file['GainOffsetCalibration']['output_directory'],
											file['GainOffsetCalibration']['output_file_csv'])) + '.csv'
	if os.path.exists(gainOffsetFileName):
		print("\nGain and Offset file found\n")
		cam.LoadGainOffsetCalib(gainOffsetFileName)
	else:
		raise Exception('Gain and Offset calibration must be done before P0 Calibration')
		

	cb_file = os.path.join(folderName, file['GeometricCalibration']['output_file_frames_pkl'])

	also_save_image_as_bin = file['GeometricCalibration']['also_save_image_as_bin']
	hdrTemp = dict(
					nFrames = int(file['GeometricCalibration']['Nframes']),
					outputFormat = 'pkl',
					also_save_image_as_bin = also_save_image_as_bin,
					fileName = cb_file + '.pkl'
					)

	#Img, Hdr = cam.StreamAndCapturePCM(hdrTemp)
	_, frames = cam.StreamAndCapturePCM(hdrTemp)
	Img = frames['Image']
	Hdr = frames['Header']
	print('Frames Captured')
	cam.DisconnectSensor()
	avg_frame = np.mean(Img, axis = 3)
	avg_frame = np.float32(np.squeeze(avg_frame))
	# print(np.max(avg_frame),np.min(avg_frame))
	avg_frame = (avg_frame - np.min(avg_frame))
	avg_frame = avg_frame / np.max(avg_frame)
	avg_frame2 = np.uint8(avg_frame * 255)
	cv2.imwrite(os.path.join(folderName, 'passiveIR.png'), avg_frame2)

	del cam
