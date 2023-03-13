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

#import ftd2xx as ftd
import lamp_control.IR_Lamp_Control as ir_lamp_control


def focus_adjust(config_file=None, CalibfolderName = ""):
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
	if file['FocusAdjustment']['lamp_control'] == True:
		irl, id_check = ir_lamp_control.lamp_setup()

	if CalibfolderName == "":
		now = datetime.now()
		folderName = file['FocusAdjustment']['output_directory'][:-1] + '_' + str(now.strftime('%Y%m%d_%H%M%S'))
	else:
		folderName = os.path.join(CalibfolderName, file['FocusAdjustment']['output_directory'][:-1])
	
	if not os.path.exists(folderName):
		os.mkdir(folderName)
	else:
		now = datetime.now()
		print('\nALERT: ' + folderName + ' already exists')
		old_folderName = folderName + str(now.strftime('_old_%Y%m%d_%H%M%S'))
		os.rename(folderName, old_folderName)
		print('ALERT: RENAMING OLD FOLDER: ' + folderName + ' as '+ old_folderName +'\n')
		os.mkdir(folderName)


	cfg_file = file['FocusAdjustment']['configuration_file']
	ini_file = file['FocusAdjustment']['ini_file']
	root_dir = file['FocusAdjustment']['root_dir']
	Nframes = file['FocusAdjustment']['Nframes']
	InitWarmupFrames = file['FocusAdjustment']['InitWarmupFrames']
	NWarmupFrames = file['FocusAdjustment']['NWarmupFrames']
	output_file_pkl = file['FocusAdjustment']['output_file_pkl']
	also_save_image_as_bin = file['FocusAdjustment']['also_save_image_as_bin']

	try:
		calibration_dir_path = file['calibration_directory']
	except:
		calibration_dir_path = ""

	print("\nTurning on camera...\n")
	module_class = file['module_class'] # MZ: Type of Imager Module (Walden_R1, Legacy, Ofilm)
	cam = tof.CalibAPI(module_class = module_class)
	cam.ConnectSensor(Mode=3, fps=10, cfg = os.path.join(calibration_dir_path,file['FocusAdjustment']['configuration_file']))

	ini_file = os.path.join(calibration_dir_path, ini_file)
	root_dir = os.path.join(calibration_dir_path, root_dir)

	# cam.LoadGainOffsetCalib(file['FocusAdjustment']['gainOffsetCorrection'])
	# Check if gain and offset calibration file is found
	gainOffsetFileName = str(os.path.join(CalibfolderName, 
										  file['GainOffsetCalibration']['output_directory'],
										  file['GainOffsetCalibration']['output_file_csv'])) + '.csv'
	if os.path.exists(gainOffsetFileName):
		print("\nGain and Offset file found\n")
		cam.LoadGainOffsetCalib(gainOffsetFileName)
	else:
		raise Exception('Gain and Offset calibration must be done before P0 Calibration')

	focus_file = os.path.join(folderName, file['FocusAdjustment']['output_file_frames_pkl'])

	## First iteration
	hdrTemp = dict(
					mode = 3,
					nFrames = Nframes,
					outputFormat = 'pkl',
					also_save_image_as_bin = also_save_image_as_bin,
					fileName = focus_file + '.pkl'
					)

	_, frames = cam.StreamAndCapturePCM(hdrTemp)
	Img = frames['Image']
	print('Frames Captured')
	avg_frame = np.mean(Img, axis = 3)
	avg_frame = np.int16(avg_frame - np.min(avg_frame))
	avg_frame2 = np.squeeze(avg_frame)

	# time.sleep(10)
	# print("done")

	#avg_frame2 = avg_frame2 / np.max(avg_frame2)
	#avg_frame2 = np.uint8(avg_frame * 255)
	avg_frame2 = np.uint8( (avg_frame2/np.amax(avg_frame2)*(255)))
	#cv2.imwrite(os.path.join(folderName, 'pre_focus_adjustment.png'), avg_frame2)

	results_folder = folderName + '/'
	focus_adj_dict = focus.FocusMeasure(avg_frame, True, root_dir, ini_file, results_folder)

	exit_p = False

	answ = input('Refocus? (Y/N): ')
	if (answ == 'N' or answ == 'n'):
		exit_p = True

	try:
		while exit_p == False:
			hdrTemp = dict(
							mode = 3,
							nFrames = Nframes,
							nWarmupFrames = NWarmupFrames,
							OVERRIDE_LSDACS = False,
							LSDACValue = 108,
							enableSweep = 0,
							outputFormat = 'pkl',
							fileName = focus_file + '.pkl',
							displayFrame = False,
							displayDuration = 4
							)

			_, frames_tmp = cam.CaptureFramesP0(hdrTemp)
			Img = frames_tmp['Image']
			print('Frames Captured')
			avg_frame = np.mean(Img, axis = 3)
			avg_frame = np.int16(avg_frame - np.min(avg_frame))
			avg_frame2 = np.squeeze(avg_frame)
			#avg_frame2 = avg_frame2 / np.max(avg_frame2)
			#avg_frame2 = np.uint8(avg_frame * 255)
			avg_frame2 = np.uint8( (avg_frame2/np.amax(avg_frame2)*(255)))
			#cv2.imwrite(os.path.join(folderName, 'post_focus_adjustment.png'), avg_frame2)

			focus_adj_dict = focus.FocusMeasure(avg_frame, True, root_dir, ini_file, results_folder)

			answ = input('Refocus? (Y/N): ')
			if (answ == 'N' or answ == 'n'):
				exit_p = True


		now = datetime.now()
		focus_adj_dict['info'] = dict()
		focus_adj_dict['info']['CheckSum'] = 0
		focus_adj_dict['info']['Year'] = int(now.strftime('%Y'))
		focus_adj_dict['info']['Month'] = int(now.strftime('%m'))
		focus_adj_dict['info']['Day'] = int(now.strftime('%d'))
		focus_adj_dict['info']['Hour'] = int(now.strftime('%H'))
		focus_adj_dict['info']['Minute'] = int(now.strftime('%M'))
		focus_adj_dict['info']['Second'] = int(now.strftime('%S'))
		focus_adj_dict['info']['configuration_file'] = cfg_file
		focus_adj_dict['info']['Pass'] = 1

	except:
		now = datetime.now()
		focus_adj_dict['info'] = dict()
		focus_adj_dict['info']['CheckSum'] = 0
		focus_adj_dict['info']['Year'] = int(now.strftime('%Y'))
		focus_adj_dict['info']['Month'] = int(now.strftime('%m'))
		focus_adj_dict['info']['Day'] = int(now.strftime('%d'))
		focus_adj_dict['info']['Hour'] = int(now.strftime('%H'))
		focus_adj_dict['info']['Minute'] = int(now.strftime('%M'))
		focus_adj_dict['info']['Second'] = int(now.strftime('%S'))
		focus_adj_dict['info']['configuration_file'] = cfg_file
		focus_adj_dict['info']['Pass'] = 0


	f = open(os.path.join(folderName, file['FocusAdjustment']['output_file_pkl']+'.pkl'), 'wb')
	pickle.dump(focus_adj_dict, f)
	f.close()

	#ADI IR lamp setup
	if file['FocusAdjustment']['lamp_control'] == True:
		ir_lamp_control.lamp_shutdown(irl, id_check)