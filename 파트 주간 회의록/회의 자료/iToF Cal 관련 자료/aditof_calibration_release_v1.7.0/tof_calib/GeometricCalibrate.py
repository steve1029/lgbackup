#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#

import numpy as np
import sys
from . import GenerateGeometricIntrinsicsfromcsv as GeoCSV
from . import Geometric_functions as GeoFN
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

# import ftd2xx as ftd
# import lamp_control.IR_Lamp_Control as ir_lamp_control

def Geometric_calibrate(config_file=None, CalibfolderName = ""):
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


	try:
		calibration_dir_path = file['calibration_directory']
	except:
		calibration_dir_path = ""

	calibration_mode = file['GeometricCalibration']['calibration_mode']
	limits_file = file['limits_file']

	if os.path.abspath(limits_file) != limits_file:
		limits_file = os.path.join(calibration_dir_path, limits_file)
	try:
		with open(limits_file) as ly:
			limits = yaml.load(ly, Loader=yaml.SafeLoader)['GeometricCalibration']
	except:
		raise Exception("\n\nWARNING: Incorrect/non-existing limits file\n\n")

	intrinsic_coords_path = os.path.join(calibration_dir_path, file['GeometricCalibration']['coordinates_json_file'])

	if CalibfolderName == "":
		now = datetime.now()
		folderName = file['GeometricCalibration']['output_directory'] + '_' + str(now.strftime('%Y%m%d_%H%M%S'))
	else:
		folderName = os.path.join(CalibfolderName, file['GeometricCalibration']['output_directory'])
	
	if not os.path.exists(folderName):
		os.mkdir(folderName)
	else:
		now = datetime.now()
		print('\nALERT: ' + folderName + ' already exists')
		old_folderName = folderName + str(now.strftime('_old_%Y%m%d_%H%M%S'))
		os.rename(folderName, old_folderName)
		print('ALERT: RENAMING OLD FOLDER: ' + folderName + ' as '+ old_folderName +'\n')
		os.mkdir(folderName)

	if calibration_mode == 'csv':
		GeoCSV.GenerateGeometricIntrinsicsfromcsv(config_file=config_file, CalibfolderName=CalibfolderName)

	elif calibration_mode == 'image':
		input_file = file['GeometricCalibration']['input_file_png']
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

		img = plt.imread(input_file)
		geo_dict = GeoFN.GeometricCal(img, intrinsic_coords_path, folderName, geo_dict)

	elif calibration_mode == 'capture':
		#ADI IR lamp setup
		# if file['GeometricCalibration']['lamp_control'] == True:
		# 	irl, id_check = ir_lamp_control.lamp_setup()

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

		cfg = file['GeometricCalibration']['configuration_file']
		output_file_csv_report = file['GeometricCalibration']['output_file_csv_report']

		if os.path.abspath(cfg) != cfg:
			cfg = os.path.join(calibration_dir_path, cfg)

		cam = tof.CalibAPI(module_class = module_class)
		cam.ConnectSensor(Mode=3, fps=10, cfg=cfg)

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
		cv2.imwrite(os.path.join(folderName, 'checkerboard.png'), avg_frame2)

		# rgh = plt.imread(os.path.join(folderName, 'checkerboard.png'))
		# print(np.max(rgh),np.min(rgh))
		# plt.figure()
		# plt.imshow(rgh*255)
		# plt.show()

		geo_dict = GeoFN.GeometricCal(avg_frame, intrinsic_coords_path, folderName, geo_dict)

		#ADI IR lamp setup
		# if file['GeometricCalibration']['lamp_control'] == True:
		# 	ir_lamp_control.lamp_shutdown(irl, id_check)
		
	else:
		print("Invalid options. Valid options for calibration_mode are:")
		print('"capture"')
		print('"image"')
		print('"csv"')

	
	if calibration_mode == 'image' or 'capture':
		geo_dict['Geometric_pass'] = True #Initialize to True

		if geo_dict['Rerr']  < limits['reprojection_error'][1]:
			Rerr_status = "PASSED"
		else:
			Rerr_status = "FAILED"
			print('\nReprojection Error FAIL(' +str( geo_dict['Rerr'])+'), please redo calibration.\n')
			geo_dict['Geometric_pass'] = False
		geo_dict['Rerr'] = [geo_dict['Rerr'], Rerr_status, limits['reprojection_error'][0], limits['reprojection_error'][1]]

		if limits['Fc1'][0] < geo_dict['Fc1']  < limits['Fc1'][1]:
			Fc1_status = "PASSED"
		else:
			Fc1_status = "FAILED"
			print('\nFc1 Error FAIL\n')
			geo_dict['Geometric_pass'] = False
		geo_dict['Fc1'] = [geo_dict['Fc1'], Fc1_status, limits['Fc1'][0], limits['Fc1'][1]]

		if limits['Fc2'][0] < geo_dict['Fc2']  < limits['Fc2'][1]:
			Fc2_status = "PASSED"
		else:
			Fc2_status = "FAILED"
			print('\nFc2 Error FAIL\n')
			geo_dict['Geometric_pass'] = False
		geo_dict['Fc2'] = [geo_dict['Fc2'], Fc2_status, limits['Fc2'][0], limits['Fc2'][1]]

		if limits['cc1'][0] < geo_dict['cc1']  < limits['cc1'][1]:
			cc1_status = "PASSED"
		else:
			cc1_status = "FAILED"
			print('\ncc1 Error FAIL\n')
			geo_dict['Geometric_pass'] = False
		geo_dict['cc1'] = [geo_dict['cc1'], cc1_status, limits['cc1'][0], limits['cc1'][1]]

		if limits['cc2'][0] < geo_dict['cc2']  < limits['cc2'][1]:
			cc2_status = "PASSED"
		else:
			cc2_status = "FAILED"
			print('\ncc2 Error FAIL\n')
			geo_dict['Geometric_pass'] = False
		geo_dict['cc2'] = [geo_dict['cc2'], cc2_status, limits['cc2'][0], limits['cc2'][1]]

		geo_dict['info']['Pass'] = geo_dict['Geometric_pass']

		if geo_dict['info']['Pass'] == True:
			device_passed = "PASSED"
			print("\n=====================")
			print('GEOMETRIC CAL PASSED')
			print("=====================\n")
		else:
			device_passed = "FAILED"
			print("\n=====================")
			print('GEOMETRIC CAL FAILED')
			print("=====================\n")

		f = open(os.path.join(folderName, file['GeometricCalibration']['output_file_pkl']+'.pkl'), 'wb')
		pickle.dump(geo_dict, f)
		f.close()

		# Save results to CSV file 
		Geo_status = dict()
		Geo_status['Pass/Fail'] = [geo_dict['Geometric_pass'], device_passed]
		for key, value in geo_dict.items():
			if isinstance(geo_dict[key], (list,tuple,dict, str)):
				if len(geo_dict[key]) <= 4:
					Geo_status[key] = value
			elif not isinstance(geo_dict[key], bool):
				Geo_status[key] = value
		df = pd.DataFrame(dict([ (k,pd.Series(v)) for k,v in Geo_status.items() ]))
		df = df.T
		df.columns = ['Value', 'Result', 'Min Limit', 'Max Limit']
		df.to_csv(os.path.join(folderName, output_file_csv_report + '.csv'))
