#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#

import cv2
import numpy as np
import sys
from . import CalibAPI as tof
from . import P0_functions as P0
import time
import matplotlib.pyplot as plt
import pickle
import os
from datetime import datetime
import logging
import yaml
from . import efuse
from . import temp_sensor
import csv
from csv import writer
import pandas as pd

def test_frames(config_file=None, CalibfolderName = ""):
	'''
		Connects to camera and captures frames in non-laser mode
		
		Inputs:
		  config_file           - name of configuration (.cfg) file
		  CalibfolderName       - name of calbration folder
		  
		Outputs:
		  None
		
	'''

	with open(config_file) as y:
		file = yaml.load(y, Loader=yaml.SafeLoader)

	y.close()
	# create IOLib object and open camera

	now = datetime.now()

	if CalibfolderName == "":
		folderName = CalibfolderName + '_' + str(now.strftime('%Y%m%d_%H%M%S'))
	else:
		folderName = os.path.join(CalibfolderName, file['Connectivity_ICQ']['output_directory'])

	cfg_file = file['Connectivity_ICQ']['configuration_file']
	try:
		calibration_dir_path = file['calibration_directory']
	except:
		calibration_dir_path = ""

	output_file_csv_report = file['Connectivity_ICQ']['output_file_csv_report']
	output_file_pkl = file['Connectivity_ICQ']['output_file_pkl']
	limits_file = file['limits_file']
	module_class = file['module_class']

	if os.path.abspath(cfg_file) != cfg_file:
		cfg_file = os.path.join(calibration_dir_path, cfg_file)

	if os.path.abspath(limits_file) != limits_file:
		limits_file = os.path.join(calibration_dir_path, limits_file)

	try:
		with open(limits_file) as ly:
			limits = yaml.load(ly, Loader=yaml.SafeLoader)['Connectivity_ICQ']

	except:
		print(limits_file)
		raise Exception("\n\nWARNING: Incorrect/non-existing limits file\n\n")

	
	if not os.path.exists(folderName):
		os.mkdir(folderName)
	else:
		now = datetime.now()
		print('\nALERT: ' + folderName + ' already exists')
		old_folderName = folderName + str(now.strftime('_old_%Y%m%d_%H%M%S'))
		os.rename(folderName, old_folderName)
		print('ALERT: RENAMING OLD FOLDER: ' + folderName + ' as '+ old_folderName +'\n')
		os.mkdir(folderName)

	cam = tof.CalibAPI(module_class = module_class)
	ICQ_dict = dict()
	ICQ_dict['info'] = dict()
	ICQ_dict['info']['CheckSum'] = 0
	ICQ_dict['info']['Year'] = int(now.strftime('%Y'))
	ICQ_dict['info']['Month'] = int(now.strftime('%m'))
	ICQ_dict['info']['Day'] = int(now.strftime('%d'))
	ICQ_dict['info']['Hour'] = int(now.strftime('%H'))
	ICQ_dict['info']['Minute'] = int(now.strftime('%M'))
	ICQ_dict['info']['Second'] = int(now.strftime('%S'))
	ICQ_dict['info']['configuration_file'] = cfg_file
	ICQ_dict['info']['Pass'] = 0

	if file['module_class'] == 'Crosby':
		ICQ_mode = 'laser'
	else:
		ICQ_mode = 'no_laser'

	try:
		[tests_failed, ICQ_dict] = cam.ICQSensorStart(Mode=3, 
									  		fps=10, 
									  		cfg = cfg_file, 
									  		limits = limits, 
									  		ICQ_dict = ICQ_dict,
									  		ICQ_mode = ICQ_mode) # 'no_laser' or 'laser'
	except:
		raise Exception('\n ERROR: NOT ABLE TO CONNECT TO CAMERA')

	hdrTemp = dict(
					mode = 3,
					nFrames = 3,
					nWarmupFrames = 3,
					OVERRIDE_LSDACS = False,
					LSDACValue = 109,
					enableSweep = 0,
					outputFormat = 'pkl',
					fileName = folderName + '/testFrames.pkl'
					)

	try:
		val = cam.regread(0x0112)
		if val != 22833:
			raise Exception('ERROR : Incorrect chipId')
			ICQ_dict['info']['Connectivity_pass'] = 0
			ICQ_dict['info']['Pass'] = 0
		else:
			print('\nCONNECTION TO CAMERA -> OKAY')
			ICQ_dict['info']['Connectivity_pass'] = 1

	except:
		ICQ_dict['info']['Connectivity_pass'] = 0
		raise Exception('\n ERROR: NOT ABLE TO CONNECT TO CAMERA')

	
	temp_sensor_result = temp_sensor.read_spi_temp_sensor(cam,num_averages=5)
	Board_temp = ['Board temp', temp_sensor_result, "No Limit"]
	efuse_data = efuse.read_spi_efuse(cam, 0, 64, False)
	EFUSE_FORMAT_REV = ['EFUSE_FORMAT_REV', efuse_data[0] & 0xFF, "No Limit"]
	EFUSE_FIELD_EN = ['EFUSE_FIELD_EN', efuse_data[0] >> 8, "No Limit"]
	EFUSE_LOTID_STR = (chr(efuse_data[1] & 0xFF) + chr(efuse_data[1] >> 8) + chr(efuse_data[2] & 0xFF) + chr(efuse_data[2] >> 8) 
		+ chr(efuse_data[3] & 0xFF) + chr(efuse_data[3] >> 8) + chr(efuse_data[4] & 0xFF) + chr(efuse_data[4] >> 8) + chr(efuse_data[5] & 0xFF)) 
	EFUSE_LOTID = ['EFUSE_LOTID', EFUSE_LOTID_STR, "No Limit"]
	EFUSE_WAFER = ['EFUSE_WAFER', efuse_data[5] >> 8, "No Limit"]
	EFUSE_DIE_X = ['EFUSE_DIE_X', efuse_data[6] & 0xFF, "No Limit"]
	EFUSE_DIE_Y = ['EFUSE_DIE_Y', efuse_data[6] >> 8, "No Limit"]
	EFUSE_TEST_PROG_REV_CP = ['EFUSE_TEST_PROG_REV_CP', efuse_data[7], "No Limit"]
	EFUSE_TEST_PROG_REV_FT = ['EFUSE_TEST_PROG_REV_FT', efuse_data[8], "No Limit"]
	EFUSE_TEMP_CAL_VER = ['EFUSE_TEMP_CAL_VER', efuse_data[11] & 0xFF, "No Limit"]
	EFUSE_TEMP_TJ = ['EFUSE_TEMP_TJ', efuse_data[11] >> 8, "No Limit"] 
	EFUSE_TEMP_ADC = ['EFUSE_TEMP_ADC', efuse_data[12] & 0xFFF, "No Limit"] 
	EFUSE_PCM4_CORERVT = ['EFUSE_PCM4_CORERVT', efuse_data[13], "No Limit"] 
	EFUSE_PCM14_IO = ['EFUSE_PCM14_IO', efuse_data[14], "No Limit"] 
	EFUSE_PCM10_CORERVN = ['EFUSE_PCM10_CORERVN', efuse_data[15], "No Limit"] 
	EFUSE_PCM11_CORERVP = ['EFUSE_PCM11_CORERVP', efuse_data[16], "No Limit"] 
	EFUSE_PCM2_COREHVT = ['EFUSE_PCM2_COREHVT', efuse_data[17], "No Limit"] 
	EFUSE_PCM15_ION = ['EFUSE_PCM15_ION', efuse_data[18], "No Limit"] 
	EFUSE_PCM16_IOP = ['EFUSE_PCM16_IOP', efuse_data[19], "No Limit"] 
	EFUSE_FOM1 = ['EFUSE_FOM1', efuse_data[36], "No Limit"] 
	EFUSE_FOM2 = ['EFUSE_FOM2', efuse_data[37], "No Limit"] 
	EFUSE_ADC_CAL_VER = ['EFUSE_ADC_CAL_VER', efuse_data[48] & 0xFF, "No Limit"] 
	EFUSE_ADC_DIFF = ['EFUSE_ADC_DIFF', efuse_data[48] >> 8, "No Limit"] 
	EFUSE_ADC_UPDN = ['EFUSE_ADC_UPDN', efuse_data[49], "No Limit"] 
	EFUSE_IRAMP = ['EFUSE_IRAMP', efuse_data[50], "No Limit"] 
	EFUSE_CHECKSUM = ['EFUSE_CHECKSUM', efuse_data[63], "No Limit"] 

	ICQ_dict['Board temp'] = Board_temp[1:]
	ICQ_dict['EFUSE_FORMAT_REV'] = EFUSE_FORMAT_REV[1:]
	ICQ_dict['EFUSE_FIELD_EN'] = EFUSE_FIELD_EN[1:]
	ICQ_dict['EFUSE_LOTID'] = EFUSE_LOTID[1:]
	ICQ_dict['EFUSE_WAFER'] = EFUSE_WAFER[1:]
	ICQ_dict['EFUSE_DIE_X'] = EFUSE_DIE_X[1:]
	ICQ_dict['EFUSE_DIE_Y'] = EFUSE_DIE_Y[1:]
	ICQ_dict['EFUSE_TEST_PROG_REV_CP'] = EFUSE_TEST_PROG_REV_CP[1:]
	ICQ_dict['EFUSE_TEST_PROG_REV_FT'] = EFUSE_TEST_PROG_REV_FT[1:]
	ICQ_dict['EFUSE_TEMP_CAL_VER'] = EFUSE_TEMP_CAL_VER[1:]
	ICQ_dict['EFUSE_TEMP_TJ'] = EFUSE_TEMP_TJ[1:]
	ICQ_dict['EFUSE_TEMP_ADC'] = EFUSE_TEMP_ADC[1:]
	ICQ_dict['EFUSE_PCM4_CORERVT'] = EFUSE_PCM4_CORERVT[1:]
	ICQ_dict['EFUSE_PCM14_IO'] = EFUSE_PCM14_IO[1:]
	ICQ_dict['EFUSE_PCM10_CORERVN'] = EFUSE_PCM10_CORERVN[1:]
	ICQ_dict['EFUSE_PCM11_CORERVP'] = EFUSE_PCM11_CORERVP[1:]
	ICQ_dict['EFUSE_PCM2_COREHVT'] = EFUSE_PCM2_COREHVT[1:]
	ICQ_dict['EFUSE_PCM15_ION'] = EFUSE_PCM15_ION[1:]
	ICQ_dict['EFUSE_PCM16_IOP'] = EFUSE_PCM16_IOP[1:]
	ICQ_dict['EFUSE_FOM1'] = EFUSE_FOM1[1:]
	ICQ_dict['EFUSE_FOM2'] = EFUSE_FOM2[1:]
	ICQ_dict['EFUSE_ADC_CAL_VER'] = EFUSE_ADC_CAL_VER[1:]
	ICQ_dict['EFUSE_ADC_DIFF'] = EFUSE_ADC_DIFF[1:]
	ICQ_dict['EFUSE_ADC_UPDN'] = EFUSE_ADC_UPDN[1:]
	ICQ_dict['EFUSE_IRAMP'] = EFUSE_IRAMP[1:]
	ICQ_dict['EFUSE_CHECKSUM'] = EFUSE_CHECKSUM[1:]

	if ICQ_dict['info']['Connectivity_pass'] == 1:
		try:
			Img, Hdr = cam.CaptureFramesP0(hdrTemp)
			print('CAPTURING passive IR FRAMES -> OKAY\n')
			ICQ_dict['info']['Capture_pass'] = 1
			if tests_failed == 0:
				ICQ_dict['info']['Pass'] = 1
			else:
				ICQ_dict['info']['Pass'] = 0
		except:
			ICQ_dict['info']['Capture_pass'] = 0
			ICQ_dict['info']['Pass'] = 0
			raise Exception('\n ERROR: CANNOT CAPURE FRAMES')
	else:
		ICQ_dict['info']['Capture_pass'] = 0
		ICQ_dict['info']['Pass'] = 0

	if ICQ_dict['info']['Pass'] == 1:
		device_passed = "PASSED"
		print("\n=================")
		print('ICQ TESTS PASSED')
		print("=================\n")
	else:
		device_passed = "FAILED"
		print("\n=================")
		print('ICQ TESTS FAILED')
		print("=================\n")

	cam.DisconnectSensor()


	pkl_file = os.path.join(folderName, file['Connectivity_ICQ']['output_file_pkl'])
	f = open(pkl_file + ".pkl", 'wb')
	pickle.dump(ICQ_dict, f)
	f.close()
		
	# Save results to CSV file 
	ICQ_status = dict()
	ICQ_status['Pass/Fail'] = [ICQ_dict['info']['Pass'], device_passed]
	for key, value in ICQ_dict.items():
		if isinstance(ICQ_dict[key], (list,tuple,dict, str)):
			if len(ICQ_dict[key]) <= 4:
				ICQ_status[key] = value
	#ICQ_status.update(ICQ_dict)
	df = pd.DataFrame(dict([ (k,pd.Series(v)) for k,v in ICQ_status.items() ]))
	df = df.T
	df.columns = ['Value', 'Result', 'Min Limit', 'Max Limit']
	df.to_csv(os.path.join(folderName, output_file_csv_report + '.csv'))

	del cam

if __name__ == "__main__":
    test_frames("./config/CalibrationConfig.yaml" , CalibfolderName = "")