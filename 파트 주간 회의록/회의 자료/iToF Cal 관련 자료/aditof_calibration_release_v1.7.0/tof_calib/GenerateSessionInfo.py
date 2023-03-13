#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#

import numpy as np
import yaml
import os
import shutil
from datetime import datetime


def GenerateYamlSessionFile(config_file=None, CalibfolderName=None):
	'''
		Generates a yaml session file

		Inputs:
		  config_file           - name of configuration (.cfg) file
		  CalibfolderName       - name of calbration folder
		  
		Outputs:
		  None
		
	'''

	with open(config_file) as y:
		config_file = yaml.load(y, Loader=yaml.SafeLoader)

	P0_sensor_FW_path = config_file['P0Calibration']['configuration_file']
	try:
		calibration_dir_path = config_file['calibration_directory']
	except:
		calibration_dir_path = ""
	P0_sensor_FW_basename = os.path.basename(P0_sensor_FW_path)

	sessionConfigPath = os.path.join(CalibfolderName, 'SessionConfig')

	if not os.path.exists(sessionConfigPath):
		os.mkdir(sessionConfigPath)

	P0_sensor_FW_session = os.path.join(sessionConfigPath, P0_sensor_FW_basename)
	print(P0_sensor_FW_basename)
	print(P0_sensor_FW_session)

	if os.path.abspath(P0_sensor_FW_path) != P0_sensor_FW_path:
		P0_sensor_FW_path = os.path.join(calibration_dir_path, P0_sensor_FW_path)

	shutil.copyfile(P0_sensor_FW_path, P0_sensor_FW_session)

	moduleName = os.path.basename(CalibfolderName)
	if len(moduleName) > 30:
		serialNumber = moduleName[-30:]
	else:
		serialNumber = moduleName


	if not os.path.exists(os.path.join(CalibfolderName, 'header.yaml')):
		now = datetime.now()

		ySession = dict()
		ySession['module_serial_no'] = serialNumber
		ySession['barcode'] = ''
		ySession['module_class'] = config_file['module_class']
		ySession['date'] = str(now.strftime('M:%m, D:%d, Y:%Y'))
		ySession['time'] = str(now.strftime('Hr:%H, M:%M, Sec:%S'))
		ySession['sensor_firmware'] = P0_sensor_FW_session
		ySession['compressed_ccb'] = ''
		ySession['ccb'] = ''
		ySession['flashConfigJson'] = ''
		ySession['depthConfigJson'] = ''
		ySession['compressed_ccb_flashed'] = False

		with open(os.path.join(CalibfolderName,'header.yaml'),'w') as f:
			yaml.dump(ySession, f)

	else:
		with open(os.path.join(CalibfolderName,'header.yaml')) as f:
			ySession = yaml.safe_load(f)
		
		now = datetime.now()
		ySession['date'] = str(now.strftime('M:%m, D:%d, Y:%Y'))
		ySession['time'] = str(now.strftime('Hr:%H, M:%M, Sec:%S'))
		ySession['sensor_firmware'] = P0_sensor_FW_session

		with open(os.path.join(CalibfolderName,'header.yaml'),'w') as f:
			yaml.dump(ySession, f)
