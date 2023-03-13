#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#

import os
import numpy as np
import yaml
import pickle


def checkCalibration(calib_dir, calib_file):
	"""
	Checks a particular calibration
	Inputs:
		  config_file           - name of configuration (.yaml) file
		  CalibfolderName       - name of calbration folder

	Outputs:
		status: calibration status
	"""
	if os.path.exists(calib_dir):
		status = 'INCOMPLETE'

		if os.path.exists(calib_file):
			f = open(calib_file, 'rb')
			calib = pickle.load(f)
			f.close()
			try:
				keys1 = calib.keys()
				if 'info' in keys1:
					if calib['info']['Pass'] == 1:
						status = 'PASS'
					else:
						status = 'FAIL'
				else:
					status = 'UNKNOWN'
			except:
				status = 'FAIL'
	else:
		status = 'NOT EXEC'

	return status



def GetCalibrationStatus(config_file=None, CalibfolderName = ""):
	'''
		Finds status of calibration
		
		Inputs:
		  config_file           - name of configuration (.yaml) file
		  CalibfolderName       - name of calbration folder
		  
		Outputs:
		  None
		
	'''
	with open(config_file) as y:
		file = yaml.load(y, Loader=yaml.SafeLoader)

	status = dict()
	status['1'] = ':'
	status['2'] = ':'
	status['3'] = ':'
	status['4'] = ':'
	status['5'] = ':'
	status['6'] = ':'
	status['7'] = ':'
	status['8'] = ':'
	status['9'] = ':'
	status['10'] = ':'

	ICQC_dir = file['Connectivity_ICQ']['output_directory']
	ICQC_dir = os.path.join(CalibfolderName, ICQC_dir)
	ICQC_file = file['Connectivity_ICQ']['output_file_pkl'] + '.pkl'
	ICQC_file = os.path.join(ICQC_dir, ICQC_file)

	GO_dir = file['GainOffsetCalibration']['output_directory']
	GO_dir = os.path.join(CalibfolderName, GO_dir)
	GO_file = file['GainOffsetCalibration']['output_file_pkl'] + '.pkl'
	GO_file = os.path.join(GO_dir, GO_file)

	AVRL_dir = file['AddressValueRegisterList']['output_directory']
	AVRL_dir = os.path.join(CalibfolderName, AVRL_dir)
	AVRL_file = file['AddressValueRegisterList']['output_file_pkl'] + '.pkl'
	AVRL_file = os.path.join(AVRL_dir, AVRL_file)

	Blemish_dir = file['Blemish']['output_directory']
	Blemish_dir = os.path.join(CalibfolderName, Blemish_dir)
	Blemish_file = file['Blemish']['output_file_pkl'] + '.pkl'
	Blemish_file = os.path.join(Blemish_dir, Blemish_file)

	FA_dir = file['FocusAdjustment']['output_directory']
	FA_dir = os.path.join(CalibfolderName, FA_dir)
	FA_file = file['FocusAdjustment']['output_file_pkl'] + '.pkl'
	FA_file = os.path.join(FA_dir, FA_file)

	FV_dir = file['FocusVerification']['output_directory']
	FV_dir = os.path.join(CalibfolderName, FV_dir)
	FV_file = file['FocusVerification']['output_file_pkl'] + '.pkl'
	FV_file = os.path.join(FV_dir, FV_file)

	Geo_dir = file['GeometricCalibration']['output_directory']
	Geo_dir = os.path.join(CalibfolderName, Geo_dir)
	Geo_file = file['GeometricCalibration']['output_file_pkl'] + '.pkl'
	Geo_file = os.path.join(Geo_dir, Geo_file)

	LSDAC_dir = file['LSDACSetting']['output_directory']
	LSDAC_dir = os.path.join(CalibfolderName, LSDAC_dir)
	LSDAC_file = file['LSDACSetting']['output_file_pkl'] + '.pkl'
	LSDAC_file = os.path.join(LSDAC_dir, LSDAC_file)

	P0_dir = file['P0Calibration']['output_directory']
	P0_dir = os.path.join(CalibfolderName, P0_dir)
	P0_file = np.zeros(len(file['P0Calibration']['calibration']['mode']),dtype=object)
	for i in range(len(file['P0Calibration']['calibration']['mode'])):
		P0_file[i] = file['P0Calibration']['output_file_uncompressed_pkl'] + str(file['P0Calibration']['calibration']['mode'][i]) + '.pkl'
		P0_file[i] = os.path.join(P0_dir, P0_file[i])

	if file['P0Calibration']['calibration']['compress_P0Table'] == True:
		P0_file = np.zeros(len(file['P0Calibration']['calibration']['mode']),dtype=object)
		for i in range(len(file['P0Calibration']['calibration']['mode'])):
			P0_file[i] = file['P0Calibration']['output_file_compressed_pkl'] + str(file['P0Calibration']['calibration']['mode'][i]) + '.pkl'
			P0_file[i] = os.path.join(P0_dir, P0_file[i])

	Verify_dir = file['Calibration_verification']['output_directory']
	Verify_dir = os.path.join(CalibfolderName, Verify_dir)
	Verify_file = file['Calibration_verification']['output_file_pkl'] + '.pkl'
	Verify_file = os.path.join(Verify_dir, Verify_file)

	status['1'] = status['1'] + checkCalibration(ICQC_dir, ICQC_file)
	status['2'] = status['2'] + checkCalibration(GO_dir, GO_file)
	status['3'] = status['3'] + checkCalibration(AVRL_dir, AVRL_file)
	status['4'] = status['4'] + checkCalibration(Blemish_dir, Blemish_file)
	status['5'] = status['5'] + checkCalibration(FA_dir, FA_file)
	status['6'] = status['6'] + checkCalibration(FV_dir, FV_file)
	status['7'] = status['7'] + checkCalibration(Geo_dir, Geo_file)
	status['8'] = status['8'] + checkCalibration(LSDAC_dir, LSDAC_file)


	status_arr = []
	for i in range(len(file['P0Calibration']['calibration']['mode'])):
		status_arr.append(checkCalibration(P0_dir, P0_file[i]))

	status_arr = np.array(status_arr)

	if not(('FAIL' in status_arr) or ('INCOMPLETE' in status_arr) or ('UNKNOWN' in status_arr) or ('NOT EXEC' in status_arr)):
		status['9'] = status['9'] + 'PASS'
	elif  'FAIL' in status_arr:
		status['9'] = status['9'] + 'FAIL'
	elif  'UNKNOWN' in status_arr:
		status['9'] = status['9'] + 'UNKNOWN'
	elif  'INCOMPLETE' in status_arr:
		status['9'] = status['9'] + 'INCOMPLETE'
	elif  'EXEC' in status_arr:
		status['9'] = status['9'] + 'EXEC'
	else:
		status['9'] = status['9'] + 'NOT EXEC'

	status['10'] = status['10'] + checkCalibration(Verify_dir, Verify_file)

	return status