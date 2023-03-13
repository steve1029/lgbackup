#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#

import numpy as np
import pickle
import os

def add_results_ICQC(CalibfolderName, calibResult, file, result_dir):
	'''
		Adds results of Connectivity and ICQ

		Inputs:
		  CalibfolderName       - Name of calibration output directory
		  calibResult           - Calibration result dict
		  file                  - Name of .yaml config file
		  result_dir            - Name of result directory
		  
		Outputs:
		  calibResult           - Updated calibration result dict
		
	'''
	print("Generating results for ICQ and Connectivity...")
	ICQC_dir = os.path.join(CalibfolderName, file['Connectivity_ICQ']['output_directory'])
	ICQC_output = os.path.join(ICQC_dir, file['Connectivity_ICQ']['output_file_pkl']+'.pkl')
	if os.path.exists(ICQC_dir):
		if os.path.exists(ICQC_output):
			f = open(ICQC_output, 'rb')
			ICQC_dict = pickle.load(f)
			f.close()
			calibResult['ICQC_exists'] = True
			if ICQC_dict['info']['Pass'] == True:
				calibResult['ICQC_pass'] = True
				calibResult['ICQC_Malka_pass'] = True
				calibResult['ICQC_Connectivity_pass'] = True
				calibResult['ICQC_Capture_pass'] = True
			else:
				calibResult['ICQC_pass'] = False

			calibResult['ICQC'] = dict()
			for key in ICQC_dict.keys():
				if key == 'info':
					pass
				else:
					calibResult['ICQC'][key] = ICQC_dict[key]

	return calibResult