#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#

import numpy as np
import pickle
import os


def add_results_LSDAC(CalibfolderName, calibResult, file, result_dir):
	'''
		Adds results of LSDAC setting

		Inputs:
		  CalibfolderName       - Name of calibration output directory
		  calibResult           - Calibration result dict
		  file                  - Name of .yaml config file
		  result_dir            - Name of result directory
		  
		Outputs:
		  calibResult           - Updated calibration result dict
		
	'''
	print("Generating results for LSDAC...")
	LSDAC_dir = os.path.join(CalibfolderName, file['LSDACSetting']['output_directory'])
	LSDAC_output = os.path.join(LSDAC_dir, 'LSDAC_values.pkl')
	if os.path.exists(LSDAC_dir):
		if os.path.exists(LSDAC_output):
			f = open(LSDAC_output, 'rb')
			LSDAC_values = pickle.load(f)
			f.close()
			calibResult['LSDAC_pass'] = True
			calibResult['LSDAC_exists'] = True
			calibResult['LSDACSetting'] = LSDAC_values

	return calibResult