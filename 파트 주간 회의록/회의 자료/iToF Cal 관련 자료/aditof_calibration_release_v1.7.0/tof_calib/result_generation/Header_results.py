#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#
import numpy as np
import pickle
import os
import yaml

def add_results_header(CalibfolderName, calibResult, file, result_dir):
	'''
		Adds results of geometric calibration

		Inputs:
		  CalibfolderName       - Name of calibration output directory
		  calibResult           - Calibration result dict
		  file                  - Name of .yaml config file
		  result_dir            - Name of result directory
		  
		Outputs:
		  calibResult           - Updated calibration result dict
		
	'''
	print("Generating results for Header...")

	header_output = os.path.join(CalibfolderName, "header.yaml")

	if os.path.exists(header_output):
		with open(header_output) as y:
			yaml_header = yaml.load(y, Loader=yaml.SafeLoader)

		calibResult['Header_pass'] = True
		calibResult['Header_exists'] = True

		calibResult['HeaderInfo'] = dict()
		
		for key, val in yaml_header.items():
			calibResult['HeaderInfo'][key] = str(val)


	return calibResult