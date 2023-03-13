#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#
import numpy as np
import pickle
import os
import yaml

def add_results_AVRL(CalibfolderName, calibResult, file, result_dir):
	'''
		Adds results of Address value register list

		Inputs:
		  CalibfolderName       - Name of calibration output directory
		  calibResult           - Calibration result dict
		  file                  - Name of .yaml config file
		  result_dir            - Name of result directory
		  
		Outputs:
		  calibResult           - Updated calibration result dict
		
	'''
	print("Generating results for VLOW and DAC trim...")
	AVRL_dir = os.path.join(CalibfolderName, file['AddressValueRegisterList']['output_directory'])
	AVRL_output = os.path.join(AVRL_dir, 'AVRL_dict.pkl')
	if os.path.exists(AVRL_dir):
		if os.path.exists(AVRL_output):
			f = open(AVRL_output, 'rb')
			AVRL_dict = pickle.load(f)
			f.close()
			calibResult['AVRL'] = dict()
			calibResult['AVRL_exists'] = True
			calibResult['AVRL_pass'] = AVRL_dict['AVRL_pass']
			calibResult['AVRL']['vlow_lin'] = AVRL_dict['vlow_lin']
			calibResult['AVRL']['vlow_sw'] = AVRL_dict['vlow_sw']
			calibResult['AVRL']['DAC_SatRef'] = AVRL_dict['DAC_SatRef']
			calibResult['AVRL']['DAC_TX1'] = AVRL_dict['DAC_TX1']
			calibResult['AVRL']['DAC_Bias1'] = AVRL_dict['DAC_Bias1']
			calibResult['AVRL']['DAC_Bias2'] = AVRL_dict['DAC_Bias2']
			calibResult['AVRL']['DAC_Vrest'] = AVRL_dict['DAC_Vrest']
			calibResult['AVRL']['vlow_vadj'] = AVRL_dict['vlow_vadj']
			calibResult['AVRL']['DAC_SatRef_val'] = AVRL_dict['DAC_SatRef_val']
			calibResult['AVRL']['DAC_TX1_val'] = AVRL_dict['DAC_TX1_val']
			calibResult['AVRL']['DAC_Bias1_val'] = AVRL_dict['DAC_Bias1_val']
			calibResult['AVRL']['DAC_Bias2_val'] = AVRL_dict['DAC_Bias2_val']
			calibResult['AVRL']['DAC_Vrest_val'] = AVRL_dict['DAC_Vrest_val']

	return calibResult