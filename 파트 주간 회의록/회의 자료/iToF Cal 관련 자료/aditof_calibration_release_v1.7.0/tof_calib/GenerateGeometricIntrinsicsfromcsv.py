#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#

import numpy as np
import csv
import os
import pickle
from datetime import datetime
import yaml

def GenerateGeometricIntrinsicsfromcsv(config_file, CalibfolderName):
	'''
		Generates Geometric intrinsics pkl files from csv input

		Inputs:
		  config_file           - name of configuration (.yaml) file
		  CalibfolderName       - name of calbration folder
		  
		Outputs:
		  None
		
	'''
	with open(config_file) as y:
		yaml_file = yaml.load(y, Loader=yaml.SafeLoader)

	geo_dict = dict()
	geo_dict_keys = ['Fc1', 'Fc2', 'cc1', 'cc2', 'Cx', 'Cy', 'Kc1', 'Kc2', 'Kc3', 'Kc4', 'Kc5', 'Kc6', 'Tx', 'Ty']
	file_dir_path = os.path.join(CalibfolderName, yaml_file['GeometricCalibration']['output_directory'])


	with open(yaml_file['GeometricCalibration']['input_file_csv']) as csv_file:
		csv_reader = csv.reader(csv_file, delimiter=',')
		line_count = 0
		for row in csv_reader:
			if line_count == 0:
				line_count += 1
			else:
				geo_dict[geo_dict_keys[line_count-1]] = np.float32(row[1])
				line_count += 1


	now = datetime.now()
	geo_dict['Rerr'] = 0
	geo_dict['info'] = dict()
	geo_dict['info']['CheckSum'] = 0
	geo_dict['info']['Year'] = int(now.strftime('%Y'))
	geo_dict['info']['Month'] = int(now.strftime('%m'))
	geo_dict['info']['Day'] = int(now.strftime('%d'))
	geo_dict['info']['Hour'] = int(now.strftime('%H'))
	geo_dict['info']['Minute'] = int(now.strftime('%M'))
	geo_dict['info']['Second'] = int(now.strftime('%S'))
	geo_dict['info']['configuration_file'] = yaml_file['GeometricCalibration']['configuration_file']
	geo_dict['info']['Pass'] = 1

	f = open(os.path.join(file_dir_path, yaml_file['GeometricCalibration']['output_file_pkl']+'.pkl'), 'wb')
	pickle.dump(geo_dict, f)
	f.close()
