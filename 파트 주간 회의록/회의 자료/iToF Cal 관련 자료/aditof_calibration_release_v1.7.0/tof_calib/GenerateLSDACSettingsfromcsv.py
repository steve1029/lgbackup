#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#

import numpy as np
try:
	from . import GenerateLSDACBlock as LSDAC
except:
	import GenerateLSDACBlock as LSDAC

import csv
import os
import pickle
from datetime import datetime
import yaml

def ShiftEndian(value):
	'''
		Converts big endian to little endians and vice verse

		Inputs:
		  value                 - Hexadecimal input
		  
		Outputs:
		  Converted value       - converted hexadecimal output
		
	'''
	for i in range(len(value)):
		tmp = value[i]
		tmp1 = tmp & 0xFF
		tmp2 = tmp >> 8
		value[i] = int(tmp1 * (2**8) + tmp2)
	return value


def GenerateLSDACSettingsfromcsv(config_file, CalibfolderName):
	'''
		Generates a LSDAC pkl file from csv input

		Inputs:
		  config_file           - Name of configuration .cfg file
		  CalibfolderName       - Name of calibration folder
		  
		Outputs:
		  None
		
	'''
	with open(config_file) as y:
		yaml_file = yaml.load(y, Loader=yaml.SafeLoader)

	LSDAC_dict = dict()
	file_dir_path = os.path.join(CalibfolderName, yaml_file['LSDACSetting']['output_directory'])
	
	if not os.path.exists(file_dir_path):
		os.mkdir(file_dir_path)
	else:
		now = datetime.now()
		print('\nALERT: ' + file_dir_path + ' already exists')
		old_folderName = file_dir_path + str(now.strftime('_old_%Y%m%d_%H%M%S'))
		os.rename(file_dir_path, old_folderName)
		print('ALERT: RENAMING OLD FOLDER: ' + file_dir_path + ' as '+ old_folderName +'\n')
		os.mkdir(file_dir_path)

	LSDAC_setting = np.zeros([1,3],dtype=np.int16)


	with open(os.path.join(CalibfolderName, yaml_file['LSDACSetting']['input_file_csv'])) as csv_file:
		csv_reader = csv.reader(csv_file, delimiter=',')
		line_count = 0
		for row in csv_reader:
			if line_count == 0:
				line_count += 1
			else:
				LSDAC_setting = np.append(LSDAC_setting, np.zeros([1,3],dtype=np.int16), axis = 0)
				LSDAC_setting[line_count][0] = np.int16(row[1])
				LSDAC_setting[line_count][1] = np.int16(row[2])
				LSDAC_setting[line_count][2] = np.int16(row[3])
				line_count += 1

	LSDAC_setting = LSDAC_setting[1:,:]

	f = open(os.path.join(file_dir_path, 'LSDAC_values.pkl'), 'wb')
	pickle.dump(LSDAC_setting, f)
	f.close()


	address, value_tmp = LSDAC.GenerateLSDACBlock(LSDAC_setting)

	#### IMPORTANT : Using Little Endian for Value
	value = value_tmp[::-1][:-2]
	value = ShiftEndian(value)
	print('Little Endian for value')
	checkSumLSB = value_tmp[0] >> 16 # Invert LSB and MSB
	checkSumMSB = value_tmp[0] & 0xFFFF
	LSDAC_dict['value'] = value
	LSDAC_dict['checkSumLSB'] = checkSumLSB
	LSDAC_dict['checkSumMSB'] = checkSumMSB
	now = datetime.now()
	LSDAC_dict['info'] = dict()
	LSDAC_dict['info']['CheckSum'] = 0
	LSDAC_dict['info']['Year'] = int(now.strftime('%Y'))
	LSDAC_dict['info']['Month'] = int(now.strftime('%m'))
	LSDAC_dict['info']['Day'] = int(now.strftime('%d'))
	LSDAC_dict['info']['Hour'] = int(now.strftime('%H'))
	LSDAC_dict['info']['Minute'] = int(now.strftime('%M'))
	LSDAC_dict['info']['Second'] = int(now.strftime('%S'))
	LSDAC_dict['info']['configuration_file'] = yaml_file['LSDACSetting']['configuration_file']
	LSDAC_dict['info']['Pass'] = 1


	if LSDAC_dict['info']['Pass'] == 1:
		print("\n=================")
		print('LSDAC CAL PASSED')
		print("=================\n")
	else:
		print("\n=================")
		print('LSDAC CAL FAILED')
		print("=================\n")

	f = open(os.path.join(file_dir_path, yaml_file['LSDACSetting']['output_file_pkl'] + '.pkl'), 'wb')
	pickle.dump(LSDAC_dict, f)
	f.close()


if __name__ == "__main__":
	now = datetime.now()
	CalibfolderName = 'LSDACSettings_' + str(now.strftime('%Y%m%d_%H%M%S'))
	os.mkdir(CalibfolderName)
	GenerateLSDACSettingsfromcsv(CalibfolderName)