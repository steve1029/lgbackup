#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#

import numpy as np
import numpy.matlib
import csv
import os
import time
import pickle
import yaml
import logging
import sys
from . import CalibAPI as tof
from datetime import datetime
import pandas as pd

def VlowDACTrim(config_file=None, CalibfolderName = ""):
	'''
		Performs VLOW trim and Pixel DAC trim (setting values)
		
		Inputs:
		  config_file           - name of configuration (.yaml) file
		  CalibfolderName       - name of calbration folder
		  
		Outputs:
		  None
		
		'''

	with open(config_file) as y:
		yaml_file = yaml.load(y, Loader=yaml.SafeLoader)

	config = yaml_file['AddressValueRegisterList']['configuration_file']
	try:
		calibration_dir_path = yaml_file['calibration_directory']
	except:
		calibration_dir_path = ""	
	y.close()

	AVRL_dict = dict()

	if os.path.abspath(config) != config:
		config = os.path.join(calibration_dir_path, config)

	folderName = os.path.join(CalibfolderName, yaml_file['AddressValueRegisterList']['output_directory'])
	output_file_csv_report = yaml_file['AddressValueRegisterList']['output_file_csv_report']


	if not os.path.exists(folderName):
		os.mkdir(folderName)
	else:
		now = datetime.now()
		print('\nALERT: ' + folderName + ' already exists')
		old_folderName = folderName + str(now.strftime('_old_%Y%m%d_%H%M%S'))
		os.rename(folderName, old_folderName)
		print('ALERT: RENAMING OLD FOLDER: ' + folderName + ' as '+ old_folderName +'\n')
		os.mkdir(folderName)

	logging.basicConfig(filename=os.path.join(folderName, 'AddressValueRegisterList.log'), 
							filemode='w', format='%(asctime)s - %(levelname)s - %(message)s',
							level=logging.INFO)

	now = datetime.now()
	AddressValueRegisterList_dict = dict()
	AddressValueRegisterList_dict['info'] = dict()
	AddressValueRegisterList_dict['info']['CheckSum'] = 0
	AddressValueRegisterList_dict['info']['Year'] = int(now.strftime('%Y'))
	AddressValueRegisterList_dict['info']['Month'] = int(now.strftime('%m'))
	AddressValueRegisterList_dict['info']['Day'] = int(now.strftime('%d'))
	AddressValueRegisterList_dict['info']['Hour'] = int(now.strftime('%H'))
	AddressValueRegisterList_dict['info']['Minute'] = int(now.strftime('%M'))
	AddressValueRegisterList_dict['info']['Second'] = int(now.strftime('%S'))
	AddressValueRegisterList_dict['info']['configuration_file'] = config
	AddressValueRegisterList_dict['info']['Pass'] = 0

	## VLOW TRIM (LINEAR AND SWITCHER)

	logging.info('CALIBRATING VLOW TRIM (LINEAR AND SWITCHER) \n\n')

	vlowThresh = 0.05 # tolerance of 50mV
	targetVoltage = 1.00
	estimatedSlopeLinear = 0.009 # 9.00mV/LSB
	estimatedSlopeSwitcher = 0.0079 # 7.9mV/LSB

	module_class = yaml_file['module_class']

	API = tof.CalibAPI(module_class = module_class)
	API.ConnectSensor(Mode=3, cfg = config)

	API.testBoardADC()


	orig_vlow_val = API.regread(0x0172)
	orig_vlow_lin_vadj = (orig_vlow_val & 0x5F00) >> 8
	orig_vlow_sw_vadj = orig_vlow_val & 0x005F
	logging.info('ORIGINAL VLOW VALUE: ' + str(orig_vlow_val))
	logging.info('ORIGINAL VLOW LINEAR ADJ: ' + str(orig_vlow_lin_vadj))
	logging.info('ORIGINAL VLOW SWITCHER ADJ: ' + str(orig_vlow_sw_vadj))

	# Vlow Linear calibration
	logging.info('\nCALIBRATING VLOW LINEAR\n')

	orig_Vlow_ctrl = API.regread(0x0170)
	orig_Vlow_en = API.regread(0x0168)

	s = API.regwrite(0x0170, 0x0000)
	s = API.regwrite(0x0168, 0x0006)
	time.sleep(0.1)

	measured_vlow_lin = API.CamMeasureAnaTestVoltage('AnaTestMux_12')
	logging.info('measured_vlow_lin (AnaTestMux_12): ' + str(measured_vlow_lin))

	vlow_lin_vadj = orig_vlow_lin_vadj + np.ceil((targetVoltage - measured_vlow_lin)/estimatedSlopeLinear)
	s = API.regrmw(0x0172, int(vlow_lin_vadj) << 8, 0x5F00)
	logging.info('Writing vlow_lin_vadj value ' + str(vlow_lin_vadj) + ' to register 0x0172')

	measured_vlow_lin = API.CamMeasureAnaTestVoltage('AnaTestMux_12')
	logging.info('measured_vlow_lin (AnaTestMux_12): ' + str(measured_vlow_lin))
	retries = 10
	while((measured_vlow_lin > 1 + vlowThresh) or (measured_vlow_lin < 1 - vlowThresh)) and retries < 1:
		print('Retry:', retries)
		vlow_lin_vadj = orig_vlow_lin_vadj + np.ceil((targetVoltage - measured_vlow_lin)/estimatedSlopeLinear)
		measured_vlow_lin = API.CamMeasureAnaTestVoltage('AnaTestMux_12')
		logging.info('measured_vlow_lin (AnaTestMux_12): ' + str(measured_vlow_lin))

	# Vlow Switcher calibration
	logging.info('\nCALIBRATING VLOW SWITCHER\n')

	s = API.regwrite(0x0168, 0x8000)
	time.sleep(0.1)

	measured_vlow_sw = API.CamMeasureAnaTestVoltage('AnaTestMux_12')
	logging.info('measured_vlow_sw (AnaTestMux_12): '+ str(measured_vlow_sw))

	vlow_sw_vadj = orig_vlow_sw_vadj + np.round((targetVoltage - measured_vlow_sw)/estimatedSlopeSwitcher)
	s = API.regrmw(0x0172, int(vlow_sw_vadj), 0x005F)
	logging.info('vlow_sw_vadj: '+ str(int(vlow_sw_vadj)))
	logging.info('writing vlow_sw_vadj to register 0x0172')

	measured_vlow_sw = API.CamMeasureAnaTestVoltage('AnaTestMux_12')
	logging.info('measured_vlow_sw (AnaTestMux_12): ' + str(measured_vlow_sw))
	retries = 10
	while((measured_vlow_sw > 1 + vlowThresh) or (measured_vlow_sw < 1 - vlowThresh)) and retries < 1:
		print('Retry:', retries)
		vlow_sw_vadj = orig_vlow_sw_vadj + np.round((targetVoltage - measured_vlow_sw)/estimatedSlopeSwitcher)
		measured_vlow_sw = API.CamMeasureAnaTestVoltage('AnaTestMux_12')
		logging.info('measured_vlow_sw (AnaTestMux_12): ' + str(measured_vlow_sw))

	Vlow_Vadj = (int(vlow_lin_vadj) << 8) + int(vlow_sw_vadj)
	print('Vlow_Vadj val:', hex(Vlow_Vadj))
	logging.info('Vlow_Vadj val:'+ str(hex(Vlow_Vadj)))
	AVRL_dict['vlow_vadj'] = Vlow_Vadj

	# Restore original settings
	s = API.regwrite(0x0168, orig_Vlow_en)
	s = API.regwrite(0x0170, orig_Vlow_ctrl)
	logging.info('Restoring original settings')

	limits_file = yaml_file['limits_file']

	if os.path.abspath(limits_file) != limits_file:
		limits_file = os.path.join(calibration_dir_path, limits_file)
	try:
		with open(limits_file) as ly:
			limits = yaml.load(ly, Loader=yaml.SafeLoader)['AddressValueRegisterList']
	except:
		raise Exception("\n\nWARNING: Incorrect/non-existing limits file\n\n")

	AVRL_dict['AVRL_pass'] = True #Initialize to True
	if limits['Vlow_lin_V'][0] < measured_vlow_lin < limits['Vlow_lin_V'][1]:
		vlow_lin_status = "PASSED"
	else:
		vlow_lin_status = "FAILED"
		print('\nVLOW Linear Regulator Trim FAIL\n')
		AVRL_dict['AVRL_pass'] = False
	AVRL_dict['vlow_lin'] = [measured_vlow_lin, vlow_lin_status, limits['Vlow_lin_V'][0], limits['Vlow_lin_V'][1]]

	if limits['Vlow_sw_V'][0] < measured_vlow_sw < limits['Vlow_sw_V'][1]:
		vlow_sw_status = "PASSED"
	else:
		vlow_sw_status = "FAILED"
		print('\nVLOW Switching Regulator Trim FAIL\n')
		AVRL_dict['AVRL_pass'] = False
	AVRL_dict['vlow_sw'] = [measured_vlow_sw, vlow_sw_status, limits['Vlow_sw_V'][0], limits['Vlow_sw_V'][1]]


	## PIXEL DAC TRIM
	logging.info('\n\nPIXEL DAC TRIM \n\n')

	target_DAC_SatRef = 0.750
	target_DAC_TX1 = 1.1
	target_DAC_Bias1 = 0.250
	target_DAC_Bias2 = 1.1
	target_DAC_Vrest = 0.500
	logging.info('target_DAC_SatRef = ' + str(target_DAC_SatRef))
	logging.info('target_DAC_Bias1 = ' + str(target_DAC_Bias1))
	logging.info('target_DAC_Bias1 = ' + str(target_DAC_Bias1))
	logging.info('target_DAC_Bias2 = ' + str(target_DAC_Bias2))
	logging.info('target_DAC_Vrest = ' + str(target_DAC_Vrest))

	# DAC_SatRef
	DAC_SatRef_x = np.array([0x0040, 0x0048, 0x0050, 0x0058])
	DAC_SatRef_y = np.zeros(len(DAC_SatRef_x))
	logging.info('DAC_SatRef')

	for i in range(len(DAC_SatRef_x)):
		s = API.regwrite(0x0132, DAC_SatRef_x[i])
		s = API.regwrite(0x0126, 0x0020)
		logging.info('Writing value ' + str(DAC_SatRef_x[i]) + ' to register 0x0132')
		logging.info('Writing value 0x0020 to register 0x0126')
		DAC_SatRef_y[i] = API.CamMeasureAnaTestVoltage('DAC_SATREF')
		logging.info('DAC_SatRef_y (DAC_SATREF): ' + str(DAC_SatRef_y[i]))

	p_SatRef = np.polyfit(DAC_SatRef_y, DAC_SatRef_x, 1)

	DAC_SatRef_val = int(np.round(np.polyval(p_SatRef, target_DAC_SatRef)))
	logging.info('\n Calculated DAC_SatRef_val: ' + str(DAC_SatRef_val))

	# Verify DAC_SatRef
	s = API.regwrite(0x0132, DAC_SatRef_val)
	s = API.regwrite(0x0126, 0x0020)
	DAC_SatRef = API.CamMeasureAnaTestVoltage('DAC_SATREF')
	logging.info('\n Verifying DAC_SatRef: ' + str(DAC_SatRef))
	AVRL_dict['DAC_SatRef'] = DAC_SatRef


	# DAC_TX1
	logging.info('DAC_TX1')
	DAC_TX1_x = np.array([0x0080, 0x0088, 0x0090, 0x0098])
	DAC_TX1_y = np.zeros(len(DAC_TX1_x))

	for i in range(len(DAC_TX1_x)):
		s = API.regwrite(0x0132, DAC_TX1_x[i])
		s = API.regwrite(0x0126, 0x0001)
		logging.info('Writing value ' + str(DAC_TX1_x[i]) + ' to register 0x0132')
		logging.info('Writing value 0x0001 to register 0x0126')
		DAC_TX1_y[i] = API.CamMeasureAnaTestVoltage('DAC_TX1')
		logging.info('DAC_TX1_y (DAC_TX1): ' + str(DAC_TX1_y[i]))

	p_TX1 = np.polyfit(DAC_TX1_y, DAC_TX1_x, 1)

	DAC_TX1_val = int(np.round(np.polyval(p_TX1, target_DAC_TX1)))
	logging.info('\n Calculated DAC_TX1_val: ' + str(DAC_TX1_val))

	# Verify DAC_TX1
	s = API.regwrite(0x0132, DAC_TX1_val)
	s = API.regwrite(0x0126, 0x0001)
	DAC_TX1 = API.CamMeasureAnaTestVoltage('DAC_TX1')
	logging.info('\n Verifying DAC_TX1: ' + str(DAC_TX1))
	AVRL_dict['DAC_TX1'] = DAC_TX1


	# DAC_Bias1
	logging.info('DAC_Bias1')
	DAC_Bias1_x = np.array([0x0010, 0x0018, 0x0020, 0x0028])
	DAC_Bias1_y = np.zeros(len(DAC_Bias1_x))

	for i in range(len(DAC_Bias1_x)):
		s = API.regwrite(0x0132, DAC_Bias1_x[i])
		s = API.regwrite(0x0126, 0x0008)
		logging.info('Writing value ' + str(DAC_Bias1_x[i]) + ' to register 0x0132')
		logging.info('Writing value 0x0008 to register 0x0126')
		DAC_Bias1_y[i] = API.CamMeasureAnaTestVoltage('DAC_BIAS1')
		logging.info('DAC_Bias1_y (DAC_BIAS1): ' + str(DAC_Bias1_y[i]))

	p_Bias1 = np.polyfit(DAC_Bias1_y, DAC_Bias1_x, 1)

	DAC_Bias1_val = int(np.round(np.polyval(p_Bias1, target_DAC_Bias1)))
	logging.info('\n Calculated DAC_Bias1_val: ' + str(DAC_Bias1_val))

	# Verify DAC_Bias1
	s = API.regwrite(0x0132, DAC_Bias1_val)
	s = API.regwrite(0x0126, 0x0008)
	DAC_Bias1 = API.CamMeasureAnaTestVoltage('DAC_BIAS1')
	logging.info('\n Verifying DAC_TX1: ' + str(DAC_Bias1))
	AVRL_dict['DAC_Bias1'] = DAC_Bias1


	# DAC_Bias2
	logging.info('DAC_Bias2')
	DAC_Bias2_x = np.array([0x0080, 0x0088, 0x0090, 0x0098])
	DAC_Bias2_y = np.zeros(len(DAC_Bias2_x))

	for i in range(len(DAC_Bias2_x)):
		s = API.regwrite(0x0132, DAC_Bias2_x[i])
		s = API.regwrite(0x0126, 0x0010)
		logging.info('Writing value ' + str(DAC_Bias2_x[i]) + ' to register 0x0132')
		logging.info('Writing value 0x0010 to register 0x0126')
		DAC_Bias2_y[i] = API.CamMeasureAnaTestVoltage('DAC_BIAS2')
		logging.info('DAC_Bias2_y (DAC_BIAS2): ' + str(DAC_Bias2_y[i]))

	p_Bias2 = np.polyfit(DAC_Bias2_y, DAC_Bias2_x, 1)

	DAC_Bias2_val = int(np.round(np.polyval(p_Bias2, target_DAC_Bias2)))
	logging.info('\n Calculated DAC_Bias2_val: ' + str(DAC_Bias2_val))

	# Verify DAC_Bias2
	s = API.regwrite(0x0132, DAC_Bias2_val)
	s = API.regwrite(0x0126, 0x0010)
	DAC_Bias2 = API.CamMeasureAnaTestVoltage('DAC_BIAS2')
	logging.info('\n Verifying DAC_Bias2: ' + str(DAC_Bias2))
	AVRL_dict['DAC_Bias2'] = DAC_Bias2


	# DAC_Vrest
	logging.info('DAC_Vrest')
	DAC_Vrest_x = np.array([0x0070, 0x0078, 0x0080, 0x0088])
	DAC_Vrest_y = np.zeros(len(DAC_Vrest_x))

	for i in range(len(DAC_Vrest_x)):
		s = API.regwrite(0x0132, DAC_Vrest_x[i])
		s = API.regwrite(0x0126, 0x0040)
		logging.info('Writing value ' + str(DAC_Vrest_x[i]) + ' to register 0x0132')
		logging.info('Writing value 0x0040 to register 0x0126')
		DAC_Vrest_y[i] = API.CamMeasureAnaTestVoltage('DAC_VREST')
		logging.info('DAC_Vrest_y (DAC_VREST): ' + str(DAC_Vrest_y[i]))

	p_Vrest = np.polyfit(DAC_Vrest_y, DAC_Vrest_x, 1)

	DAC_Vrest_val = int(np.round(np.polyval(p_Vrest, target_DAC_Vrest)))
	logging.info('\n Calculated DAC_Vrest_val: ' + str(DAC_Vrest_val))

	# Verify DAC_Vrest
	s = API.regwrite(0x0132, DAC_Vrest_val)
	s = API.regwrite(0x0126, 0x0040)
	DAC_Vrest = API.CamMeasureAnaTestVoltage('DAC_VREST')
	logging.info('\n Verifying DAC_Vrest: ' + str(DAC_Vrest))
	AVRL_dict['DAC_Vrest'] = DAC_Vrest

	AddressValueRegisterList = np.array([[0x0172, Vlow_Vadj, 0xFFFF, 0],
										[0x0132, DAC_SatRef_val, 0xFFFF, 0],
										[0x0126, 0x0020, 0xFFFF, 0],
										[0x0132, DAC_TX1_val, 0xFFFF, 0],
										[0x0126, 0x0001, 0xFFFF, 0],
										[0x0132, DAC_Bias1_val, 0xFFFF, 0],
										[0x0126, 0x0008, 0xFFFF, 0],
										[0x0132, DAC_Bias2_val, 0xFFFF, 0],
										[0x0126, 0x0010, 0xFFFF, 0],
										[0x0132, DAC_Vrest_val, 0xFFFF, 0],
										[0x0126, 0x0040, 0xFFFF, 0],
										[0x0246, 0xC1F5, 0x01FF, 0]])

	AVRL_dict['DAC_SatRef_val'] = DAC_SatRef_val
	AVRL_dict['DAC_TX1_val'] = DAC_TX1_val
	AVRL_dict['DAC_Bias1_val'] = DAC_Bias1_val
	AVRL_dict['DAC_Bias2_val'] = DAC_Bias2_val
	AVRL_dict['DAC_Vrest_val'] = DAC_Vrest_val


	AddressValueRegisterList_dict['AddressValueRegisterList'] = AddressValueRegisterList
	AddressValueRegisterList_dict['info']['Pass'] = AVRL_dict['AVRL_pass']


	if AddressValueRegisterList_dict['info']['Pass'] == True:
		device_passed = "PASSED"
		print("\n=================")
		print('VLOW TRIM PASSED')
		print("=================\n")
	else:
		device_passed = "FAILED"
		print("\n=================")
		print('VLOW TRIM FAILED')
		print("=================\n")

	file = os.path.join(folderName, yaml_file['AddressValueRegisterList']['output_file_pkl'])
	f = open(file + ".pkl", 'wb')
	pickle.dump(AddressValueRegisterList_dict, f)
	f.close()

	f = open(os.path.join(folderName, 'AVRL_dict.pkl'), 'wb')
	pickle.dump(AVRL_dict, f)
	f.close()

	file = os.path.join(folderName, yaml_file['AddressValueRegisterList']['output_file_csv'])
	c = csv.writer(open(file + '.csv', 'w', newline=''))
	for i in range(len(AddressValueRegisterList)):
		c.writerow(AddressValueRegisterList[i])

	# Save results to CSV file 
	Vlow_status = dict()
	Vlow_status['Pass/Fail'] = [AVRL_dict['AVRL_pass'], device_passed]
	for key, value in AVRL_dict.items():
		if not isinstance(AVRL_dict[key], bool):
			Vlow_status[key] = value
	df = pd.DataFrame(dict([ (k,pd.Series(v)) for k,v in Vlow_status.items() ]))
	df = df.T
	df.columns = ['Value', 'Result', 'Min Limit', 'Max Limit']
	df.to_csv(os.path.join(folderName, output_file_csv_report + '.csv'))

if __name__ == "__main__":
    VlowDACTrim("./config/VLowDACTrim.yaml", CalibfolderName = "")