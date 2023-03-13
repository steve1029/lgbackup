#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#

from datetime import datetime
import os
import yaml
import sys
import numpy as np


def WriteEEPROM(CalibfolderName, finalCCB=False):
	import tof_calib.CCB_flash as CCBFlash

	with open(os.path.join(CalibfolderName, 'header.yaml')) as y:
		header = yaml.load(y, Loader=yaml.SafeLoader)

	cfg_file = header['sensor_firmware']
	ccb_file = header['compressed_ccb']
	json_file = header['flashConfigJson']

	status = CCBFlash.WriteBarcode(cfg_file, ccb_file, json_file)

	if finalCCB == True:
		print("camera1.writeModuleEeprom()", status)
		if (status == True):
			print("Flash programming complete")
			header['compressed_ccb_flashed'] = True
			with open(os.path.join(CalibfolderName,'header.yaml'),'w') as f:
				yaml.dump(header, f)
			print("header.yaml updated")
			print("\n=================")
			print("FLASH SUCCESSFUL!")
			print("=================")
		else:
			print("Flash programming failed")

def PrintBarcodeFromEEPROM(CalibfolderName):
	import tof_calib.CCB_flash as CCBFlash

	with open(os.path.join(CalibfolderName, 'header.yaml')) as y:
		header = yaml.load(y, Loader=yaml.SafeLoader)

	cfg_file = header['sensor_firmware']
	ccb_file = header['compressed_ccb']
	json_file = header['flashConfigJson']

	barcode = CCBFlash.ReadBarcode(json_file)

	print("\nBARCODE:", barcode)

	return barcode
	

if __name__ == "__main__":

	if len(sys.argv) == 2:
		configurationFile = os.path.abspath(str(sys.argv[1]))

		try:
			with open(configurationFile) as y:
				yaml_file = yaml.load(y, Loader=yaml.SafeLoader)
		except:
			raise Exception("Please enter a valid *.yaml configuration file")

		now = datetime.now()
		CalibfolderName = yaml_file['module_class'] + '_' + str(now.strftime('%Y%m%d_%H%M%S'))
	elif len(sys.argv) == 3:
		configurationFile = os.path.abspath(str(sys.argv[1]))
		CalibfolderName = os.path.abspath(str(sys.argv[2]))
	else:
		print('\n\nEnter configurationFile. ex: python run_Blemish.py ./config/CalibrationConfig_Walden.yaml')
		sys.exit()

	try:
		with open(configurationFile) as y:
			yaml_file = yaml.load(y, Loader=yaml.SafeLoader)
	except:
		raise Exception("Please enter a valid *.yaml configuration file")


	if not os.path.exists(CalibfolderName):
		os.mkdir(CalibfolderName)


	with open(configurationFile) as y:
		yaml_file = yaml.load(y, Loader=yaml.SafeLoader)
		calibration_dir_path = yaml_file["calibration_directory"]

	sys.path.append(calibration_dir_path)

	calibration_SW_version = np.genfromtxt(os.path.join(calibration_dir_path,'version.txt'),dtype='str')
	print("\n=============================")
	print("    ToF Calibration " + str(calibration_SW_version))
	print("=============================\n")

	import tof_calib.CCB_flash as CCBFlash
	import tof_calib.GenerateSessionInfo as SessInfo

	SessInfo.GenerateYamlSessionFile(config_file=configurationFile, CalibfolderName=CalibfolderName)

	WriteEEPROM(CalibfolderName=CalibfolderName, finalCCB = True)

	sys.exit(0)
