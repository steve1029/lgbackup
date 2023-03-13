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


def run_focus_adjustment(CalibfolderName, configurationFile):
	FA.focus_adjust(config_file=configurationFile, CalibfolderName=CalibfolderName)

def run_generate_header(CalibfolderName, configurationFile):
	SessInfo.GenerateYamlSessionFile(config_file=configurationFile, CalibfolderName=CalibfolderName)

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
		print('\n\nEnter configurationFile. ex: python run_FocusAdjust.py ./config/CalibrationConfig_Walden.yaml')
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

	import tof_calib.FocusAdjustment as FA
	import tof_calib.GenerateSessionInfo as SessInfo
	from imatest.it import ImatestLibrary, ImatestException

	# Initialize Imatest Library
	imatestLib = ImatestLibrary()

	run_generate_header(CalibfolderName, configurationFile)
	run_focus_adjustment(CalibfolderName, configurationFile)
	
	imatestLib.terminate_library()

	sys.exit(0)
