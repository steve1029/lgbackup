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

from imatest.it import ImatestLibrary, ImatestException

# Initialize Imatest Library
imatestLib = ImatestLibrary()

def run_test_frames(CalibfolderName, configurationFile):
	test_frames.test_frames(config_file=configurationFile, CalibfolderName=CalibfolderName)

def run_gainoffset_calibration(CalibfolderName, configurationFile):
	gainoffset.gain_offset_calibrate(config_file=configurationFile, CalibfolderName=CalibfolderName)

def run_VlowDACTrim(CalibfolderName, configurationFile):
	VlowDACTrim.VlowDACTrim(configurationFile, CalibfolderName=CalibfolderName)

def run_P0_calibration(CalibfolderName, configurationFile):
	P0.P0_calibrate(config_file=configurationFile, CalibfolderName=CalibfolderName)

def run_geo_calibration(CalibfolderName, configurationFile):
	geo.Geometric_calibrate(config_file=configurationFile, CalibfolderName=CalibfolderName)

def run_LSDAC_setting(CalibfolderName, configurationFile):
	LSDAC.GenerateLSDACSettingsfromcsv(config_file=configurationFile, CalibfolderName=CalibfolderName)

def run_focus_adjustment(CalibfolderName, configurationFile):
	FA.focus_adjust(config_file=configurationFile, CalibfolderName=CalibfolderName)

def run_focus_verification(CalibfolderName, configurationFile):
	FV.focus_verify(config_file=configurationFile, CalibfolderName=CalibfolderName)

def run_blemish_test(CalibfolderName, configurationFile):
	Blemish.blemish_test(config_file=configurationFile, CalibfolderName=CalibfolderName)

def run_barcode(CalibfolderName, configurationFile):
	print("\n\nENTER 'ERASE' TO ERASE, '-1' TO USE DEFAULT BARCODE")
	barcode = str(input("ENTER BARCODE:"))
	barcode = barcode.lower()

	CCBWr.CCB_write(config_file=configurationFile, CalibfolderName=CalibfolderName, barcode_ip=barcode)
	CCBFlash.WriteEEPROM(CalibfolderName=CalibfolderName, finalCCB=False)

def run_CCB_write(CalibfolderName, configurationFile):
	#Read any barcode present in NVM
	barcode = CCBFlash.PrintBarcodeFromEEPROM(CalibfolderName)
	CCBWr.CCB_write(config_file=configurationFile, CalibfolderName=CalibfolderName, barcode_ip=barcode)


def run_CCB_flash(CalibfolderName, configurationFile):
	CCBFlash.WriteEEPROM(CalibfolderName=CalibfolderName, finalCCB=True)

def run_barcode_read(CalibfolderName, configurationFile):
	CCBFlash.PrintBarcodeFromEEPROM(CalibfolderName=CalibfolderName)

def run_generate_header(CalibfolderName, configurationFile):
	SessInfo.GenerateYamlSessionFile(config_file=configurationFile, CalibfolderName=CalibfolderName)

def run_verify_calibration(CalibfolderName, configurationFile):
	vcal.FixedDistVerification(config_file=configurationFile, CalibfolderName=CalibfolderName)

# def run_fallout(CalibfolderName, configurationFile):
# 	try:
# 		dc.depth_compute(config_file=configurationFile, CalibfolderName=CalibfolderName)
# 	except:
# 		print('\nALERT: Fallout not installed. Depth compute not available.')

def generate_report(CalibfolderName, configurationFile):
	res.generate_results(config_file=configurationFile, CalibfolderName=CalibfolderName)

def addStatus(option, status):
	return (option + (35-len(option)) * ' ' + status)

if __name__ == "__main__":

	now = datetime.now()

	if len(sys.argv) == 2:
		configurationFile = os.path.abspath(str(sys.argv[1]))

		try:
			with open(configurationFile) as y:
				yaml_file = yaml.load(y, Loader=yaml.SafeLoader)
		except:
			raise Exception("Please enter a valid *.yaml configuration file")

		CalibfolderName = yaml_file['module_class'] + '_' + str(now.strftime('%Y%m%d_%H%M%S'))
	elif len(sys.argv) == 3:
		configurationFile = os.path.abspath(str(sys.argv[1]))
		CalibfolderName = os.path.abspath(str(sys.argv[2]))
	else:
		print('\n\nEnter configurationFile. ex: python run_calibration.py <path to config yaml file>')
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

	import tof_calib.GainOffsetCalibrate as gainoffset
	import tof_calib.P0Calibrate as P0
	import tof_calib.VlowDACTrim as VlowDACTrim
	import tof_calib.CCB_writer as CCBWr
	import run_CCB_flash as CCBFlash
	import tof_calib.GenerateLSDACSettingsfromcsv as LSDAC
	import tof_calib.GeometricCalibrate as geo
	import tof_calib.test_TOF_frames as test_frames
	import tof_calib.FocusAdjustment as FA
	import tof_calib.FocusVerification as FV
	import tof_calib.CameraBlemish as Blemish
	import tof_calib.ScanFiles as Scan
	import tof_calib.GenerateSessionInfo as SessInfo
	import tof_calib.VerifyCalibration as vcal
	import tof_calib.GenerateResults as res


	run_generate_header(CalibfolderName, configurationFile)

	calibration_SW_version = str(np.genfromtxt(os.path.join(calibration_dir_path,'version.txt'),dtype='str'))


	options = np.zeros(10, dtype=object)
	options[0] = "1 -> CONNECTIVITY TEST and ICQ"
	options[1] = "2 -> GAIN AND OFFSET CALIBRATION"
	options[2] = "3 -> VLOW AND DAC TRIM"
	options[3] = "4 -> BLEMISH TEST"
	options[4] = "5 -> FOCUS ADJUSTMENT"
	options[5] = "6 -> FOCUS VERIFICATION"
	options[6] = "7 -> GEOMETRIC CALIBRATION"
	options[7] = "8 -> LSDAC SETTING"
	options[8] = "9 -> P0 CALIBRATION"
	options[9] = "V -> VERIFY CALIBRATION"

	while(True):
		status = Scan.GetCalibrationStatus(configurationFile, CalibfolderName)
		print("\n=============================")
		print("    ToF Calibration " + str(calibration_SW_version))
		print("=============================\n")
		print("Calibration file directory created->", CalibfolderName)
		print("Configuration file used ->", configurationFile,'\n')
		print("OPTIONS:")
		print(addStatus(options[0], status['1']))
		print(addStatus(options[1], status['2']))
		print(addStatus(options[2], status['3']))
		print(addStatus(options[3], status['4']))
		print(addStatus(options[4], status['5']))
		print(addStatus(options[5], status['6']))
		print(addStatus(options[6], status['7']))
		print(addStatus(options[7], status['8']))
		print(addStatus(options[8], status['9']))
		print("B -> WRITE BARCODE **")
		print("RB -> READ BARCODE")
		print("C -> GENERATE CCB")
		print("M -> FLASH CCB TO MODULE")
		print(addStatus(options[9], status['10']))
		print("R -> GENERATE REPORT")
		print("Q -> QUIT")
		print("\n** STORES A BARCODE, GENERATES A CCB AND FLASHES IT INTO MEMORY\n")
		a = str(input("\n\nSelect option:"))


		if a == '1':
			run_test_frames(CalibfolderName, configurationFile)
		elif a == '2':
			run_gainoffset_calibration(CalibfolderName, configurationFile)
		elif a == '3':
			run_VlowDACTrim(CalibfolderName, configurationFile)
		elif a == '4':
			run_blemish_test(CalibfolderName, configurationFile)
		elif a == '5':
			run_focus_adjustment(CalibfolderName, configurationFile)
		elif a == '6':
			run_focus_verification(CalibfolderName, configurationFile)
		elif a == '7':
			run_geo_calibration(CalibfolderName, configurationFile)
		elif a == '8':
			run_LSDAC_setting(CalibfolderName, configurationFile)
		elif a == '9':
			run_P0_calibration(CalibfolderName, configurationFile)
		elif a == 'b' or a == 'B':
			run_barcode(CalibfolderName, configurationFile)
		elif a == 'rb' or a == 'RB':
			run_barcode_read(CalibfolderName, configurationFile)
		elif a == 'c' or a == 'C':
			run_CCB_write(CalibfolderName, configurationFile)
		elif a == 'm' or a == 'M':
			run_CCB_flash(CalibfolderName, configurationFile)
		elif a == 'v'or a == 'V':
			run_verify_calibration(CalibfolderName, configurationFile)
		elif a == 'r' or a == 'R':
			generate_report(CalibfolderName, configurationFile)
		elif a == 'q' or a == 'Q':
			imatestLib.terminate_library()
			break
		else:
			print("INVALID OPTION \n\n\n")
