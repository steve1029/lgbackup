#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#

import cv2
import numpy as np
import sys
# try:
from . import CalibAPI as tof
# except:
# 	import CalibAPI as tof
# try:
from . import P0_functions as P0

from .result_generation import P0_results as P0_res
# except:
# 	import P0_functions as P0
import time
import matplotlib.pyplot as plt
import pickle
import os
from datetime import datetime
import logging
import yaml
import shutil
import pandas as pd


def verifyLimit(val, limitArray, all_pass):
	'''
	Returns a PASS if val is between the limit array else fail.

	Inputs: 
		val                     - value 
		limitArray              - limit array [min , max] 

	Outputs:
		resultStr               - PASS or FAIL 
	'''

	if (val >= limitArray[0]) and (val <= limitArray[1]):
		return 'PASS', all_pass
	else:
		return 'FAIL', False


def P0_quick_tests(config_file, CalibfolderName, folderName, mode, compress_P0Table = False):
	'''
		Performs quick tests on P0 calibration results using limits from 
		limits.yaml

		Inputs: 
		  config_file           - name of configuration (.yaml) file  
		  CalibfolderName       - name of calbration folder  
		  folderName            - location of P0 saved files 
		  mode                  - Camera mode

		Outputs:
			testDict              - dictionary containing the results

	''' 

	with open(config_file) as y:
		file = yaml.load(y, Loader=yaml.SafeLoader)

	y.close()

	P0_config = file['P0Calibration']['configuration_file']
	cal_settings = file['P0Calibration']['calibration']

	compress_P0Table = cal_settings['compress_P0Table']
	limits_file = file['limits_file']
	mode_info_file = file['mode_info_file']

	try:
		calibration_dir_path = file['calibration_directory']
	except:
		calibration_dir_path = ""

	P0_table_template = file['P0Calibration']['output_file_uncompressed_pkl']
	comp_P0_table_template = file['P0Calibration']['output_file_compressed_pkl']

	print(">>>>>>>>>>>>>", calibration_dir_path)

	if os.path.abspath(P0_config) != P0_config:
		P0_config = os.path.join(calibration_dir_path, P0_config)

	if os.path.abspath(limits_file) != limits_file:
		limits_file = os.path.join(calibration_dir_path, limits_file)

	if os.path.abspath(mode_info_file) != mode_info_file:
		mode_info_file = os.path.join(calibration_dir_path, mode_info_file)

	P0_files = os.listdir(folderName)
	testDict = dict()

	P0_table_file = ""
	depth_mm_file = ""
	AB_file = ""
	mean_frames_file = ""

	comp_P0_table_file = ""
	comp_depth_mm_file = ""
	comp_AB_file = ""
	residue_file = ""
		

	## Compressed P0 table

	if compress_P0Table:
		for P0_file in P0_files:
			if P0_file.startswith(comp_P0_table_template + str(mode)) and P0_file[-4:]=='.pkl':
				comp_P0_table_file = os.path.join(folderName, P0_file)

			if P0_file.startswith('test_mode_' + str(mode) + '_compressedfreqDist') and P0_file[-4:]=='.pkl':
				comp_depth_mm_file = os.path.join(folderName, P0_file)
	
			if P0_file.startswith('test_mode_' + str(mode) + '_compressedAB') and P0_file[-4:]=='.pkl':
				comp_AB_file = os.path.join(folderName, P0_file)
	
			if P0_file.startswith('mean_frames_Mode_' + str(mode)) and P0_file[-4:]=='.pkl':
				mean_frames_file = os.path.join(folderName, P0_file)

			if P0_file.startswith('residue_mm_P0Table_mode_' + str(mode)) and P0_file[-4:]=='.pkl':
				residue_file = os.path.join(folderName, P0_file)
	
		with open(limits_file) as y:
			limits = yaml.load(y, Loader=yaml.SafeLoader)
		y.close()
	
		with open(mode_info_file) as y:
			modeInfo = yaml.load(y, Loader=yaml.SafeLoader)
		y.close()
	
		freqMhzArray = modeInfo['mode'][mode]['freqsMhz']
		maxPhaseSteps = modeInfo['mode'][mode]['maxPhaseSteps']
	
		all_pass = True

		# Compressed Depth Error
		f = open(comp_depth_mm_file, 'rb')
		cde = pickle.load(f)
		f.close()
	
		ref_dist = float(comp_depth_mm_file.split('_')[-1][:-6])

		for f in range(len(freqMhzArray)):
			cde_max = np.round(np.max(cde[:,:,f]) - ref_dist, 4)
			cde_min = np.round(np.min(cde[:,:,f]) - ref_dist, 4)
			cde_limit = limits['P0Calibration']['DepthError_'+str(int(freqMhzArray[f]))+'Mhz']
			cde_max_res, all_pass = verifyLimit(cde_max, cde_limit, all_pass)
			cde_min_res, all_pass = verifyLimit(cde_min, cde_limit, all_pass)

			testDict['Compressed_depth_error_mode_' + str(mode) + '_freq_' + str(freqMhzArray[f]) + '_max_mm'] = [cde_max, cde_limit[0], cde_limit[1], cde_max_res]
			testDict['Compressed_depth_error_mode_' + str(mode) + '_freq_' + str(freqMhzArray[f]) + '_min_mm'] = [cde_min, cde_limit[0], cde_limit[1], cde_min_res]

		del cde
	
		# Compressed AB Value
		f = open(comp_AB_file, 'rb')
		AB = pickle.load(f)
		f.close()
	
		for f in range(len(freqMhzArray)):
			AB_max = np.round(np.max(AB[:,:,f]), 4)
			AB_min = np.round(np.min(AB[:,:,f]), 4)
			AB_max_limit = limits['P0Calibration']['MaxAB_'+str(int(freqMhzArray[f]))+'Mhz']
			AB_min_limit = limits['P0Calibration']['MinAB_'+str(int(freqMhzArray[f]))+'Mhz']
			AB_max_res, all_pass = verifyLimit(AB_max, AB_max_limit, all_pass)
			AB_min_res, all_pass = verifyLimit(AB_min, AB_min_limit, all_pass)
	
			testDict['AB_mode_' + str(mode) + '_freq_' + str(freqMhzArray[f]) + '_max'] = [AB_max, AB_max_limit[0], AB_max_limit[1], AB_max_res]
			testDict['AB_mode_' + str(mode) + '_freq_' + str(freqMhzArray[f]) + '_min'] = [AB_min, AB_min_limit[0], AB_min_limit[1], AB_min_res]
	
		del AB

		# Estimated Error
		f = open(mean_frames_file, 'rb')
		mean_frames = pickle.load(f)
		f.close()
	
		ch = int(modeInfo['mode'][mode]['height'] / 2)
		cw = int(modeInfo['mode'][mode]['width'] / 2)
	
		mean_frames_center = np.zeros([len(freqMhzArray),maxPhaseSteps])
		for i in range(len(freqMhzArray)):
			mean_frames_center[i,:] = mean_frames[ch, cw, i, :]
	
		del mean_frames
	
		est_dist_err = P0_res.estimate_distance(mean_frames_center, freqMhzArray, maxPhaseSteps, ref_dist) * 1000
		est_dist_err_limit = limits['P0Calibration']['EstDepthErr_mode_' + str(int(mode))]
		est_dist_err_max = np.round(np.max(est_dist_err), 4)
		est_dist_err_min = np.round(np.min(est_dist_err), 4)
		est_dist_err_max_res, all_pass = verifyLimit(est_dist_err_max, est_dist_err_limit, all_pass)
		est_dist_err_min_res, all_pass = verifyLimit(est_dist_err_min, est_dist_err_limit, all_pass)
	
		testDict['Est_dist_error_mode_' + str(mode) + '_max_mm'] = [est_dist_err_max, est_dist_err_limit[0], est_dist_err_limit[1], est_dist_err_max_res]
		testDict['Est_dist_error_mode_' + str(mode) + '_min_mm'] = [est_dist_err_min, est_dist_err_limit[0], est_dist_err_limit[1], est_dist_err_min_res]

		# Residue Error
		f = open(residue_file, 'rb')
		residue = pickle.load(f)
		f.close()

		for f in range(len(freqMhzArray)):
			residue_max = np.round(np.max(residue[:,:,f]), 4)
			residue_min = np.round(np.min(residue[:,:,f]), 4)
			residue_limit = limits['P0Calibration']['Residue_Err_'+str(int(freqMhzArray[f]))+'Mhz']
			residue_max_res, all_pass = verifyLimit(residue_max, residue_limit, all_pass)
			residue_min_res, all_pass = verifyLimit(residue_min, residue_limit, all_pass)

			testDict['Residue_' + str(mode) + '_freq_' + str(freqMhzArray[f]) + '_max'] = [residue_max, residue_limit[0], residue_limit[1], residue_max_res]
			testDict['Residue_' + str(mode) + '_freq_' + str(freqMhzArray[f]) + '_min'] = [residue_min, residue_limit[0], residue_limit[1], residue_min_res]

		del residue


	else:
		for P0_file in P0_files:
			if P0_file.startswith(P0_table_template + str(mode)) and P0_file[-4:]=='.pkl':
				P0_table_file = os.path.join(folderName, P0_file)
	
			if P0_file.startswith('test_mode_' + str(mode) + '_freqDist') and P0_file[-4:]=='.pkl':
				depth_mm_file = os.path.join(folderName, P0_file)
	
			if P0_file.startswith('test_mode_' + str(mode) + '_AB') and P0_file[-4:]=='.pkl':
				AB_file = os.path.join(folderName, P0_file)
	
			if P0_file.startswith('mean_frames_Mode_' + str(mode)) and P0_file[-4:]=='.pkl':
				mean_frames_file = os.path.join(folderName, P0_file)
	
		with open(limits_file) as y:
			limits = yaml.load(y, Loader=yaml.SafeLoader)
		y.close()
	
		with open(mode_info_file) as y:
			modeInfo = yaml.load(y, Loader=yaml.SafeLoader)
		y.close()
	
		freqMhzArray = modeInfo['mode'][mode]['freqsMhz']
		maxPhaseSteps = modeInfo['mode'][mode]['maxPhaseSteps']
	
		all_pass = True
	
		# Depth Error
		f = open(depth_mm_file, 'rb')
		de = pickle.load(f)
		f.close()
	
		ref_dist = float(depth_mm_file.split('_')[-1][:-6])
	
		for f in range(len(freqMhzArray)):
			de_max = np.round(np.max(de[:,:,f]) - ref_dist, 4)
			de_min = np.round(np.min(de[:,:,f]) - ref_dist, 4)
			de_limit = limits['P0Calibration']['DepthError_'+str(int(freqMhzArray[f]))+'Mhz']
			de_max_res, all_pass = verifyLimit(de_max, de_limit, all_pass)
			de_min_res, all_pass = verifyLimit(de_min, de_limit, all_pass)
	
			testDict['Depth_error_mode_' + str(mode) + '_freq_' + str(freqMhzArray[f]) + '_max_mm'] = [de_max, de_limit[0], de_limit[1], de_max_res]
			testDict['Depth_error_mode_' + str(mode) + '_freq_' + str(freqMhzArray[f]) + '_min_mm'] = [de_min, de_limit[0], de_limit[1], de_min_res]
	
		del de
	
		# AB Value
		f = open(AB_file, 'rb')
		AB = pickle.load(f)
		f.close()
	
		for f in range(len(freqMhzArray)):
			AB_max = np.round(np.max(AB[:,:,f]), 4)
			AB_min = np.round(np.min(AB[:,:,f]), 4)
			AB_max_limit = limits['P0Calibration']['MaxAB_'+str(int(freqMhzArray[f]))+'Mhz']
			AB_min_limit = limits['P0Calibration']['MinAB_'+str(int(freqMhzArray[f]))+'Mhz']
			AB_max_res, all_pass = verifyLimit(AB_max, AB_max_limit, all_pass)
			AB_min_res, all_pass = verifyLimit(AB_min, AB_min_limit, all_pass)
	
			testDict['AB_mode_' + str(mode) + '_freq_' + str(freqMhzArray[f]) + '_max'] = [AB_max, AB_max_limit[0], AB_max_limit[1], AB_max_res]
			testDict['AB_mode_' + str(mode) + '_freq_' + str(freqMhzArray[f]) + '_min'] = [AB_min, AB_min_limit[0], AB_min_limit[1], AB_min_res]
	
		del AB
	
		# Estimated Error
		f = open(mean_frames_file, 'rb')
		mean_frames = pickle.load(f)
		f.close()
	
		ch = int(modeInfo['mode'][mode]['height'] / 2)
		cw = int(modeInfo['mode'][mode]['width'] / 2)
	
		mean_frames_center = np.zeros([len(freqMhzArray),maxPhaseSteps])
		for i in range(len(freqMhzArray)):
			mean_frames_center[i,:] = mean_frames[ch, cw, i, :]
	
		del mean_frames
	
		est_dist_err = P0_res.estimate_distance(mean_frames_center, freqMhzArray, maxPhaseSteps, ref_dist) * 1000
		est_dist_err_limit = limits['P0Calibration']['EstDepthErr_mode_' + str(int(mode))]
		est_dist_err_max = np.round(np.max(est_dist_err), 4)
		est_dist_err_min = np.round(np.min(est_dist_err), 4)
		est_dist_err_max_res, all_pass = verifyLimit(est_dist_err_max, est_dist_err_limit, all_pass)
		est_dist_err_min_res, all_pass = verifyLimit(est_dist_err_min, est_dist_err_limit, all_pass)
	
		testDict['Est_dist_error_mode_' + str(mode) + '_max_mm'] = [est_dist_err_max, est_dist_err_limit[0], est_dist_err_limit[1], est_dist_err_max_res]
		testDict['Est_dist_error_mode_' + str(mode) + '_min_mm'] = [est_dist_err_min, est_dist_err_limit[0], est_dist_err_limit[1], est_dist_err_min_res]


	print("PARAMETER : Value, min, max, result")
	for key,val in testDict.items():
		print(str(key) + " : " + str(val))

	print("* NOTE: float is rounded to int in dictionary keys")

	if compress_P0Table:
		f = open(comp_P0_table_file, 'rb')
		comp_P0_table_dict = pickle.load(f)
		f.close()

		comp_P0_table_dict['info']['Pass'] = comp_P0_table_dict['info']['Pass'] and all_pass

		f = open(comp_P0_table_file, 'wb')
		pickle.dump(comp_P0_table_dict, f)
		f.close()

	else:
		f = open(P0_table_file, 'rb')
		P0_table_dict = pickle.load(f)
		f.close()
	
		P0_table_dict['info']['Pass'] = P0_table_dict['info']['Pass'] and all_pass
	
		f = open(P0_table_file, 'wb')
		pickle.dump(P0_table_dict, f)
		f.close()

	testDict['All_pass'] = all_pass

	return testDict



# def P0_calibrate(config_file=None, CalibfolderName = ""):

# 	with open(config_file) as y:
# 		file = yaml.load(y, Loader=yaml.SafeLoader)
# 	folderName = os.path.join(CalibfolderName, file['P0Calibration']['output_directory'])
# 	y.close()

# 	dict5 = P0_quick_tests(config_file, CalibfolderName, folderName, 5, True)

# 	dict7 = P0_quick_tests(config_file, CalibfolderName, folderName, 7, True)

# 	prelimTestResults = dict()
# 	prelimTestResults['All_pass'] = True

# 	tmp = prelimTestResults['All_pass']
# 	for key,val in dict5.items():
# 		prelimTestResults[key] = val

# 	prelimTestResults['All_pass'] = prelimTestResults['All_pass'] and tmp

# 	tmp = prelimTestResults['All_pass']

# 	for key,val in dict7.items():
# 		prelimTestResults[key] = val

# 	prelimTestResults['All_pass'] = prelimTestResults['All_pass'] and tmp

# 	prelimTestResults_fname = os.path.join(folderName,'preliminaryTestResults.pkl')
# 	f = open(prelimTestResults_fname, 'wb')
# 	pickle.dump(prelimTestResults, f)
# 	f.close()





def P0_calibrate(config_file=None, CalibfolderName = ""):
	'''
		Top function for P0 calibration
		
		Inputs: 
		  config_file           - name of configuration (.yaml) file 
		  CalibfolderName       - name of calbration folder 
		  
		Outputs: 
		  None 
		
	'''

	with open(config_file) as y:
		file = yaml.load(y, Loader=yaml.SafeLoader)

	P0_config = file['P0Calibration']['configuration_file']
	cal_settings = file['P0Calibration']['calibration']
	verification_settings = file['P0Calibration']['verification']
	archer_settings = file['ArcherSettings'] # MZ: Added Archer setting 

	try:
		calibration_dir_path = file['calibration_directory']
	except:
		calibration_dir_path = ""

	P0_table_template = file['P0Calibration']['output_file_uncompressed_pkl']
	comp_P0_table_template = file['P0Calibration']['output_file_compressed_pkl']


	if os.path.abspath(P0_config) != P0_config:
		P0_config = os.path.join(calibration_dir_path, P0_config)

	if CalibfolderName == "":
		now = datetime.now()
		folderName = 'P0_output_' + str(now.strftime('%Y%m%d_%H%M%S'))
	else:
		folderName = os.path.join(CalibfolderName, file['P0Calibration']['output_directory'])
	
	if not os.path.exists(folderName):
		os.mkdir(folderName)
	else:
		now = datetime.now()
		print('\nALERT: ' + folderName + ' already exists')
		old_folderName = folderName + str(now.strftime('_old_%Y%m%d_%H%M%S'))
		os.rename(folderName, old_folderName)
		print('ALERT: RENAMING OLD FOLDER: ' + folderName + ' as '+ old_folderName +'\n')
		os.mkdir(folderName)

	mode = cal_settings['mode']
	OVERRIDE_LSDACS = cal_settings['OVERRIDE_LSDACS']
	LSDAC_VALUE = cal_settings['LSDAC_VALUE']
	NFrames = cal_settings['NFrames']
	NFrames_warmup = cal_settings['NFrames_warmup']
	distance = cal_settings['distance']
	fps = cal_settings['fps']
	compress_P0Table = cal_settings['compress_P0Table']

	try:
		n_modes_cfg = file['num_modes_cfg']
	except:
		print("\n\nWARNING: n_modes_cfg not found in yaml file, update to latest version of yaml file")
		print("Assuming n_modes_cfg = 10\n\n")
		n_modes_cfg = 10

	verify_calibration = verification_settings['verify_calibration']
	verify_OVERRIDE_LSDAC = verification_settings['verify_OVERRIDE_LSDACS']
	verify_LSDAC_VALUE = verification_settings['verify_LSDAC_VALUE']
	verify_NFrames = verification_settings['verify_NFrames']
	verify_NFrames_warmup = verification_settings['verify_NFrames_warmup']

	IllumModule = file['IlluminationModule'] # MZ: Type of illumination module (Intersil, Archer)
	module_class = file['module_class'] # MZ: Type of Imager Module (Walden_R1, Legacy, Ofilm)
	ArcherRegsConfig = np.array(archer_settings['RegsConfig'],dtype=np.uint16) # MZ: Load Archer regs config
	ArcherSlaveID = archer_settings['SlaveID']

	also_save_image_as_bin = file['P0Calibration']['also_save_image_as_bin']
	output_file_csv_report = file['P0Calibration']['output_file_csv_report']

	y.close()

	P0_calib = P0.P0_calibration(folderName, config_file)
	# create IOLib object and open camera

	if (5 in mode) and (10 in mode):
		#mode = mode[mode != 10]
		mode.remove(10)
		mode10_no_capture = True # Not capturing mode 10 to save time
	else:
		mode10_no_capture = False

	prelimTestResults = dict()
	prelimTestResults['All_pass'] = True

	for m in range(len(mode)):
		cam = tof.CalibAPI(module_class = module_class, n_modes_cfg = n_modes_cfg)
		cam.ConnectSensor(Mode=mode[m], fps=fps, cfg = P0_config)

		if IllumModule == 'ArcherI2C':	# MZ: If Archer driver, write the regs config (I2C only)
			cam.ConfigADSD3000(ArcherSlaveID, ArcherRegsConfig)
		
		# Check if gain and offset calibration file is found
		gainOffsetFileName = str(os.path.join(CalibfolderName, 
											  file['GainOffsetCalibration']['output_directory'],
											  file['GainOffsetCalibration']['output_file_csv'])) + '.csv'
		if os.path.exists(gainOffsetFileName):
			print("\nGain and Offset file found\n")
			cam.LoadGainOffsetCalib(gainOffsetFileName)
		else:
			raise Exception('Gain and Offset calibration must be done before P0 Calibration')
		

		if verify_calibration:
			verify_file = folderName + '/test_mode_' + str(mode[m]) + '.pkl'
			hdrTemp = dict(
							mode = mode[m],
			                nFrames = verify_NFrames[m],
			                nWarmupFrames = verify_NFrames_warmup[m],
			                OVERRIDE_LSDACS = verify_OVERRIDE_LSDAC[m],
			                LSDACValue = verify_LSDAC_VALUE[m],
			                enableSweep = 0,
			                outputFormat = 'pkl',
			                also_save_image_as_bin = also_save_image_as_bin,
			                fileName = verify_file
			                )

			_, test_frames = cam.CaptureFramesP0(hdrTemp)

			print('Test Frames Captured')

			if np.sum(np.isnan(test_frames['Image'])) > 10:
				raise Exception("One or more P0 verification frames are saturated, please check the P0 setup")


		sweep_file = folderName + '/sweep_mode_' + str(mode[m]) + '.pkl'

		hdrTemp = dict(
						mode = mode[m],
		                nFrames = NFrames[m],
		                nWarmupFrames = NFrames_warmup[m],
		                OVERRIDE_LSDACS = OVERRIDE_LSDACS[m],
		                LSDACValue = LSDAC_VALUE[m],
		                enableSweep = 1,
		                outputFormat = 'none',
		                also_save_image_as_bin = False,
		                fileName = sweep_file
		                )

		_,sweep_frames = cam.CaptureFramesP0(hdrTemp)

		if np.sum(np.isnan(sweep_frames['Image'])) > 100:
			raise Exception("One or more P0 sweep frames are saturated, please check the P0 setup")

		now = datetime.now()
		P0Table_dict = dict()
		P0Table_dict['info'] = dict()
		P0Table_dict['info']['CheckSum'] = 0
		P0Table_dict['info']['Year'] = int(now.strftime('%Y'))
		P0Table_dict['info']['Month'] = int(now.strftime('%m'))
		P0Table_dict['info']['Day'] = int(now.strftime('%d'))
		P0Table_dict['info']['Hour'] = int(now.strftime('%H'))
		P0Table_dict['info']['Minute'] = int(now.strftime('%M'))
		P0Table_dict['info']['Second'] = int(now.strftime('%S'))
		P0Table_dict['info']['configuration_file'] = file['P0Calibration']['configuration_file']
		P0Table_dict['info']['Pass'] = 0


		P0Table_file = os.path.join(folderName, file['P0Calibration']['output_file_uncompressed_pkl'] + str(mode[m]) + '.pkl')
		success = P0_calib.generate_P0Table(distance, mode[m], sweep_frames, P0Table_file, P0Table_dict) # calib dist, sweep file, P0 Table file

		if success:
			print("P0 Table generated successfully for MODE " + str(mode[m]))
		# P0_calib.save_P0Table()
		

		cam.DisconnectSensor()

		del cam

		if compress_P0Table:
			comp_P0Table_file = os.path.join(folderName, file['P0Calibration']['output_file_compressed_pkl'] + str(mode[m]) + '.pkl')
			P0_calib.compress_P0Table(P0Table_file, comp_P0Table_file)
			uncomp_P0Table_file = os.path.join(folderName, 'un'+file['P0Calibration']['output_file_compressed_pkl'] + str(mode[m]) + '_2D_P0' + '.pkl')
			P0_calib.uncompress_P0Table(comp_P0Table_file, uncomp_P0Table_file)

		if verify_calibration:
			if compress_P0Table:
				P0_calib.apply_compressed_P0Table(verify_file, comp_P0Table_file, P0Table_file, mode[m])
			else:
				P0_calib.apply_P0Table(verify_file, P0Table_file, mode[m])

		testdict = P0_quick_tests(config_file, CalibfolderName, folderName, int(mode[m]), compress_P0Table)

		tmp = prelimTestResults['All_pass']

		for key, val in testdict.items():
			prelimTestResults[key] = val

		prelimTestResults['All_pass'] = prelimTestResults['All_pass'] and tmp


	if mode10_no_capture:
		print("\nCOPYING MODE 5 FILES AS MODE 10 FILES...\n")
		shutil.copyfile(os.path.join(folderName, file['P0Calibration']['output_file_uncompressed_pkl'] + '5.pkl'),
						os.path.join(folderName, file['P0Calibration']['output_file_uncompressed_pkl'] + '10.pkl'))

		shutil.copyfile(os.path.join(folderName, 'mean_frames_Mode_5.pkl'),
						os.path.join(folderName, 'mean_frames_Mode_10.pkl'))

		if verify_calibration:
			shutil.copyfile(os.path.join(folderName, 'test_mode_5.pkl'),
							os.path.join(folderName, 'test_mode_10.pkl'))

		P0Table_file = os.path.join(folderName, file['P0Calibration']['output_file_uncompressed_pkl'] + '10.pkl')
		verify_file = os.path.join(folderName, 'test_mode_10.pkl')
		
		if compress_P0Table:
			comp_P0Table_file = os.path.join(folderName, file['P0Calibration']['output_file_compressed_pkl'] + '10.pkl')
			P0_calib.compress_P0Table(P0Table_file, comp_P0Table_file)
			uncomp_P0Table_file = os.path.join(folderName, 'un'+file['P0Calibration']['output_file_compressed_pkl'] + '10_2D_P0' + '.pkl')
			P0_calib.uncompress_P0Table(comp_P0Table_file, uncomp_P0Table_file)

		if verify_calibration:
			if compress_P0Table:
				P0_calib.apply_compressed_P0Table(verify_file, comp_P0Table_file, P0Table_file, 10)
			else:
				P0_calib.apply_P0Table(verify_file, P0Table_file, 10)


	if prelimTestResults['All_pass'] == True:
		device_passed = "PASSED"
		print("\n======================")
		print('P0 CALIBRATION PASSED')
		print("======================\n")
	else:
		device_passed = "FAILED"
		print("\n======================")
		print('P0 CALIBRATION FAILED')
		print("======================\n")

	prelimTestResults_fname = os.path.join(folderName,'preliminaryTestResults.pkl')
	f = open(prelimTestResults_fname, 'wb')
	pickle.dump(prelimTestResults, f)
	f.close()

	# Save results to CSV file 
	P0_status = dict()
	P0_status['Pass/Fail'] = [prelimTestResults['All_pass'], device_passed]
	for key, value in prelimTestResults.items():
		if isinstance(prelimTestResults[key], (list,tuple,dict, str)):
			if len(prelimTestResults[key]) <= 4:
				P0_status[key] = value
		elif not isinstance(prelimTestResults[key], bool):
			P0_status[key] = value
	df = pd.DataFrame(dict([ (k,pd.Series(v)) for k,v in P0_status.items() ]))
	df = df.T
	df.columns = ['Value', 'Result', 'Min Limit', 'Max Limit']
	df.to_csv(os.path.join(folderName, output_file_csv_report + '.csv'))


if __name__ == "__main__":
    P0_calibrate("./config/CalibrationConfig.yaml" , CalibfolderName = "")