#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#
import numpy as np
import pickle
import os
import shutil

def image2bin(image, fname):
	"""
	Saves a 2D matrix in bin format
	Inputs:
		image:    2D matrix
		fname:    name of file to be saved

	Outputs:
		none
	"""

	tmp = np.squeeze(np.array(image, dtype=np.float32))
	(h,w) = np.shape(tmp)
	tmp_flattened = tmp.flatten()
	write_file = open(fname +'--'+str(h)+'x'+str(w) +'.bin', "wb")
	tmp_flattened.tofile(write_file, format='%f')

def add_results_blemish(CalibfolderName, calibResult, file, result_dir):
	'''
		Adds results of blemish test

		Inputs:
		  CalibfolderName       - Name of calibration output directory
		  calibResult           - Calibration result dict
		  file                  - Name of .yaml config file
		  result_dir            - Name of result directory
		  
		Outputs:
		  calibResult           - Updated calibration result dict
		
	'''
	print("Generating results for Blemish Test...")

	Blem_dir = os.path.join(CalibfolderName, file['Blemish']['output_directory'])
	Blem_output = os.path.join(Blem_dir, file['Blemish']['output_file_pkl'] + '.pkl')
	if os.path.exists(Blem_dir):
		if os.path.exists(Blem_output):
			f = open(Blem_output, 'rb')
			Blem_dict = pickle.load(f)
			f.close()
			calibResult['Blemish_pass'] = Blem_dict['Blemish_pass']
			calibResult['Blemish_exists'] = True

			calibResult['Blemish'] = dict()

			calibResult['Blemish']['totalHotPixels'] = Blem_dict['totalHotPixels']
			calibResult['Blemish']['totalDeadPixels'] = Blem_dict['totalDeadPixels']
			calibResult['Blemish']['totalBadPixels'] = Blem_dict['totalBadPixels']
			calibResult['Blemish']['badPixelsCenter'] = Blem_dict['badPixelsCenter']
			calibResult['Blemish']['badPixelsSides'] = Blem_dict['badPixelsSides']
			for key, val in Blem_dict['blemishResults'].items():
				calibResult['Blemish'][str(key)] = str(val)

			try:
				dest = shutil.copy(os.path.join(Blem_dir, 'Blemish_original.png'), os.path.join(result_dir, 'Blemish_original.png'))
				dest = shutil.copy(os.path.join(Blem_dir, 'Results', '__blem-countR.png'), os.path.join(result_dir, 'Blemish_countR.png'))
				calibResult['Blemish']['Blemish_original_image'] = os.path.join('Results', 'Blemish_original.png')
				calibResult['Blemish']['Blemish_countR_image'] = os.path.join('Results', 'Blemish_countR.png')
			except:
				pass

	return calibResult