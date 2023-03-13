#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#
import numpy as np
import pickle
import os
import cv2


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

def add_results_geometric(CalibfolderName, calibResult, file, result_dir):
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
	print("Generating results for Geometric Calibration...")

	Geo_dir = os.path.join(CalibfolderName, file['GeometricCalibration']['output_directory'])
	Geo_output = os.path.join(Geo_dir, file['GeometricCalibration']['output_file_pkl'] + '.pkl')
	if os.path.exists(Geo_dir):
		if os.path.exists(Geo_output):
			f = open(Geo_output, 'rb')
			Geo_dict = pickle.load(f)
			f.close()
			calibResult['Geometric_pass'] = Geo_dict['Geometric_pass']
			calibResult['Geometric_exists'] = True

			calibResult['Geometric'] = dict()
			calibResult['Geometric']['Fc1'] = Geo_dict['Fc1']
			calibResult['Geometric']['Fc2'] = Geo_dict['Fc2']
			calibResult['Geometric']['cc1'] = Geo_dict['cc1']
			calibResult['Geometric']['cc2'] = Geo_dict['cc2']
			calibResult['Geometric']['Cx'] = Geo_dict['Cx']
			calibResult['Geometric']['Cy'] = Geo_dict['Cy']
			calibResult['Geometric']['Kc1'] = Geo_dict['Kc1']
			calibResult['Geometric']['Kc2'] = Geo_dict['Kc2']
			calibResult['Geometric']['Kc3'] = Geo_dict['Kc3']
			calibResult['Geometric']['Kc4'] = Geo_dict['Kc4']
			calibResult['Geometric']['Kc5'] = Geo_dict['Kc5']
			calibResult['Geometric']['Kc6'] = Geo_dict['Kc6']
			calibResult['Geometric']['Tx'] = Geo_dict['Tx']
			calibResult['Geometric']['Ty'] = Geo_dict['Ty']
			try:
				calibResult['Geometric']['Rerr'] = Geo_dict['Rerr']
			except:
				pass

			try:
				Geo_frames = os.path.join(Geo_dir, file['GeometricCalibration']['output_file_frames_pkl'] + '.pkl')
				f = open(Geo_frames, 'rb')
				Img = pickle.load(f)['Image']
				avg_frame = np.mean(Img, axis = 3)
				avg_frame = np.float32(np.squeeze(avg_frame))
				# print(np.max(avg_frame),np.min(avg_frame))
				avg_frame = (avg_frame - np.min(avg_frame))
				avg_frame = avg_frame / np.max(avg_frame)
				avg_frame2 = np.uint8(avg_frame * 255)
				cv2.imwrite(os.path.join(result_dir, 'checkerboard.png'), avg_frame2)
				image2bin(avg_frame, os.path.join(result_dir, 'checkerboardRAW'))
				calibResult['Geometric']['checkerboard']= os.path.join('Results', 'checkerboard.png')
			except:
				pass

	return calibResult