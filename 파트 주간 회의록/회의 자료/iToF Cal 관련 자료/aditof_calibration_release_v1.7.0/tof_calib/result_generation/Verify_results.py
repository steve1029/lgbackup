#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#
import numpy as np
import pickle
import os
import cv2
import shutil
import matplotlib.pyplot as plt
import yaml


def image2bin(image, fname):
	"""
	Saves a 2D matrix in bin format
	Inputs:
		image:    2D matrix
		fname:    name of file to be saved

	Outputs:
		none
	"""

	tmp = np.squeeze(np.array(image, dtype=np.int16))
	(h,w) = np.shape(tmp)
	tmp_flattened = tmp.flatten()
	write_file = open(fname +'--'+str(h)+'x'+str(w) +'.bin', "wb")
	tmp_flattened.tofile(write_file, format='%d')

def add_results_verify(CalibfolderName, calibResult, file, result_dir):
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
	print("Generating results for Calibration Verification...")

	ver_dir = os.path.join(CalibfolderName, file['Calibration_verification']['output_directory'])
	ver_output = os.path.join(ver_dir, file['Calibration_verification']['output_file_pkl']+'.pkl')

	if os.path.exists(ver_dir):
		if os.path.exists(ver_output):

			f = open(ver_output, 'rb')
			ver_dict = pickle.load(f)
			f.close()

			calibResult['Verify'] = dict()
			calibResult['Cal_verify_exists'] = True
			calibResult['Cal_verify_pass'] = ver_dict['Verify_pass']
			
			for key in ver_dict.keys():
				if key == 'info':
					pass
				else:
					calibResult['Verify'][key] = ver_dict[key]

			AB_frames_fname = os.path.join(ver_dir, 'AB.pkl')
			f = open(AB_frames_fname, 'rb')
			AB_frames = pickle.load(f)
			f.close()

			image2bin(AB_frames, os.path.join(result_dir, 'verify_AB'))

			plt.figure()
			plt.imshow(AB_frames, cmap = 'gray')
			plt.colorbar()
			plt.title('Active Brightness')
			#plt.show()

			plt.savefig(os.path.join(result_dir, 'verify_AB.png'))
			calibResult['Verify']['AB'] = os.path.join('Results', 'verify_AB.png')

			rad_depth_frames_fname = os.path.join(ver_dir, 'RadialDepth.pkl')
			f = open(rad_depth_frames_fname, 'rb')
			rad_depth_frames = pickle.load(f)
			f.close()

			image2bin(rad_depth_frames, os.path.join(result_dir, 'verify_RadialDepth'))

			plt.figure()
			plt.imshow(rad_depth_frames)
			plt.colorbar()
			plt.title('Radial Depth (mm)')
			plt.savefig(os.path.join(result_dir, 'verify_RadialDepth.png'))
			calibResult['Verify']['Rad_depth'] = os.path.join('Results', 'verify_RadialDepth.png')

			Z_depth_frames_fname = os.path.join(ver_dir, 'ZDepth.pkl')
			f = open(Z_depth_frames_fname, 'rb')
			Z_depth_frames = pickle.load(f)
			f.close()

			image2bin(Z_depth_frames, os.path.join(result_dir, 'verify_ZDepth'))

			plt.figure()
			plt.imshow(Z_depth_frames)
			plt.colorbar()
			plt.title('Z Depth (mm)')
			plt.savefig(os.path.join(result_dir, 'verify_ZDepth.png'))
			calibResult['Verify']['Z_depth'] = os.path.join('Results', 'verify_ZDepth.png')

			dest = shutil.copy(os.path.join(ver_dir, 'XYZ.bin'), os.path.join(result_dir, 'verify_XYZ.bin'))
			dest = shutil.copy(os.path.join(ver_dir, 'fitted_plane.png'), os.path.join(result_dir, 'fitted_plane.png'))
			calibResult['Verify']['XYZ'] = os.path.join('Results', 'fitted_plane.png')
			dest = shutil.copy(os.path.join(ver_dir, 'AB_stats.png'), os.path.join(result_dir, 'AB_stats.png'))
			calibResult['Verify']['AB_stats'] = os.path.join('Results', 'AB_stats.png')
			dest = shutil.copy(os.path.join(ver_dir, 'AB_profiles.png'), os.path.join(result_dir, 'AB_profiles.png'))
			calibResult['Verify']['AB_profiles'] = os.path.join('Results', 'AB_profiles.png')
			dest = shutil.copy(os.path.join(ver_dir, 'AB_hist.png'), os.path.join(result_dir, 'AB_hist.png'))
			calibResult['Verify']['AB_hist'] = os.path.join('Results', 'AB_hist.png')

	return calibResult