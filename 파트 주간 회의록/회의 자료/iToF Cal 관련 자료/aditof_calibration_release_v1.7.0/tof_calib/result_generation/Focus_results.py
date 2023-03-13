#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#
import numpy as np
import pickle
import os
import shutil
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

	tmp = np.squeeze(np.array(image, dtype=np.float32))
	(h,w) = np.shape(tmp)
	tmp_flattened = tmp.flatten()
	write_file = open(fname +'--'+str(h)+'x'+str(w) +'.bin', "wb")
	tmp_flattened.tofile(write_file, format='%f')

def add_results_focus(CalibfolderName, calibResult, file, result_dir):
	'''
		Adds results of focus verification
		Inputs:
		  CalibfolderName       - Name of calibration output directory
		  calibResult           - Calibration result dict
		  file                  - Name of .yaml config file
		  result_dir            - Name of result directory
		  
		Outputs:
		  calibResult           - Updated calibration result dict
		
	'''
	print("Generating results for Focus verification...")

	Focus_dir = os.path.join(CalibfolderName, file['FocusVerification']['output_directory'])
	Focus_output = os.path.join(Focus_dir, file['FocusVerification']['output_file_pkl'] + '.pkl')
	if os.path.exists(Focus_dir):
		if os.path.exists(Focus_output):
			f = open(Focus_output, 'rb')
			Focus_dict = pickle.load(f)
			f.close()
			calibResult['Focus_pass'] = Focus_dict['Focus_pass']
			calibResult['Focus_exists'] = True

			sfrplusResults = Focus_dict['sfrplusResults']

			calibResult['Focus'] = dict()

			calibResult['Focus']['MTF50Center'] = Focus_dict['MTF50Center']
			calibResult['Focus']['MTF30Center'] = Focus_dict['MTF30Center']
			calibResult['Focus']['MTF0p5NyqCenter'] = Focus_dict['MTF0p5NyqCenter']
			calibResult['Focus']['MTF30WtdMean'] = Focus_dict['MTF30WtdMean']

			#Full Imatest summary
			calibResult['Focus']['version'] = str(sfrplusResults['version'])
			calibResult['Focus']['mtfPeak'] = str(sfrplusResults['mtfPeak'])
			calibResult['Focus']['mtf50'] = str(sfrplusResults['mtf50'])
			calibResult['Focus']['mtf50p'] = str(sfrplusResults['mtf50p'])
			calibResult['Focus']['mtf30'] = str(sfrplusResults['mtf30'])
			calibResult['Focus']['mtf30p'] = str(sfrplusResults['mtf30p'])
			calibResult['Focus']['mtf20'] = str(sfrplusResults['mtf20'])
			calibResult['Focus']['mtf20p'] = str(sfrplusResults['mtf20p'])
			calibResult['Focus']['mtf10'] = str(sfrplusResults['mtf10'])
			calibResult['Focus']['mtf10p'] = str(sfrplusResults['mtf10p'])
			calibResult['Focus']['secondary_readout_1'] = str(sfrplusResults['secondary_readout_1'])
			calibResult['Focus']['secondary_1_results'] = str(sfrplusResults['secondary_1_results'])
			calibResult['Focus']['secondary_1_min_max_ratio'] = str(sfrplusResults['secondary_1_min_max_ratio'])
			calibResult['Focus']['secondary_readout_2'] = str(sfrplusResults['secondary_readout_2'])
			calibResult['Focus']['secondary_2_results'] = str(sfrplusResults['secondary_2_results'])
			calibResult['Focus']['secondary_2_min_max_ratio'] = str(sfrplusResults['secondary_2_min_max_ratio'])
			calibResult['Focus']['secondary_readout_3'] = str(sfrplusResults['secondary_readout_3'])
			calibResult['Focus']['secondary_3_results'] = str(sfrplusResults['secondary_3_results'])
			calibResult['Focus']['quadrants'] = str(sfrplusResults['quadrants'])
			calibResult['Focus']['secondary_1_quadrant_mean'] = str(sfrplusResults['secondary_1_quadrant_mean'])
			calibResult['Focus']['secondary_1_quadrant_min'] = str(sfrplusResults['secondary_1_quadrant_min'])
			calibResult['Focus']['secondary_1_quadrant_min_location'] = str(sfrplusResults['secondary_1_quadrant_min_location'])
			calibResult['Focus']['secondary_1_quadrant_max'] = str(sfrplusResults['secondary_1_quadrant_max'])
			calibResult['Focus']['secondary_1_quadrant_max_location'] = str(sfrplusResults['secondary_1_quadrant_max_location'])
			calibResult['Focus']['secondary_2_quadrant_mean'] = str(sfrplusResults['secondary_2_quadrant_mean'])
			calibResult['Focus']['secondary_2_quadrant_min'] = str(sfrplusResults['secondary_2_quadrant_min'])
			calibResult['Focus']['secondary_2_quadrant_min_location'] = str(sfrplusResults['secondary_2_quadrant_min_location'])
			calibResult['Focus']['secondary_2_quadrant_max'] = str(sfrplusResults['secondary_2_quadrant_max'])
			calibResult['Focus']['secondary_2_quadrant_max_location'] = str(sfrplusResults['secondary_2_quadrant_max_location'])
			calibResult['Focus']['summaryRegions'] = str(sfrplusResults['summaryRegions'])
			calibResult['Focus']['region_weights'] = str(sfrplusResults['region_weights'])
			calibResult['Focus']['mtf_summary_specified_units'] = str(sfrplusResults['mtf_summary_specified_units'])
			calibResult['Focus']['mtf50_CP_summary'] = str(sfrplusResults['mtf50_CP_summary'])
			calibResult['Focus']['mtf50_specified_units_summary'] = str(sfrplusResults['mtf50_specified_units_summary'])
			calibResult['Focus']['mtf50_LWPH_summary'] = str(sfrplusResults['mtf50_LWPH_summary'])
			calibResult['Focus']['mtf50p_CP_summary'] = str(sfrplusResults['mtf50p_CP_summary'])
			calibResult['Focus']['mtf50p_specified_units_summary'] = str(sfrplusResults['mtf50p_specified_units_summary'])
			calibResult['Focus']['mtf50p_LWPH_summary'] = str(sfrplusResults['mtf50p_LWPH_summary'])
			calibResult['Focus']['mtf20_CP_summary'] = str(sfrplusResults['mtf20_CP_summary'])
			calibResult['Focus']['mtf20_specified_units_summary'] = str(sfrplusResults['mtf20_specified_units_summary'])
			calibResult['Focus']['mtf20_LWPH_summary'] = str(sfrplusResults['mtf20_LWPH_summary'])
			calibResult['Focus']['riseDistPxls_summary'] = str(sfrplusResults['riseDistPxls_summary'])
			calibResult['Focus']['overSharpening_Pct_summary'] = str(sfrplusResults['overSharpening_Pct_summary'])
			calibResult['Focus']['overshoot_Pct_summary'] = str(sfrplusResults['overshoot_Pct_summary'])
			calibResult['Focus']['CA_areaPxls_summary'] = str(sfrplusResults['CA_areaPxls_summary'])
			calibResult['Focus']['riseDist_PH_summary'] = str(sfrplusResults['riseDist_PH_summary'])
			calibResult['Focus']['peakMTF_summary'] = str(sfrplusResults['peakMTF_summary'])
			calibResult['Focus']['LSF_PW50_pxls_summary'] = str(sfrplusResults['LSF_PW50_pxls_summary'])
			calibResult['Focus']['CA_areaPCT_summary'] = str(sfrplusResults['CA_areaPCT_summary'])
			calibResult['Focus']['CA_crossingPCT_summary'] = str(sfrplusResults['CA_crossingPCT_summary'])
			calibResult['Focus']['CA_R_G_PCT_summary'] = str(sfrplusResults['CA_R_G_PCT_summary'])
			calibResult['Focus']['CA_B_G_PCT_summary'] = str(sfrplusResults['CA_B_G_PCT_summary'])
			calibResult['Focus']['CA_R_G_Pixels_summary'] = str(sfrplusResults['CA_R_G_Pixels_summary'])
			calibResult['Focus']['CA_B_G_Pixels_summary'] = str(sfrplusResults['CA_B_G_Pixels_summary'])
			calibResult['Focus']['secondary_1_summary'] = str(sfrplusResults['secondary_1_summary'])
			calibResult['Focus']['secondary_2_summary'] = str(sfrplusResults['secondary_2_summary'])
			calibResult['Focus']['edgeRoughSTD_summary'] = str(sfrplusResults['edgeRoughSTD_summary'])
			calibResult['Focus']['MTF_Area_PkNorm_summary'] = str(sfrplusResults['MTF_Area_PkNorm_summary'])

			try:
				dest = shutil.copy(os.path.join(Focus_dir, 'focus_verification.png'), os.path.join(result_dir, 'focus_verification.png'))
				dest = shutil.copy(os.path.join(Focus_dir, 'Results', '__Y_multi_cpp.png'), os.path.join(result_dir, 'Y_multi_cpp.png'))
				dest = shutil.copy(os.path.join(Focus_dir, 'Results', '__Y_profiles.png'), os.path.join(result_dir, 'Y_profiles.png'))
				calibResult['Focus']['Focus_verification_image'] = os.path.join('Results', 'focus_verification.png')
				calibResult['Focus']['Y_multi_cpp_image'] = os.path.join('Results', 'Y_multi_cpp.png')
				calibResult['Focus']['Y_profiles_image'] = os.path.join('Results', 'Y_profiles.png')
			except:
				pass

	return calibResult