#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#

import numpy as np
import pickle
import os
from scipy.constants import c, pi
from scipy import signal as sig
from datetime import datetime
import yaml
import matplotlib.pyplot as plt
import yaml
# from .. import P0_functions as P0


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


def synthesize_waveform_from_fourier_coefficients(nh, cn, f_fund, Fs, t_start,
                                                  t_end):
    """
    nh: harmonic indexes
    cn: Fourier coefficients
    f_fund: frequency of fundamental
    Fs: sampling frequency
    t_start: start time
    t_end: end time
    """

    # Sampling period
    Ts = 1 / Fs

    # Sample times
    t = np.arange(t_start, t_end, Ts)

    # Harmonic frequencies
    fh = nh * f_fund

    # Inverse Fourier transform
    yt = np.zeros(t.shape, dtype='complex')
    for ii in range(len(fh)):
        yt += cn[ii] * np.exp(1j * t * 2 * pi * fh[ii])

    # Return time stamps and waveform
    return ([t, yt])


def sample_correlation(nh, cn, phase):
    """
    nh: harmonic indexes
    cn: Fourier coefficients
    phase: phase at which to sample correlation (in radian)
    """

    # Inverse Fourier transform
    V = 0
    for ii in range(len(nh)):
        V += cn[ii] * np.exp(1j * phase * nh[ii])

    # Return correlation sample
    return (np.real(V))


def generate_fft_waveform(fft_array):
	"""
	fft_array : 1D array containing fft
	Output : FFT with positive and negative components added
	"""
	fft_array2 = np.append(fft_array, [0])[::-1][:-1]
	return (fft_array + fft_array2)[:int(len(fft_array)/2)]


def estimate_distance(mean_frames_center, freqs, maxPhaseSteps, ref_dist):
	'''
	Estimates the distance given mean frames, i,e frames averaged for 
	phase steps 
		Inputs:
			mean_frames_center   - 2D array of mean frames 
			freqs                - Array of frequencies 
			maxPhaseSteps        - Maximum phase steps 

		Outputs:
			estimated_dist_error 
	'''

	dist_range = np.linspace(0, 5, 1000)
	n_dist = len(dist_range)
	# freqs = [198e6, 189e6, 54e6]
	freqs = np.array(freqs) * 1e6
	n_freqs = len(freqs)

	cn = np.zeros([n_freqs, maxPhaseSteps])
	cn = np.fft.fftshift(np.fft.fft(mean_frames_center) / maxPhaseSteps)

	# Nh = 12
	# nh = np.arange(-12, 12)
	Nh = int(maxPhaseSteps/2)
	nh = np.arange(-int(maxPhaseSteps/2), int(maxPhaseSteps/2))
	# [t,
	#  yt] = synthesize_waveform_from_fourier_coefficients(nh, cn[0, :], freqs[0],
	#                                                      10e9, 0, 3 / freqs[0])

	V0 = np.zeros([n_freqs, n_dist])
	V120 = np.zeros([n_freqs, n_dist])
	V240 = np.zeros([n_freqs, n_dist])

	for ii, f in enumerate(freqs):
		for jj, dist in enumerate(dist_range):
			# Generate laser waveforms harmonics
			T_cycle = 1 / f
			dt = dist * 2 / c / T_cycle * 2 * pi

			# Sample correlation
			V0[ii, jj] = sample_correlation(nh, cn[ii, :], +dt)
			V120[ii, jj] = sample_correlation(nh, cn[ii, :], 2 * pi / 3 + dt)
			V240[ii, jj] = sample_correlation(nh, cn[ii, :], 4 * pi / 3 + dt)

	re = 2 * V0 - V120 - V240
	im = np.sqrt(3) * (V240 - V120)
	C = re + 1j * im
	# # Calculate estimate distance assuming perfect unwrapping
	# est_dist_0 = c / (4 * pi * freqs[0]) * np.unwrap(np.angle(C[0, :]))
	# est_dist_1 = c / (4 * pi * freqs[1]) * np.unwrap(np.angle(C[1, :]))
	# est_dist_2 = c / (4 * pi * freqs[2]) * np.unwrap(np.angle(C[2, :]))

	# # Apply "P0 calibration" the distance measurement of each frequency
	# P0_dist = ref_dist / 1000.00
	# dist_idx_P0 = (np.abs(dist_range - P0_dist)).argmin()
	# est_dist_0 -= est_dist_0[dist_idx_P0] - P0_dist
	# est_dist_1 -= est_dist_1[dist_idx_P0] - P0_dist
	# est_dist_2 -= est_dist_2[dist_idx_P0] - P0_dist

	# # Weighted sum for final distance calculation
	# est_dist = est_dist_0 * freqs[0]**2 / np.sum(
	#     np.array(freqs)**2) + est_dist_1 * freqs[1]**2 / np.sum(
	#         np.array(freqs)**2) + est_dist_2 * freqs[2]**2 / np.sum(
	#             np.array(freqs)**2)

	est_dist_n = np.zeros(n_freqs, dtype=object)
	# Calculate estimate distance assuming perfect unwrapping
	for i in range(n_freqs):
		est_dist_n[i] = c / (4 * pi * freqs[i]) * np.unwrap(np.angle(C[i, :]))

	# Apply "P0 calibration" the distance measurement of each frequency
	P0_dist = ref_dist / 1000.00
	dist_idx_P0 = (np.abs(dist_range - P0_dist)).argmin()

	for i in range(n_freqs):
		est_dist_n[i] -= est_dist_n[i][dist_idx_P0] - P0_dist

	# Weighted sum for final distance calculation
	est_dist = np.zeros(np.shape(est_dist_n[0]))
	for i in range(n_freqs):
		est_dist = est_dist + est_dist_n[i] * freqs[i]**2 / np.sum(np.array(freqs)**2)

	return est_dist - dist_range



def add_results_P0(CalibfolderName, calibResult, file, result_dir, config_file, reducedP0report):
	'''
		Adds results of P0 calibration

		Inputs:
		  CalibfolderName       - Name of calibration output directory
		  calibResult           - Calibration result dict
		  file                  - Name of .yaml config file
		  result_dir            - Name of result directory
		  
		Outputs:
		  calibResult           - Updated calibration result dict
		
	'''


	try:
		with open(config_file) as y:
			file = yaml.load(y, Loader=yaml.SafeLoader)

		mode_info_file_path = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '../..', file['mode_info_file']))

		#with open(file['mode_info_file']) as f:
		with open(mode_info_file_path) as f:
			modeInfo = yaml.load(f, Loader=yaml.SafeLoader)
	except:
		# with open(modeInfo) as y:
		# 	modeInfo = yaml.load(y, Loader=yaml.SafeLoader)
		pass


	print("Generating results for P0 Calibration...")

	P0_dir = os.path.join(CalibfolderName, file['P0Calibration']['output_directory'])
	P0_uncompressed_output = os.path.join(P0_dir, file['P0Calibration']['output_file_uncompressed_pkl'])
	P0_compressed_output = os.path.join(P0_dir, file['P0Calibration']['output_file_compressed_pkl'])

	# freqs = [198, 189, 54]

	if os.path.exists(P0_dir):
		P0_files = os.listdir(P0_dir)

		P0Table_modes = np.array([], dtype =np.int8)
		for P0_file in P0_files:
			if P0_file.startswith(file['P0Calibration']['output_file_uncompressed_pkl']) and P0_file[-4:]=='.pkl':
				P0Table_modes = np.append(P0Table_modes, int(P0_file.split('_')[-1][:-4]))

		compressed_P0Table_modes = np.array([], dtype =np.int8)
		for P0_file in P0_files:
			if P0_file.startswith(file['P0Calibration']['output_file_compressed_pkl']) and P0_file[-4:]=='.pkl':
				compressed_P0Table_modes = np.append(compressed_P0Table_modes, int(P0_file.split('_')[-1][:-4]))

		calibResult['P0'] = dict()
		
		try:
			P0_prelim = os.path.join(P0_dir, 'preliminaryTestResults.pkl')
			f = open(P0_prelim, 'rb')
			prelim = pickle.load(f)
			f.close()
			calibResult['P0']['PreliminaryTestResults'] = dict()

			for key,val in prelim.items():
				calibResult['P0']['PreliminaryTestResults'][key] = val

			calibResult['P0']['PreliminaryTestResults_exists'] = True
		except:
			calibResult['P0']['PreliminaryTestResults_exists'] = False

		if len(P0Table_modes) != 0:
			calibResult['P0_pass'] = True

			if calibResult['P0']['PreliminaryTestResults_exists']:
				calibResult['P0_pass'] = calibResult['P0_pass'] and calibResult['P0']['PreliminaryTestResults']['All_pass']

			calibResult['P0_exists'] = True

			P0Table_modes = sorted(P0Table_modes)
		else:
			calibResult['P0_pass'] = False

		if len(compressed_P0Table_modes) != 0:
			calibResult['comp_P0_exists'] = True

			compressed_P0Table_modes = sorted(compressed_P0Table_modes)


		if calibResult['P0_exists']:
			

			calibResult['P0']['P0Table_uncompressed_mode_0_exists'] = False
			calibResult['P0']['P0Table_uncompressed_mode_1_exists'] = False
			calibResult['P0']['P0Table_uncompressed_mode_4_exists'] = False
			calibResult['P0']['P0Table_uncompressed_mode_5_exists'] = False
			calibResult['P0']['P0Table_uncompressed_mode_7_exists'] = False
			calibResult['P0']['P0Table_uncompressed_mode_10_exists'] = False

			print("--Generating P0 Table results...")
			for m in P0Table_modes:
				calibResult['P0']['P0Table_uncompressed_mode_' + str(m) + '_exists'] = True
				f = open(P0_uncompressed_output + str(m) + '.pkl', 'rb')
				try:
					P0_tab = pickle.load(f)['P0Table']
				except:
					P0_tab = pickle.load(f)

				f.close()

				freqs = modeInfo['mode'][int(m)]['freqsMhz']

				P0_ax = np.zeros(len(freqs), dtype=object)
				P0_img = np.zeros(len(freqs), dtype=object)

				# P0_calib = P0.P0_calibration()

				figureSize = 2*len(freqs)

				fig = plt.figure(figsize=(15, figureSize))
				for i in range(len(freqs)):
					P0_ax[i] = fig.add_subplot(1, len(freqs), i+1)
					P0_img[i] = P0_ax[i].imshow(P0_tab[:,:,i])
					image2bin(P0_tab[:,:,i], os.path.join(result_dir, file['P0Calibration']['output_file_uncompressed_pkl'] + str(m) + '_' + str(i)))
					plt.title('P0 table - ' + str(freqs[i]) + 'Mhz')
					fig.colorbar(P0_img[i], fraction=0.046, pad=0.08,  orientation="horizontal")
					P0_ax[i].autoscale(False)

				plt.savefig(os.path.join(result_dir, file['P0Calibration']['output_file_uncompressed_pkl'] + str(m) + '.png'))
				plt.close()
				calibResult['P0']['P0Table_uncompressed_mode_' + str(m)] = os.path.join('Results', file['P0Calibration']['output_file_uncompressed_pkl'] + str(m) + '.png')
				del P0_tab

		verify_modes = np.array([], dtype =np.int8)
		# verify_dists = np.array([], dtype =np.int16)
		verify_AB_files = np.array([], dtype =object)
		verify_dist_files = np.array([], dtype =object)
		mean_frames_files = np.array([], dtype =object)

		for P0_file in P0_files:
			if ('_AB' in P0_file) and P0_file[-4:]=='.pkl':
				verify_modes = np.append(verify_modes, int(P0_file.split('_')[2]))
				# verify_dists = np.append(verify_dists, int(P0_file.split('_')[-1][:-6]))
				verify_AB_files = np.append(verify_AB_files, P0_file)

			if ('_freqDist' in P0_file) and P0_file[-4:]=='.pkl':
				verify_dist_files = np.append(verify_dist_files, P0_file)

			if ('mean_frames' in P0_file) and P0_file[-4:]=='.pkl':
				 mean_frames_files = np.append(mean_frames_files, P0_file)

		calibResult['P0']['P0Table_verify_mode_0_exists'] = False
		calibResult['P0']['P0Table_verify_mode_1_exists'] = False
		calibResult['P0']['P0Table_verify_mode_4_exists'] = False
		calibResult['P0']['P0Table_verify_mode_5_exists'] = False
		calibResult['P0']['P0Table_verify_mode_7_exists'] = False
		calibResult['P0']['P0Table_verify_mode_10_exists'] = False

		print("--Generating P0 verification(depth) results...")

		if calibResult['P0_exists'] and len(verify_modes)!= 0:
			for m in verify_modes:
				calibResult['P0']['P0Table_verify_mode_' + str(m) +'_exists'] = True

		for dist_file in verify_dist_files:
			ref_dist = float(dist_file.split('_')[-1][:-6])
			ref_mode = int(dist_file.split('_')[2])
			f = open(os.path.join(P0_dir, dist_file), 'rb')
			dist = np.array(pickle.load(f))
			f.close()

			cw = int(np.shape(dist)[0] / 2)
			ch = int(np.shape(dist)[1] / 2)

			freqs = modeInfo['mode'][int(ref_mode)]['freqsMhz']
			
			meanROI_dist = np.zeros(len(freqs))
			min_dist = np.zeros(len(freqs))
			max_dist = np.zeros(len(freqs))

			for i in range(len(freqs)):
				meanROI_dist[i] = np.round(np.nanmean(dist[:,:,i][cw-5:cw+5,ch-5:ch+5]),4)
				min_dist[i] = meanROI_dist[i] - 0.5*meanROI_dist[i]
				max_dist[i] = meanROI_dist[i] + 0.5*meanROI_dist[i]
				dist[:,:,i] = np.clip(dist[:,:,i], min_dist[i], max_dist[i])


			dist[np.isnan(dist)] = 0

			DWP0_ax = np.zeros(len(freqs), dtype=object)
			PDW0_img = np.zeros(len(freqs), dtype=object)


			figureSize = 2*len(freqs)+1

			fig = plt.figure(figsize=(15, figureSize))
			for i in range(len(freqs)):
				DWP0_ax[i] = fig.add_subplot(1, len(freqs), i+1)
				PDW0_img[i] = DWP0_ax[i].imshow(dist[:,:,i])
				image2bin(dist[:,:,i], os.path.join(result_dir, dist_file[:-4] + '_' + str(i)))
				plt.title('Depth w/P0- ' + str(freqs[i]) + 'Mhz (' + str(ref_dist) + 'mm)')
				fig.colorbar(PDW0_img[i], fraction=0.046, pad=0.08,  orientation="horizontal")
				DWP0_ax[i].text(0.5,-0.4, "Mean 10x10 ROI = " + str(meanROI_dist[i]), size=8, ha="center", transform=DWP0_ax[i].transAxes)
				DWP0_ax[i].autoscale(False)


			plt.savefig(os.path.join(result_dir, dist_file[:-4] + '.png'))
			plt.close()
			calibResult['P0']['P0Table_verify_dist_mode_' + str(ref_mode)]= os.path.join('Results', dist_file[:-4] + '.png')

			if reducedP0report == True:
				#Limited P0 report, skip remaining part of loop
				continue

			# Depth error

			DE_ax = np.zeros(len(freqs), dtype=object)
			PE_img = np.zeros(len(freqs), dtype=object)

			figureSize = 2*len(freqs)

			fig = plt.figure(figsize=(15, figureSize))
			for i in range(len(freqs)):
				DE_ax[i] = fig.add_subplot(1, len(freqs), i+1)
				PE_img[i] = DE_ax[i].imshow(np.float(ref_dist) - dist[:,:,i])
				image2bin(np.float(ref_dist) - dist[:,:,i], os.path.join(result_dir, 'depth_error_mode_' + str(ref_mode) + '_' + str(i)))
				plt.title('Depth error - ' + str(freqs[i]) + 'Mhz')
				fig.colorbar(PE_img[i], fraction=0.046, pad=0.08,  orientation="horizontal")
				DE_ax[i].autoscale(False)


			plt.savefig(os.path.join(result_dir, 'depth_error_mode_' + str(ref_mode) + '.png'))
			plt.close()
			calibResult['P0']['P0Table_verify_dist_error_mode_' + str(ref_mode)]= os.path.join('Results', 'depth_error_mode_' + str(ref_mode) + '.png')

			# Depth error histogram

			DEH_ax = np.zeros(len(freqs), dtype=object)
			PEH_img = np.zeros(len(freqs), dtype=object)

			figureSize = 2*len(freqs)

			fig = plt.figure(figsize=(15, figureSize))
			for i in range(len(freqs)):
				de = np.float(ref_dist) - dist[:,:,i]
				de_hist = np.hstack(de.flatten())
				if i>0:
					DEH_ax[i] = fig.add_subplot(1, len(freqs), i+1, sharey=DEH_ax[0])
				else:
					DEH_ax[i] = fig.add_subplot(1, len(freqs), i+1)
				PEH_img[i] = DEH_ax[i].hist(de_hist, bins = 'auto')
				plt.title('Depth error histogram - ' + str(freqs[i]) + 'Mhz')
				DEH_ax[i].autoscale(True)

			plt.savefig(os.path.join(result_dir, 'depth_error_hist_mode_' + str(ref_mode) + '.png'))
			plt.close()
			calibResult['P0']['P0Table_verify_dist_error_hist_mode_' + str(ref_mode)]= os.path.join('Results', 'depth_error_hist_mode_' + str(ref_mode) + '.png')

			del dist


		if reducedP0report == True:
			#Exit P0 report generation here for reduced report
			return calibResult
			
		print("--Generating P0 verification(AB) results...")
		for AB_file in verify_AB_files:
			ref_mode = int(AB_file.split('_')[2])
			f = open(os.path.join(P0_dir, AB_file), 'rb')
			AB = pickle.load(f)
			f.close()

			cw = int(np.shape(AB)[0] / 2)
			ch = int(np.shape(AB)[1] / 2)

			freqs = modeInfo['mode'][int(ref_mode)]['freqsMhz']
			
			meanROI_AB = np.zeros(len(freqs))

			for i in range(len(freqs)):
				meanROI_AB[i] = np.round(np.nanmean(AB[:,:,i][cw-5:cw+5,ch-5:ch+5]),4)

			DWP0_ax = np.zeros(len(freqs), dtype=object)
			PDW0_img = np.zeros(len(freqs), dtype=object)


			figureSize = 2*len(freqs)+1

			fig = plt.figure(figsize=(15, figureSize))
			for i in range(len(freqs)):
				DWP0_ax[i] = fig.add_subplot(1, len(freqs), i+1)
				PDW0_img[i] = DWP0_ax[i].imshow(AB[:,:,i], vmin=0, vmax=2*meanROI_AB[i], cmap = 'gray')
				image2bin(AB[:,:,i], os.path.join(result_dir, AB_file[:-4] + '_' + str(i)))
				plt.title('AB - ' + str(freqs[i]) + 'Mhz')
				fig.colorbar(PDW0_img[i], fraction=0.046, pad=0.08,  orientation="horizontal")
				DWP0_ax[i].text(0.5,-0.4, "Mean 10x10 ROI = " + str(meanROI_AB[i]), size=8, ha="center", transform=DWP0_ax[i].transAxes)
				DWP0_ax[i].autoscale(False)


			plt.savefig(os.path.join(result_dir, AB_file[:-4] + '.png'))
			plt.close()
			calibResult['P0']['P0Table_verify_AB_mode_' + str(ref_mode)]= os.path.join('Results', AB_file[:-4] + '.png')



			# AB histogram


			DEH_ax = np.zeros(len(freqs), dtype=object)
			PEH_img = np.zeros(len(freqs), dtype=object)

			figureSize = 2*len(freqs)

			fig = plt.figure(figsize=(15, figureSize))
			for i in range(len(freqs)):
				de_hist = np.hstack(AB[:,:,i].flatten())
				DEH_ax[i] = fig.add_subplot(1, len(freqs), i+1)
				PEH_img[i] = DEH_ax[i].hist(de_hist, bins = 'auto')
				plt.title('AB histogram - ' + str(freqs[i]) + 'Mhz')
				DEH_ax[i].autoscale(True)

			plt.savefig(os.path.join(result_dir, 'AB_hist_mode_' + str(ref_mode) + '.png'))
			plt.close()
			calibResult['P0']['P0Table_verify_AB_hist_mode_' + str(ref_mode)]= os.path.join('Results', 'AB_hist_mode_' + str(ref_mode) + '.png')

			del AB


		# COMPRESSED P0

		if calibResult['P0_exists'] and calibResult['comp_P0_exists']:

			calibResult['P0']['P0Table_compressed_mode_0_exists'] = False
			calibResult['P0']['P0Table_compressed_mode_1_exists'] = False
			calibResult['P0']['P0Table_compressed_mode_4_exists'] = False
			calibResult['P0']['P0Table_compressed_mode_5_exists'] = False
			calibResult['P0']['P0Table_compressed_mode_7_exists'] = False
			calibResult['P0']['P0Table_compressed_mode_10_exists'] = False

			print("--Generating P0 Table compressed results...")

			for m in compressed_P0Table_modes:
				calibResult['P0']['P0Table_compressed_mode_' + str(m) + '_exists'] = True
				f = open(P0_compressed_output + str(m) + '.pkl', 'rb')
				P0_RC = pickle.load(f)
				f.close()

				keys = list(P0_RC.keys())
				n_freqs = 0
				try:
					for i in range(len(keys)):
						if keys[i].startswith('freq'):
							n_freqs = n_freqs + 1
				except:
					raise Exception("Invalid compressed P0 table")


				# Row coefficients
				# freqs = [198, 189, 54]
				freqs = modeInfo['mode'][int(m)]['freqsMhz']
				row_ax = np.zeros(n_freqs, dtype=object)
				row_img = np.zeros(n_freqs, dtype=object)

				fig = plt.figure(figsize=(15, 6))
				for i in range(n_freqs):
					row_ax[i] = fig.add_subplot(1, n_freqs, i+1)
					row_img[i] = row_ax[i].plot(P0_RC['freq_' + str(i)]['P0RowCoefs'])
					plt.title('P0 Row Coeff - ' + str(freqs[i]) + 'Mhz')

				plt.savefig(os.path.join(result_dir, file['P0Calibration']['output_file_compressed_pkl'] + str(m) + '_row_coeff' + '.png'))
				plt.close()
				calibResult['P0']['P0_row_coeff_compressed_mode_' + str(m)] = os.path.join('Results', file['P0Calibration']['output_file_compressed_pkl'] + str(m) + '_row_coeff' + '.png')

				# Column efficients
				col_ax = np.zeros(n_freqs, dtype=object)
				col_img = np.zeros(n_freqs, dtype=object)

				fig = plt.figure(figsize=(15, 6))
				for i in range(n_freqs):
					col_ax[i] = fig.add_subplot(1, n_freqs, i+1)
					col_img[i] = col_ax[i].plot(P0_RC['freq_' + str(i)]['P0ColCoefs'])
					plt.title('P0 Col Coeff - ' + str(freqs[i]) + 'Mhz')

				plt.savefig(os.path.join(result_dir, file['P0Calibration']['output_file_compressed_pkl'] + str(m) + '_col_coeff' + '.png'))
				plt.close()
				calibResult['P0']['P0_col_coeff_compressed_mode_' + str(m)] = os.path.join('Results', file['P0Calibration']['output_file_compressed_pkl'] + str(m) + '_col_coeff' + '.png')

				# Uncompressed-comprssed P0 Tables
				dirname, base = os.path.split(P0_compressed_output)
				fname_2D_P0 = os.path.join(dirname, 'un'+base + str(m) + '_2D_P0' + '.pkl')
				try:	
					f = open(fname_2D_P0, 'rb')
					P0_2D = pickle.load(f)['P0Table']
					f.close()
				except:
					if str(m) == '10':
						fname_2D_P0 = os.path.join(dirname, 'un'+base + str(5) + '_2D_P0' + '.pkl')
					f = open(fname_2D_P0, 'rb')
					P0_2D = pickle.load(f)['P0Table']
					f.close()

				cP0_ax = np.zeros(n_freqs, dtype=object)
				cP0_img = np.zeros(n_freqs, dtype=object)

				# P0_calib = P0.P0_calibration()

				fig = plt.figure(figsize=(15, 6))
				for i in range(n_freqs):
					cP0_ax[i] = fig.add_subplot(1, n_freqs, i+1)
					# matrix = P0_calib.Uncompress2Matrix(P0_RC['freq_' + str(i)]['P0RowCoefs'], P0_RC['freq_' + str(i)]['P0ColCoefs'])
					cP0_img[i] = cP0_ax[i].imshow(P0_2D[:,:,i])
					image2bin(P0_2D[:,:,i], os.path.join(result_dir, 'un'+file['P0Calibration']['output_file_compressed_pkl'] + str(m) + '_2D_P0' + '_' + str(i)))
					plt.title('P0 compressed table - ' + str(freqs[i]) + 'Mhz')
					fig.colorbar(cP0_img[i], fraction=0.046, pad=0.08,  orientation="horizontal")
					cP0_ax[i].autoscale(False)

				plt.savefig(os.path.join(result_dir, 'un'+file['P0Calibration']['output_file_compressed_pkl'] + str(m) + '_2D_P0' + '.png'))
				plt.close()
				calibResult['P0']['P0_2D_compressed_mode_' + str(m)] = os.path.join('Results', 'un'+file['P0Calibration']['output_file_compressed_pkl'] + str(m) + '_2D_P0' + '.png')

				del P0_2D


		print("--Generating P0 compressed verification(depth) results...")
		# P0 Compression verification
		comp_verify_modes = np.array([], dtype =np.int8)
		# verify_dists = np.array([], dtype =np.int16)
		comp_verify_AB_files = np.array([], dtype =object)
		comp_verify_dist_files = np.array([], dtype =object)
		residue_rad_files = np.array([], dtype =object)
		residue_mm_files = np.array([], dtype =object)

		for P0_file in P0_files:
			if ('_compressedAB' in P0_file) and P0_file[-4:]=='.pkl':
				comp_verify_modes = np.append(comp_verify_modes, int(P0_file.split('_')[2]))
				comp_verify_AB_files = np.append(comp_verify_AB_files, P0_file)

			if ('_compressedfreqDist' in P0_file) and P0_file[-4:]=='.pkl':
				comp_verify_dist_files = np.append(comp_verify_dist_files, P0_file)

			if ('residue_rad' in P0_file) and P0_file[-4:]=='.pkl':
				residue_rad_files = np.append(residue_rad_files, P0_file)

			if ('residue_mm' in P0_file) and P0_file[-4:]=='.pkl':
				residue_mm_files = np.append(residue_mm_files, P0_file)

		calibResult['P0']['P0Table_comp_verify_mode_0_exists'] = False
		calibResult['P0']['P0Table_comp_verify_mode_1_exists'] = False
		calibResult['P0']['P0Table_comp_verify_mode_4_exists'] = False
		calibResult['P0']['P0Table_comp_verify_mode_5_exists'] = False
		calibResult['P0']['P0Table_comp_verify_mode_7_exists'] = False
		calibResult['P0']['P0Table_comp_verify_mode_10_exists'] = False

		if calibResult['P0_exists'] and len(comp_verify_modes)!= 0:
			for m in comp_verify_modes:
				calibResult['P0']['P0Table_comp_verify_mode_' + str(m) +'_exists'] = True

		for dist_file in comp_verify_dist_files:
			ref_dist = float(dist_file.split('_')[-1][:-6])
			ref_mode = int(dist_file.split('_')[2])
			f = open(os.path.join(P0_dir, dist_file), 'rb')
			dist_comp = np.array(pickle.load(f))
			f.close()

			cw = int(np.shape(dist_comp)[0] / 2)
			ch = int(np.shape(dist_comp)[1] / 2)

			freqs = modeInfo['mode'][int(ref_mode)]['freqsMhz']
			
			meanROI_dist = np.zeros(len(freqs))
			min_dist = np.zeros(len(freqs))
			max_dist = np.zeros(len(freqs))

			for i in range(len(freqs)):
				meanROI_dist[i] = np.round(np.nanmean(dist_comp[:,:,i][cw-5:cw+5,ch-5:ch+5]),4)
				min_dist[i] = meanROI_dist[i] - 0.5*meanROI_dist[i]
				max_dist[i] = meanROI_dist[i] + 0.5*meanROI_dist[i]
				dist_comp[:,:,i] = np.clip(dist_comp[:,:,i], min_dist[i], max_dist[i])


			dist_comp[np.isnan(dist_comp)] = 0

			DWP0_ax = np.zeros(len(freqs), dtype=object)
			PDW0_img = np.zeros(len(freqs), dtype=object)


			figureSize = 2*len(freqs)+1

			fig = plt.figure(figsize=(15, figureSize))
			for i in range(len(freqs)):
				DWP0_ax[i] = fig.add_subplot(1, len(freqs), i+1)
				PDW0_img[i] = DWP0_ax[i].imshow(dist_comp[:,:,i])
				image2bin(dist_comp[:,:,i], os.path.join(result_dir, dist_file[:-4] + '_' + str(i)))
				plt.title('Depth w/comp_P0- ' + str(freqs[i]) + 'Mhz (' + str(ref_dist) + 'mm)')
				fig.colorbar(PDW0_img[i], fraction=0.046, pad=0.08,  orientation="horizontal")
				DWP0_ax[i].text(0.5,-0.4, "Mean 10x10 ROI = " + str(meanROI_dist[i]), size=8, ha="center", transform=DWP0_ax[i].transAxes)
				DWP0_ax[i].autoscale(False)


			plt.savefig(os.path.join(result_dir, dist_file[:-4] + '.png'))
			plt.close()
			calibResult['P0']['P0Table_comp_verify_dist_mode_' + str(ref_mode)]= os.path.join('Results', dist_file[:-4] + '.png')

			# Depth error


			DE_ax = np.zeros(len(freqs), dtype=object)
			PE_img = np.zeros(len(freqs), dtype=object)

			figureSize = 2*len(freqs)

			fig = plt.figure(figsize=(15, figureSize))
			for i in range(len(freqs)):
				DE_ax[i] = fig.add_subplot(1, len(freqs), i+1)
				PE_img[i] = DE_ax[i].imshow(np.float(ref_dist) - dist_comp[:,:,i])
				image2bin(np.float(ref_dist) - dist_comp[:,:,i], os.path.join(result_dir, 'depth_error_mode_' + str(ref_mode) + '_compressed'+ '_' + str(i)))
				plt.title('Depth error w/comp - ' + str(freqs[i]) + 'Mhz')
				fig.colorbar(PE_img[i], fraction=0.046, pad=0.08,  orientation="horizontal")
				DE_ax[i].autoscale(False)


			plt.savefig(os.path.join(result_dir, 'depth_error_mode_' + str(ref_mode) + '_compressed'+ '.png'))
			plt.close()
			calibResult['P0']['P0Table_comp_verify_dist_error_mode_' + str(ref_mode)]= os.path.join('Results', 'depth_error_mode_' + str(ref_mode) + '_compressed'+ '.png')

			# Depth error histogram

			DEH_ax = np.zeros(len(freqs), dtype=object)
			PEH_img = np.zeros(len(freqs), dtype=object)

			figureSize = 2*len(freqs)

			fig = plt.figure(figsize=(15, figureSize))
			for i in range(len(freqs)):
				de = np.float(ref_dist) - dist_comp[:,:,i]
				de_hist = np.hstack(de.flatten())
				if i>0:
					DEH_ax[i] = fig.add_subplot(1, len(freqs), i+1,sharey=DEH_ax[0])
				else:
					DEH_ax[i] = fig.add_subplot(1, len(freqs), i+1)
				PEH_img[i] = DEH_ax[i].hist(de_hist, bins = 'auto')
				plt.title('Depth error histogram w/comp - ' + str(freqs[i]) + 'Mhz')
				DEH_ax[i].autoscale(True)


			plt.savefig(os.path.join(result_dir, 'depth_error_hist_mode_' + str(ref_mode) + '_compressed.png'))
			plt.close()
			calibResult['P0']['P0Table_comp_verify_dist_error_hist_mode_' + str(ref_mode)]= os.path.join('Results', 'depth_error_hist_mode_' + str(ref_mode) + '_compressed.png')

			del dist_comp


		print("--Generating P0 compressed verification(AB) results...")
		for AB_file in comp_verify_AB_files:
			ref_mode = int(AB_file.split('_')[2])
			f = open(os.path.join(P0_dir, AB_file), 'rb')
			AB = pickle.load(f)
			f.close()

			cw = int(np.shape(AB)[0] / 2)
			ch = int(np.shape(AB)[1] / 2)

			freqs = modeInfo['mode'][int(ref_mode)]['freqsMhz']
			
			meanROI_AB = np.zeros(len(freqs))

			for i in range(len(freqs)):
				meanROI_AB[i] = np.round(np.nanmean(AB[:,:,i][cw-5:cw+5,ch-5:ch+5]),4)

			DWP0_ax = np.zeros(len(freqs), dtype=object)
			PDW0_img = np.zeros(len(freqs), dtype=object)


			figureSize = 2*len(freqs)+1

			fig = plt.figure(figsize=(15, figureSize))
			for i in range(len(freqs)):
				DWP0_ax[i] = fig.add_subplot(1, len(freqs), i+1)
				PDW0_img[i] = DWP0_ax[i].imshow(AB[:,:,i], vmin=0, vmax=2*meanROI_AB[i], cmap = 'gray')
				image2bin(AB[:,:,i], os.path.join(result_dir, AB_file[:-4] + '_' + str(i)))
				plt.title('AB - ' + str(freqs[i]) + 'Mhz')
				fig.colorbar(PDW0_img[i], fraction=0.046, pad=0.08,  orientation="horizontal")
				DWP0_ax[i].text(0.5,-0.4, "Mean 10x10 ROI = " + str(meanROI_AB[i]), size=8, ha="center", transform=DWP0_ax[i].transAxes)
				DWP0_ax[i].autoscale(False)


			plt.savefig(os.path.join(result_dir, AB_file[:-4] + '.png'))
			plt.close()
			calibResult['P0']['P0Table_comp_verify_AB_mode_' + str(ref_mode)]= os.path.join('Results', AB_file[:-4] + '.png')

			# AB histogram

			DEH_ax = np.zeros(len(freqs), dtype=object)
			PEH_img = np.zeros(len(freqs), dtype=object)

			figureSize = 2*len(freqs)

			fig = plt.figure(figsize=(15, figureSize))
			for i in range(len(freqs)):
				de_hist = np.hstack(AB[:,:,i].flatten())
				DEH_ax[i] = fig.add_subplot(1, len(freqs), i+1)
				PEH_img[i] = DEH_ax[i].hist(de_hist, bins = 'auto')
				plt.title('AB histogram - ' + str(freqs[i]) + 'Mhz')
				DEH_ax[i].autoscale(True)

			plt.savefig(os.path.join(result_dir, 'AB_hist_mode_' + str(ref_mode) + '_compressed' + '.png'))
			plt.close()
			calibResult['P0']['P0Table_comp_verify_AB_hist_mode_' + str(ref_mode)]= os.path.join('Results', 'AB_hist_mode_' + str(ref_mode) + '_compressed' + '.png')

			del AB


		print("--Generating P0 residue results...")
		# Residue Radian
		for residue_rad_file in residue_rad_files:
			residue_mode = int(residue_rad_file[:-4].split('_')[-1])
			f = open(os.path.join(P0_dir, residue_rad_file), 'rb')
			residue_rad = np.array(pickle.load(f))
			f.close()

			freqs = modeInfo['mode'][int(residue_mode)]['freqsMhz']

			res_ax = np.zeros(len(freqs), dtype=object)
			res_img = np.zeros(len(freqs), dtype=object)

			figureSize = 2*len(freqs)

			fig = plt.figure(figsize=(15, figureSize))
			for i in range(len(freqs)):
				res_ax[i] = fig.add_subplot(1, len(freqs), i+1)
				res_img[i] = res_ax[i].imshow(residue_rad[:,:,i])
				image2bin(residue_rad[:,:,i], os.path.join(result_dir, 'residue_rad_mode_' + str(residue_mode) + '_' + str(i)))
				plt.title('Residue in radian- ' + str(freqs[i]) + 'Mhz')
				fig.colorbar(res_img[i], fraction=0.046, pad=0.08,  orientation="horizontal")
				res_ax[i].autoscale(False)


			plt.savefig(os.path.join(result_dir, 'residue_rad_mode_' + str(residue_mode) + '.png'))
			plt.close()
			calibResult['P0']['P0Table_residue_rad_mode_' + str(residue_mode)]= os.path.join('Results', 'residue_rad_mode_' + str(residue_mode) + '.png')

		# Residue mm
		for residue_mm_file in residue_mm_files:
			residue_mode = int(residue_mm_file[:-4].split('_')[-1])
			f = open(os.path.join(P0_dir, residue_mm_file), 'rb')
			residue_mm = np.array(pickle.load(f))
			f.close()

			freqs = modeInfo['mode'][int(residue_mode)]['freqsMhz']

			res_ax = np.zeros(len(freqs), dtype=object)
			res_img = np.zeros(len(freqs), dtype=object)

			figureSize = 2*len(freqs)

			fig = plt.figure(figsize=(15, figureSize))
			for i in range(len(freqs)):
				res_ax[i] = fig.add_subplot(1, len(freqs), i+1)
				res_img[i] = res_ax[i].imshow(residue_mm[:,:,i])
				image2bin(residue_mm[:,:,i], os.path.join(result_dir, 'residue_mm_mode_' + str(residue_mode) + '_' + str(i)))
				plt.title('Residue in mm- ' + str(freqs[i]) + 'Mhz')
				fig.colorbar(res_img[i], fraction=0.046, pad=0.08,  orientation="horizontal")
				res_ax[i].autoscale(False)



			plt.savefig(os.path.join(result_dir, 'residue_mm_mode_' + str(residue_mode) + '.png'))
			plt.close()
			calibResult['P0']['P0Table_residue_mm_mode_' + str(residue_mode)]= os.path.join('Results', 'residue_mm_mode_' + str(residue_mode) + '.png')


		# Mean frames

		print("--Generating P0 mean frames results...")
		calibResult['P0']['P0Table_mean_frames_mode_0_exists'] = False
		calibResult['P0']['P0Table_mean_frames_mode_1_exists'] = False
		calibResult['P0']['P0Table_mean_frames_mode_4_exists'] = False
		calibResult['P0']['P0Table_mean_frames_mode_5_exists'] = False
		calibResult['P0']['P0Table_mean_frames_mode_7_exists'] = False
		calibResult['P0']['P0Table_mean_frames_mode_10_exists'] = False

		if calibResult['P0_exists'] and len(mean_frames_files)!= 0:
			for mean_file in mean_frames_files:
				calibResult['P0']['P0Table_mean_frames_mode_' + str(mean_file.split('_')[3][:-4]) +'_exists'] = True
				# print('P0Table_mean_frames_mode_' + str(mean_file.split('_')[3][:-4]) +'_exists')

		for mean_frames_file in mean_frames_files:
			ref_mode = int(mean_frames_file.split('_')[3][:-4])
			f = open(os.path.join(P0_dir, mean_frames_file), 'rb')
			mean_frames = pickle.load(f)
			f.close()

			cw = int(np.shape(mean_frames)[0] / 2)
			ch = int(np.shape(mean_frames)[1] / 2)

			freqs = modeInfo['mode'][int(ref_mode)]['freqsMhz']

			maxPhaseSteps = int(modeInfo['mode'][int(ref_mode)]['maxPhaseSteps'])

			mean_frames_center = np.zeros([len(freqs),maxPhaseSteps])
			for i in range(len(freqs)):
				mean_frames_center[i,:] = mean_frames[ch, cw, i, :]

			del mean_frames

			plt.figure()
			for i in range(len(freqs)):
				plt.plot(mean_frames_center[i,:], label=str(freqs[i])+"Mhz")
			# plt.plot(mean_frames_center[0,:], label="198Mhz")
			# plt.plot(mean_frames_center[1,:], label="189Mhz")
			# plt.plot(mean_frames_center[2,:], label="54Mhz")
			plt.legend()
			plt.xlabel('Phase')
			plt.ylabel('Amplitude')
			plt.grid()
			plt.savefig(os.path.join(result_dir, 'Correlated_waveform_mode_' + str(ref_mode) + '.png'))
			calibResult['P0']['Correlated_waveform_mode_' + str(ref_mode)]= os.path.join('Results', 'Correlated_waveform_mode_' + str(ref_mode) + '.png')

			fft_mean_frames_center = np.zeros([len(freqs), int(maxPhaseSteps/2)])

			for i in range(len(freqs)):
				fft_mean_frames_center[i, :] = generate_fft_waveform(np.abs(np.fft.fft(mean_frames_center[i,:]))) / maxPhaseSteps
			# fft_mean_frames_center[0, :] = generate_fft_waveform(np.abs(np.fft.fft(mean_frames_center[0,:]))) / 24
			# fft_mean_frames_center[1, :] = generate_fft_waveform(np.abs(np.fft.fft(mean_frames_center[1,:]))) / 24
			# fft_mean_frames_center[2, :] = generate_fft_waveform(np.abs(np.fft.fft(mean_frames_center[2,:]))) / 24

			plt.figure()
			for i in range(len(freqs)):
				plt.plot(fft_mean_frames_center[i,:], label=str(freqs[i])+"Mhz")
			# plt.plot(fft_mean_frames_center[0, :], label="198Mhz")
			# plt.plot(fft_mean_frames_center[1, :], label="189Mhz")
			# plt.plot(fft_mean_frames_center[2, :], label="54Mhz")
			plt.legend()
			plt.yscale('log')
			plt.xlabel('Phase')
			plt.ylabel('Amplitude')
			plt.grid()
			plt.savefig(os.path.join(result_dir, 'fft_Correlated_waveform_mode_' + str(ref_mode) + '.png'))
			calibResult['P0']['fft_Correlated_waveform_mode_' + str(ref_mode)]= os.path.join('Results', 'fft_Correlated_waveform_mode_' + str(ref_mode) + '.png')
			plt.close()

			for i in range(len(freqs)):
				calibResult['P0']['Modulation_efficiency_mode_' + str(ref_mode) + '_'+str(int(freqs[i]))+'Mhz'] = (fft_mean_frames_center[i, 1] / fft_mean_frames_center[i, 0]) / (4/np.pi) * 100
			# calibResult['P0']['Modulation_efficiency_mode_' + str(ref_mode) + '_198Mhz'] = (fft_mean_frames_center[0, 1] / fft_mean_frames_center[0, 0]) / (4/np.pi) * 100
			# calibResult['P0']['Modulation_efficiency_mode_' + str(ref_mode) + '_189Mhz'] = (fft_mean_frames_center[1, 1] / fft_mean_frames_center[1, 0]) / (4/np.pi) * 100
			# calibResult['P0']['Modulation_efficiency_mode_' + str(ref_mode) + '_54Mhz'] = (fft_mean_frames_center[2, 1] / fft_mean_frames_center[2, 0]) / (4/np.pi) * 100


			# dist_range = np.linspace(0, 5, 1000)
			# n_dist = len(dist_range)
			# # freqs = [198e6, 189e6, 54e6]
			# freqs = np.array(modeInfo['mode'][int(ref_mode)]['freqsMhz']) * 1e6
			# n_freqs = len(freqs)

			# cn = np.zeros([n_freqs, maxPhaseSteps])
			# cn = np.fft.fftshift(np.fft.fft(mean_frames_center) / maxPhaseSteps)

			# # Nh = 12
			# # nh = np.arange(-12, 12)
			# Nh = int(maxPhaseSteps/2)
			# nh = np.arange(-int(maxPhaseSteps/2), int(maxPhaseSteps/2))
			# # [t,
			# #  yt] = synthesize_waveform_from_fourier_coefficients(nh, cn[0, :], freqs[0],
			# #                                                      10e9, 0, 3 / freqs[0])

			# V0 = np.zeros([n_freqs, n_dist])
			# V120 = np.zeros([n_freqs, n_dist])
			# V240 = np.zeros([n_freqs, n_dist])

			# for ii, f in enumerate(freqs):

			#     for jj, dist in enumerate(dist_range):
			#         # Generate laser waveforms harmonics
			#         T_cycle = 1 / f
			#         dt = dist * 2 / c / T_cycle * 2 * pi

			#         # Sample correlation
			#         V0[ii, jj] = sample_correlation(nh, cn[ii, :], +dt)
			#         V120[ii, jj] = sample_correlation(nh, cn[ii, :], 2 * pi / 3 + dt)
			#         V240[ii, jj] = sample_correlation(nh, cn[ii, :], 4 * pi / 3 + dt)

			# re = 2 * V0 - V120 - V240
			# im = np.sqrt(3) * (V240 - V120)
			# C = re + 1j * im
			# # # Calculate estimate distance assuming perfect unwrapping
			# # est_dist_0 = c / (4 * pi * freqs[0]) * np.unwrap(np.angle(C[0, :]))
			# # est_dist_1 = c / (4 * pi * freqs[1]) * np.unwrap(np.angle(C[1, :]))
			# # est_dist_2 = c / (4 * pi * freqs[2]) * np.unwrap(np.angle(C[2, :]))

			# # # Apply "P0 calibration" the distance measurement of each frequency
			# # P0_dist = ref_dist / 1000.00
			# # dist_idx_P0 = (np.abs(dist_range - P0_dist)).argmin()
			# # est_dist_0 -= est_dist_0[dist_idx_P0] - P0_dist
			# # est_dist_1 -= est_dist_1[dist_idx_P0] - P0_dist
			# # est_dist_2 -= est_dist_2[dist_idx_P0] - P0_dist

			# # # Weighted sum for final distance calculation
			# # est_dist = est_dist_0 * freqs[0]**2 / np.sum(
			# #     np.array(freqs)**2) + est_dist_1 * freqs[1]**2 / np.sum(
			# #         np.array(freqs)**2) + est_dist_2 * freqs[2]**2 / np.sum(
			# #             np.array(freqs)**2)

			# est_dist_n = np.zeros(n_freqs, dtype=object)
			# # Calculate estimate distance assuming perfect unwrapping
			# for i in range(n_freqs):
			# 	est_dist_n[i] = c / (4 * pi * freqs[i]) * np.unwrap(np.angle(C[i, :]))

			# # Apply "P0 calibration" the distance measurement of each frequency
			# P0_dist = ref_dist / 1000.00
			# dist_idx_P0 = (np.abs(dist_range - P0_dist)).argmin()

			# for i in range(n_freqs):
			# 	est_dist_n[i] -= est_dist_n[i][dist_idx_P0] - P0_dist

			# # Weighted sum for final distance calculation
			# est_dist = np.zeros(np.shape(est_dist_n[0]))
			# for i in range(n_freqs):
			# 	est_dist = est_dist + est_dist_n[i] * freqs[i]**2 / np.sum(np.array(freqs)**2)

			freqMhzArray = np.array(modeInfo['mode'][int(ref_mode)]['freqsMhz'])

			est_dist_err = estimate_distance(mean_frames_center, freqMhzArray, maxPhaseSteps, ref_dist)
			dist_range = np.linspace(0, 5, 1000)

			plt.figure()
			plt.plot(dist_range, est_dist_err * 1000)
			plt.xlabel('Distance [m]')
			plt.ylabel('Depth estimation error [mm]')
			plt.grid()
			plt.savefig(os.path.join(result_dir, 'Estimated_depth_error_mode_' + str(ref_mode) + '.png'))
			calibResult['P0']['Estimated_depth_error_mode_' + str(ref_mode)]= os.path.join('Results', 'Estimated_depth_error_mode_' + str(ref_mode) + '.png')

	return calibResult