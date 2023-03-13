import numpy as np
import pickle
import yaml
import os
import P0_functions as P0
import matplotlib.pyplot as plt
import sys
from shutil import copyfile

def showFormat():
	'''
	Displays input format
	'''
	print("\n\n\nFormat:")
	print("python ./tof_calib/P0Util.py <mode> <file_name relative to calibration directory>")
	print("mode = compress, uncompress")
	print("Example: python ./tof_calib/P0Util.py compress ./Walden_001/P0_output/P0Table_mode_5.pkl")
	print("Example: python ./tof_calib/P0Util.py compress ./Walden_001/P0_output")

if len(sys.argv) == 3:
	mode = str(sys.argv[1])
	fname = sys.argv[2]
	proceed = True

else:
	showFormat()
	proceed = False

if proceed:
	P0_calib = P0.P0_calibration()

	if os.path.isfile(fname):
		if mode.upper() == "COMPRESS":
			p0_fname = fname
			comp_p0_fname = os.path.join(os.path.dirname(fname),
										 "compressed_" + os.path.basename(fname))

			P0_calib.compress_P0Table(p0_fname, comp_p0_fname)
			

		elif mode.upper() == "UNCOMPRESS":
			comp_p0_fname = fname
			uncomp_p0_fname = os.path.join(os.path.dirname(fname),
										  "uncompressed_" + os.path.basename(fname)[:-4] + '.pkl')

			P0_calib.uncompress_P0Table(comp_p0_fname, uncomp_p0_fname)

			print("\nNOTE: Uncompressed P0 table will be of float32 format instead of float64\n")

		else:
			print("Invalid mode")
			showFormat()

	elif os.path.isdir(fname):
		P0_outfiles = os.listdir(fname)
		P0Table_modes = np.array([], dtype =np.int8)
		P0Table_files = np.array([], dtype=object)
		P0Table_test_files = np.array([], dtype=object)
		for file in P0_outfiles:
			if file.startswith('P0Table_mode_') and file[-4:]=='.pkl':
				mode = int(file.split('_')[-1][:-4])
				P0Table_modes = np.append(P0Table_modes, mode)
				P0Table_files = np.append(P0Table_files, file)
				P0Table_test_files = np.append(P0Table_test_files, 'test_mode_' + str(mode) + '.pkl')


		print('P0 Tables found:', P0Table_files)

		try:
			for file in P0_outfiles:
				if file.startswith('test_mode_' + str(np.sort(P0Table_modes)[0]) + '_AB_') and file[-4:]=='.pkl':
					P0_calib.CALIBRATION_DISTANCE = int(file.split('_')[-1][:-6])

		except:
			P0_calib.CALIBRATION_DISTANCE = 0

		if 10 in P0Table_modes:
			if os.path.exists(os.path.join(fname, 'test_mode_10.pkl')) == False:
				if os.path.exists(os.path.join(fname, 'test_mode_5.pkl')):
					copyfile(os.path.join(fname, 'test_mode_5.pkl'), os.path.join(fname, 'test_mode_10.pkl'))
				else:
					P0Table_modes = np.delete(P0Table_modes,np.where(a==10)[0][0])


		if 10 in P0Table_modes:
			if os.path.exists(os.path.join(fname, 'mean_frames_Mode_10.pkl')) == False:
				if os.path.exists(os.path.join(fname, 'mean_frames_Mode_5.pkl')):
					copyfile(os.path.join(fname, 'mean_frames_Mode_5.pkl'), os.path.join(fname, 'mean_frames_Mode_10.pkl'))


		if 10 in P0Table_modes:
			if os.path.exists(os.path.join(fname, 'test_mode_10_AB_' + str(P0_calib.CALIBRATION_DISTANCE) + 'mm.pkl')) == False:
				if os.path.exists(os.path.join(fname, 'test_mode_5_AB_' + str(P0_calib.CALIBRATION_DISTANCE) + 'mm.pkl')):
					copyfile(os.path.join(fname, 'test_mode_5_AB_' + str(P0_calib.CALIBRATION_DISTANCE) + 'mm.pkl'), 
								os.path.join(fname, 'test_mode_10_AB_' + str(P0_calib.CALIBRATION_DISTANCE) + 'mm.pkl'))
					copyfile(os.path.join(fname, 'test_mode_5_freqDist_' + str(P0_calib.CALIBRATION_DISTANCE) + 'mm.pkl'), 
								os.path.join(fname, 'test_mode_10_freqDist_' + str(P0_calib.CALIBRATION_DISTANCE) + 'mm.pkl'))



		for i in range(len(P0Table_modes)):
			P0Table_file = os.path.join(fname, P0Table_files[i])
			verify_file = os.path.join(fname, P0Table_test_files[i])

			comp_P0Table_file = os.path.join(fname, 'compressed_P0Table_mode_' + str(P0Table_modes[i]) + '.pkl')
			P0_calib.compress_P0Table(P0Table_file, comp_P0Table_file)

			uncomp_P0Table_file = os.path.join(fname, 'uncompressed_P0Table_mode_'+ str(P0Table_modes[i]) + '_2D_P0' + '.pkl')
			P0_calib.uncompress_P0Table(comp_P0Table_file, uncomp_P0Table_file)

			P0_calib.apply_compressed_P0Table(verify_file, comp_P0Table_file, P0Table_file, P0Table_modes[i])