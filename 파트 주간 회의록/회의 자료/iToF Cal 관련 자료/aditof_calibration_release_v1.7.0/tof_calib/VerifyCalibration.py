#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#

import numpy as np
import os
import sys
from numpy.lib.function_base import average
import yaml
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 
import math
from datetime import datetime
import subprocess
import pickle
import logging
from matplotlib.transforms import Transform
from scipy import stats
from scipy.signal import savgol_filter
import pandas as pd

# Function to find distance 
def shortest_distance(coord, eq):

	a = eq[0]
	b = eq[1]
	c = eq[2]
	d = eq[3]
	x1 = coord[0]
	y1 = coord[1]
	z1 = coord[2]

	d = abs((a * x1 + b * y1 + c * z1 + d))  
	e = (math.sqrt(a * a + b * b + c * c)) 
	#print("Perpendicular distance is", d/e) 
	return d/e

def fit_plane(XYZ_file, folderName):
	#Read XYZ points from binary file, reshape in numpy array, and remove invalid points
	f = open(XYZ_file, 'rb')
	points_bin = pickle.load(f)
	f.close()

	if(np.size(points_bin) == 1024*1024*2*3):
		total_pixels = 1024*1024
		array_size = 1024
	elif(np.size(points_bin) == 512*512*2*3):
		total_pixels = 512*512
		array_size = 512
	else:
		print("Error: Unexpected number of points in point cloud")
		sys.exit()

	#obs_points = np.reshape(points_bin, (-1,3))
	#BUG FIX XYZ file size from depth compute
	obs_points = np.reshape(points_bin, (2,total_pixels,3))
	obs_points = obs_points[0,:,:]
	obs_points_valid = obs_points[~np.all(obs_points == 0, axis=1)]

	#Replace invalidated pixels with an average pixel value to fill array for rehape purposes
	average_point = np.average(obs_points_valid, axis=0)

	missing_points = (total_pixels) - obs_points_valid.shape[0]
	if missing_points > 20000:
		print("Error: Number of invalid points in point cloud = ", missing_points)
		sys.exit()
	filler_points = np.resize(average_point, (missing_points, *average_point.shape))

	#Replace invalid pixels with average pixel value for plane fitting calculations
	obs_points[np.all(obs_points == 0, axis=1)] = average_point
	obs_points_new = obs_points

	C_x = np.cov(obs_points_new.T)
	eig_vals, eig_vecs = np.linalg.eigh(C_x)

	variance = np.min(eig_vals)
	min_eig_val_index = np.argmin(eig_vals)
	direction_vector = eig_vecs[:, min_eig_val_index].copy()

	tmp = np.linspace(-500,500,10)
	x,y = np.meshgrid(tmp,tmp)

	normal = direction_vector

	#Find average point cloud value from non-zero data
	#new_array = np.where(obs_points!=0,obs_points,np.nan)
	column_means = np.nanmean(obs_points_new, axis=0)
	point = column_means.astype(int)
	# print(point)


	# A plane is a*x+b*y+c*z+d=0
	# [a,b,c] is the normal. Thus, we have to calculate
	# d and we're set
	d = -point.dot(normal)

	# create x,y
	xx, yy = np.meshgrid(range(-600, 600), range(-600, 600))

	# calculate corresponding z
	z = (-normal[0] * xx - normal[1] * yy - d) * 1. /normal[2]

	best_eq = (normal[0], normal[1], normal[2], d)

	# plot the surface
	#plt3d = plt.figure().gca(projection='3d')
	#plt3d.plot_surface(xx, yy, z)

	Zdepth_1d = obs_points_new[:,2]
	Zdepth = np.reshape(Zdepth_1d,(array_size,array_size))
	
	f = open(os.path.join(folderName, "ZDepth.pkl"),"wb")
	pickle.dump(Zdepth,f)
	f.close()

	fig = plt.figure()
	ax  = fig.add_subplot(111, projection='3d')
	ax.plot3D(obs_points_new[:,0], obs_points_new[:,1], obs_points_new[:,2], 'gray')
	ax.plot_surface(xx, yy, z)

	# plt.show()
	plt.savefig(os.path.join(folderName, 'fitted_plane.png'))
	plt.close()

	mse = 0
	count = 0
	for row in obs_points:
		if(row[2]>0):
			dist = shortest_distance(row, best_eq) 
			mse = mse + (dist*dist) 
			count = count + 1

	rmse = mse/count
	rmse = math.sqrt(rmse)
	print("RMSE of fitted plane = ", rmse, "mm")
	print("Calculated distance of fitted plane = ", abs(d), "mm")

	f = open(os.path.join(folderName, 'rmse.pkl'), 'wb')
	pickle.dump(rmse, f)
	f.close()

	return rmse, d

def FOI_check(AB_file, folderName):
	data = pickle.load(open(AB_file,"rb"))

	mean = round(np.mean(data),2)
	std = round(np.std(data),2)
	minmax = round((np.amax(data) - np.amin(data)),2)
	print("AB Mean = ", mean)
	print("AB std dev = ", std)
	print("AB range values = ", minmax)

	max_coord = np.argmax(data)
	print("Max AB pixel index = ", max_coord)

	if(np.size(data) == 1024*1024):
		total_pixels = 1024*1024
		array_size = 1024
	elif(np.size(data) == 512*512):
		total_pixels = 512*512
		array_size = 512
	else:
		print("Error: Unexpected number of points in point cloud")
		sys.exit()

	#array_size = 1024 #pixel array size, assuming square array for now
	center = int(array_size/2)
	max_coord_x = max_coord % array_size
	max_coord_y = max_coord // array_size
	print("Max AB pixel x-coord = ", max_coord_x)
	print("Max AB pixel y-coord = ", max_coord_y)

	#step = 128 #width/height of box, should divide into array size, e.g. 1024/128 = 8 => 8x8 box pattern
	step = int(array_size/8) #width/height of box, should divide into array size, e.g. 1024/128 = 8 => 8x8 box pattern
	boxes = np.zeros(int((len(data)/step))**2)
	box_index = 0
	for y in range(int(step/2), len(data), step):
		for x in range(int(step/2), len(data[0]), step):
			average = center_mean(data, y, x, int(step/2))
			boxes[box_index] = average
			box_index = box_index + 1

	num_boxes = 4
	slices = int(array_size/step)
	brightest = boxes.argsort()[-num_boxes:][::-1]
	xcoord_brightest = ((brightest % slices) * step) + int(step/2)
	ycoord_brightest = ((brightest // slices) * step) + int(step/2)
	brightest_coords = np.append(xcoord_brightest, ycoord_brightest)

	darkest = boxes.argsort()[:num_boxes]
	xcoord_darkest = ((darkest % slices) * step) + int(step/2)
	ycoord_darkest = ((darkest// slices) * step) + int(step/2)

	dark_dist_center_x = abs(xcoord_darkest - array_size/2)
	dark_dist_center_y = abs(ycoord_darkest - array_size/2)
	#Calculate distance from center of array to center of each dark square
	dark_dist_center = np.sqrt(np.square(dark_dist_center_x)+np.square(dark_dist_center_y))

	plt.figure()
	plt.imshow(data, cmap='gray')
	plt.title('AB image')
	plt.colorbar(label='Intensity')
	plt.annotate("Max at " + str(max_coord_x) + ", " + str(max_coord_y), xy=(max_coord_x, max_coord_y), xycoords="data",
					xytext=(0.8, 0.8), textcoords='axes fraction',
					arrowprops=dict(facecolor='black', shrink=0.05, width=0.5, headwidth=5),
					horizontalalignment='right', verticalalignment='top')

	for x in range(0,num_boxes):
		max_label = "Max" + str(x+1)
		plt.annotate(max_label, xy=(xcoord_brightest[x], ycoord_brightest[x]), xycoords="data",
					va="center", ha="center", color='green',
					bbox=dict(facecolor='none', edgecolor='green',boxstyle="round"))

	for x in range(0,num_boxes):
		min_label = "Min" + str(x+1)
		plt.annotate(min_label, xy=(xcoord_darkest[x], ycoord_darkest[x]), xycoords="data",
					va="center", ha="center", color='red',
					bbox=dict(facecolor='none', edgecolor='red',boxstyle="round"))

	#Plot grid of boxes
	for x in range(0,slices-1):
		plt.axhline(y = ((x+1)*step), color = 'r', linewidth=0.5, linestyle = '-')
		plt.axvline(x = ((x+1)*step), color = 'r', linewidth=0.5, linestyle = '-')

	#Plot circle
	if array_size == 1024:
		dark_keepout = 405 #keepout circle radius in pixels
	else:
		dark_keepout = 202.5 #keepout circle radius in pixels
	circle=plt.Circle((center, center), dark_keepout, color='r', lw=5.0, fill=False)
	plt.gca().add_patch(circle)

	#plot rectangle
	#bright_roi = 512 #square size 
	bright_roi = array_size/2 #square size 
	rectangle = plt.Rectangle((center-bright_roi/2,center-bright_roi/2), bright_roi, bright_roi, color='g', lw=5.0, fill=False)
	plt.gca().add_patch(rectangle)

	#plt.show()
	plt.savefig(os.path.join(folderName, 'AB_stats.png'))
	plt.close()

	plt.figure()
	plt.title("Illumination profiles")
	plt.xlabel("Pixels")
	plt.ylabel("grayscale value")
	xhat = savgol_filter(data[center,:], 51, 3) 
	yhat = savgol_filter(data[:,center], 51, 3)
	dhat1 = savgol_filter(data.diagonal(), 51, 3) 
	dhat2 = savgol_filter(np.flipud(data).diagonal(), 51, 3)
	plt.plot(xhat, color='red', label='X axis')
	plt.plot(yhat, color='green', label='Y axis')
	plt.plot(dhat1, color='blue', label='Diagonal1')
	plt.plot(dhat2, color='purple', label='Diagonal2')
	plt.legend()
	#plt.show()
	plt.savefig(os.path.join(folderName, 'AB_profiles.png'))
	plt.close()

	xhat_peak = np.where(xhat == np.amax(xhat))
	yhat_peak = np.where(yhat == np.amax(yhat))
	dhat1_peak = np.where(dhat1 == np.amax(dhat1))
	dhat2_peak = np.where(dhat2 == np.amax(dhat2))
	profile_peaks = np.concatenate((xhat_peak, yhat_peak, dhat1_peak, dhat2_peak), axis=0)

	pixel_low = 100
	# pixel_high = 924
	pixel_high = array_size - 100
	#xhat low/high values
	xhat_p1 = xhat[pixel_low] 
	xhat_p1_ratio = xhat_p1 / np.amax(xhat)
	xhat_p2 = xhat[pixel_high] 
	xhat_p2_ratio = xhat_p2 / np.amax(xhat)

	#yhat low/high values
	yhat_p1 = yhat[pixel_low] 
	yhat_p1_ratio = yhat_p1 / np.amax(yhat)
	yhat_p2 = yhat[pixel_high] 
	yhat_p2_ratio = yhat_p2 / np.amax(yhat)

	#dhat1 low/high values
	dhat1_p1 = dhat1[pixel_low]
	dhat1_p1_ratio = dhat1_p1 / np.amax(dhat1)
	dhat1_p2 = dhat1[pixel_high]
	dhat1_p2_ratio = dhat1_p2 / np.amax(dhat1)

	#dhat2 low/high values
	dhat2_p1 = dhat2[pixel_low]
	dhat2_p1_ratio = dhat2_p1 / np.amax(dhat2)
	dhat2_p2 = dhat2[pixel_high]
	dhat2_p2_ratio = dhat2_p2 / np.amax(dhat2)

	H_V_ratios = [xhat_p1_ratio, xhat_p2_ratio, yhat_p1_ratio, yhat_p2_ratio]
	Diag_ratios = [dhat1_p1_ratio, dhat1_p2_ratio, dhat2_p1_ratio, dhat2_p2_ratio]

	# create the histogram
	#histogram, bin_edges = np.histogram(data, bins=4096, range=(0, 4095))
	histogram, bin_edges = np.histogram(data, bins=4096)
	print("AB Hist sum = ", histogram.sum())

	# configure and draw the histogram figure
	#fig, ax = plt.figure()
	fig, ax = plt.subplots()
	plt.title("Grayscale Histogram")
	plt.xlabel("grayscale value")
	plt.ylabel("pixels")
	#plt.xlim([0.0, 4095.0])  # <- named arguments do not work here

	plt.axvline(x=np.mean(data), ls = "--", color='g', alpha=0.7, label="Mean")
	plt.axvline(x=np.mean(data)-np.std(data), ls = "--", color='b', alpha=0.7, label="SD")
	plt.axvline(x=np.mean(data)+np.std(data), ls = "--", color='b', alpha=0.7)
	plt.axvline(x=np.max(data), ls = "-", color='r', alpha=0.7, label="Min/Max")
	plt.axvline(x=np.min(data), ls = "-", color='r', alpha=0.7)
	plt.legend()

	mean = round(np.mean(data),2)

	plt.text(0.7, 0.67, 'Mean = ' + str(mean), fontsize=10, transform=ax.transAxes)
	plt.text(0.7, 0.6, 'Std = ' + str(std), fontsize=10, transform=ax.transAxes)
	plt.text(0.7, 0.53, 'Range = ' + str(minmax), fontsize=10, transform=ax.transAxes)

	plt.plot(bin_edges[0:-1], histogram)  # <- or here
	#plt.show()
	plt.savefig(os.path.join(folderName, 'AB_hist.png'))
	plt.close()

	return mean, std, max_coord_x, max_coord_y, brightest_coords, dark_dist_center, profile_peaks, H_V_ratios, Diag_ratios


def center_mean(array, row, col, size):

    #print("row = ", row)
    #print("col = ", col)
    #print("data = ", array[row, col])

    # Use min & max to handle edges
    row_min = max(0, row - size)
    row_max = min(len(array), row + size)

    col_min = max(0, col-size)
    col_max = min(len(array[0]), col + size)

    average = np.mean(array[row_min:row_max+1, col_min:col_max+1])

    return average


def FixedDistVerification(config_file, CalibfolderName):
	"""
	Top level function to analyse XYZ depth map (with calibration) 
		Inputs:
		  config_file           - name of configuration (.cfg) file
		  CalibfolderName       - name of calbration folder
		  
		Outputs:
		  None
	"""

	if CalibfolderName == "":
		now = datetime.now()
		folderName = 'Calibration_verification_' + str(now.strftime('%Y%m%d_%H%M%S'))
	else:
		folderName = os.path.join(CalibfolderName, "Calibration_verification")
	
	if not os.path.exists(folderName):
		os.mkdir(folderName)
	else:
		now = datetime.now()
		print('\nALERT: ' + folderName + ' already exists')
		old_folderName = folderName + str(now.strftime('_old_%Y%m%d_%H%M%S'))
		os.rename(folderName, old_folderName)
		print('ALERT: RENAMING OLD FOLDER: ' + folderName + ' as '+ old_folderName +'\n')
		os.mkdir(folderName)

	with open(config_file) as y:
		file = yaml.load(y, Loader=yaml.SafeLoader)

	output_file_csv_report = file['Calibration_verification']['output_file_csv_report']

	try:
		calibration_dir_path = file['calibration_directory']
	except:
		calibration_dir_path = ""

	os.chdir(calibration_dir_path)

	# # pythonEXEPath = os.path.dirname(sys.executable)
	# # print('pythonEXEPath:',pythonEXEPath)
	# # shutil.copyfile(os.path.join("aditofdevicepython", "tofi_processor.obj"), os.path.join(pythonEXEPath, "tofi_processor.obj"))

	with open(os.path.join(CalibfolderName, 'header.yaml')) as y:
		header = yaml.load(y, Loader=yaml.SafeLoader)

	depthJsonPath = header['flashConfigJson']
	if depthJsonPath == '':
		print("\n depth Json not found.\nGenerate compressed ccb first!!\n")
		return False

	if header['compressed_ccb_flashed'] == False:
		print("\n Compressed CCB not flashed to NVM!!\n")
		return False
	else:
		print(depthJsonPath)
		print(os.path.abspath(folderName))
		command = 'python ' +str(calibration_dir_path)+ '/verify/tof_output.py' + " " + '"'+str(depthJsonPath) +'"'+ " " + '"'+str(os.path.abspath(folderName))+'"'
		proc = subprocess.Popen(command, stdin = subprocess.PIPE, stdout = subprocess.PIPE)
		proc.wait()

	if os.path.exists(os.path.join(folderName, 'AB.pkl')):
		AB_mean, AB_std, max_coord_x, max_coord_y, bright_boxes, dark_dist_center, profile_peaks, H_V_ratios, Diag_ratios = FOI_check(os.path.join(folderName, 'AB.pkl'), folderName)

	if os.path.exists(os.path.join(folderName, 'XYZ.pkl')):
		rmse, dist = fit_plane(os.path.join(folderName, 'XYZ.pkl'), folderName)

	# proc = subprocess.Popen('next', stdin = subprocess.PIPE, stdout = subprocess.PIPE)
	# stdout, stderr = proc.communicate('dir c:\\')

	limits_file = file['limits_file']

	if os.path.abspath(limits_file) != limits_file:
		limits_file = os.path.join(calibration_dir_path, limits_file)

	try:
		with open(limits_file) as ly:
			limits = yaml.load(ly, Loader=yaml.SafeLoader)['CalibrationVerification']

	except:
		raise Exception("\n\nWARNING: Incorrect/non-existing limits file\n\n")


	VerCal_dict = dict()
	VerCal_dict['info'] = dict()
	VerCal_dict['Verify_pass'] = True #Initialize to True

	AB_mean_min = limits['AB_mean'][0] 
	AB_mean_max = limits['AB_mean'][1] 
	if(AB_mean_min < AB_mean < AB_mean_max):
		AB_mean_status = "PASSED"
	else:
		AB_mean_status = "FAILED"
		VerCal_dict['Verify_pass'] = False
	VerCal_dict['AB_mean'] = [AB_mean, AB_mean_status, AB_mean_min, AB_mean_max]

	AB_std_min = limits['AB_std'][0] 
	AB_std_max = limits['AB_std'][1] 
	if(AB_std_min < AB_std < AB_std_max):
		AB_std_status = "PASSED"
	else:
		AB_std_status = "FAILED"
		VerCal_dict['Verify_pass'] = False
	VerCal_dict['AB_std'] = [AB_std, AB_std_status, AB_std_min, AB_std_max]

	max_coord_min = limits['max_coord'][0] 
	max_coord_max = limits['max_coord'][1] 
	if(max_coord_min < max_coord_x < max_coord_max) & (max_coord_min < max_coord_y < max_coord_max):
		max_coord_status = "PASSED"
	else:
		max_coord_status = "FAILED"
		VerCal_dict['Verify_pass'] = False
	VerCal_dict['max_coord'] = [(max_coord_x, max_coord_y), max_coord_status, max_coord_min, max_coord_max]
	#VerCal_dict['max_coord_y'] = [max_coord_y, max_coord_status, max_coord_min, max_coord_max]

	bright_box_min = limits['bright_center'][0] 
	bright_box_max = limits['bright_center'][1] 
	if(all((bright_box_min < i < bright_box_max) for i in bright_boxes)):
		bright_box_status = "PASSED"
	else:
		bright_box_status = "FAILED"
		VerCal_dict['Verify_pass'] = False
	VerCal_dict['bright_center'] = [bright_boxes, bright_box_status, bright_box_min, bright_box_max]

	dark_dist_center_min = limits['dark_center'][0] 
	dark_dist_center_max = limits['dark_center'][1] 
	if(all((i > dark_dist_center_min) for i in dark_dist_center)):
		dark_box_status = "PASSED"
	else:
		dark_box_status = "FAILED"
		VerCal_dict['Verify_pass'] = False
	VerCal_dict['dark_center'] = [dark_dist_center, dark_box_status, dark_dist_center_min, dark_dist_center_max]

	profile_peaks_min = limits['profile_peaks'][0] 
	profile_peaks_max = limits['profile_peaks'][1] 
	if(all((profile_peaks_min < i < profile_peaks_max) for i in profile_peaks)):
		profile_peaks_status = "PASSED"
	else:
		profile_peaks_status = "FAILED"
		VerCal_dict['Verify_pass'] = False
	VerCal_dict['profile_peaks'] = [profile_peaks, profile_peaks_status, profile_peaks_min, profile_peaks_max]

	H_V_ratios_min = limits['H_V_ratios'][0] 
	H_V_ratios_max = limits['H_V_ratios'][1] 
	if(all((H_V_ratios_min < i < H_V_ratios_max) for i in H_V_ratios)):
		H_V_ratios_status = "PASSED"
	else:
		H_V_ratios_status = "FAILED"
		VerCal_dict['Verify_pass'] = False
	VerCal_dict['H_V_ratios'] = [H_V_ratios, H_V_ratios_status, H_V_ratios_min, H_V_ratios_max]

	Diag_ratios_min = limits['Diag_ratios'][0] 
	Diag_ratios_max = limits['Diag_ratios'][1] 
	if(all((Diag_ratios_min < i < Diag_ratios_max) for i in Diag_ratios)):
		Diag_ratios_status = "PASSED"
	else:
		Diag_ratios_status = "FAILED"
		VerCal_dict['Verify_pass'] = False
	VerCal_dict['Diag_ratios'] = [Diag_ratios, Diag_ratios_status, Diag_ratios_min, Diag_ratios_max]
	
	rmse_min = limits['rms_error'][0] 
	rmse_max = limits['rms_error'][1] 
	if(rmse_min < rmse < rmse_max):
		RMSE_status = "PASSED"
	else:
		RMSE_status = "FAILED"
		VerCal_dict['Verify_pass'] = False
	VerCal_dict['rms_error'] = [rmse, RMSE_status, rmse_min, rmse_max]

	plane_dist_min = limits['plane_dist'][0] 
	plane_dist_max = limits['plane_dist'][1] 
	if(plane_dist_min < abs(dist) < plane_dist_max): #plane could be negative distance so need to use abs value
		plane_dist_status = "PASSED"
	else:
		plane_dist_status = "FAILED"
		VerCal_dict['Verify_pass'] = False
	VerCal_dict['plane_dist'] = [abs(dist), plane_dist_status, plane_dist_min, plane_dist_max]

	VerCal_dict['info']['Pass'] = VerCal_dict['Verify_pass']

	if VerCal_dict['info']['Pass'] == True:
		device_passed = "PASSED"
		print("\n==================")
		print('CAL VERIFY PASSED')
		print("==================\n")
	else:
		device_passed = "FAILED"
		print("\n==================")
		print('CAL VERIFY FAILED')
		print("==================\n")

	pkl_file = os.path.join(folderName, file['Calibration_verification']['output_file_pkl'])
	f = open(pkl_file + ".pkl", 'wb')
	pickle.dump(VerCal_dict, f)
	f.close()

	# Save results to CSV file 
	Verification_status = dict()
	Verification_status['Pass/Fail'] = [VerCal_dict['Verify_pass'], device_passed]
	for key, value in VerCal_dict.items():
		if isinstance(VerCal_dict[key], (list,tuple, str)):
			if len(VerCal_dict[key]) <= 4:
				Verification_status[key] = value
	df = pd.DataFrame(dict([ (k,pd.Series(v)) for k,v in Verification_status.items() ]))
	df = df.T
	df.columns = ['Value', 'Result', 'Min Limit', 'Max Limit']
	df.to_csv(os.path.join(folderName, output_file_csv_report + '.csv'))
