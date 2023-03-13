#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#

import numpy as np 
import pickle

def GainRaw2Lx5(targetGain, nominalGain):
	"""
	Converts Gain raw format to LX5 format
	Inputs: 
		- targetGain. Per column, per 2 rows, per gain (1024x2x4) inverse gain setting
		- nominalGain. Per gain nominal inverse gain value (4 values).

	Outputs:
	- inv_adc_gain. 13b values to write to registers inv_adc_gain_0,...,inv_adc_gain_3 (1x4)
	- perColGainAdj. 8b data to write to per-column gain memory (1024x2x4).
	- perColClamped. Logical flag indicating that the target gain is outside 
	the per-column inverse gain adjustment range, resulting in poor gain correction (1024x2x4)
	"""


	if np.shape(targetGain)[2] != np.size(nominalGain):
		raise Exception('Number of target gains doesn''t match number of nominal gains')

	perColGainAdj = np.zeros(np.shape(targetGain))
	inv_adc_gain = np.zeros(np.size(nominalGain))

	for gainCnt in range(np.size(nominalGain)-1,-1,-1):
		# step 1: solve for global gain shift
		globalGainShift = np.log2(nominalGain[gainCnt] / 1.2)
		# divide by 1.2 adds some margin to ensure we are setting the nominal gain in a good range: ~0.8x to 1.7x
		globalGainShift_value = np.round(globalGainShift)
		if globalGainShift_value < -1 or globalGainShift_value > 6:
			raise Exception('Nominal gain is out of range')
		globalGainShift_equiv = 2**(-globalGainShift_value)

		# step 2: solve for global gain scale
		globalGainScale = nominalGain[gainCnt] * globalGainShift_equiv
		globalGainScale_value = np.round(512 * globalGainScale) # normalize to use full u10 field
		if globalGainScale_value >= 2**10:
			raise Exception('globalGainShift set incorrectly')
		# globalGainScale_equiv = globalGainScale_value / 512
		
		# step 3: solve for per-column gain scale
		pcGainTarget = targetGain[:,:,gainCnt] * globalGainShift_equiv
		perColGainAdj[:,:,gainCnt] = np.round(512 * pcGainTarget) - globalGainScale_value # normalize to DN of global gain

		# step 4: profit!
		globalOutputShift = np.mod(6-globalGainShift_value,8) # shift to match range defined in ADSD3100
		# globalOutputShift_equiv = 2**(-(3-globalGainShift_value))

		inv_adc_gain[gainCnt] = globalOutputShift* (2**10) + globalGainScale_value


	inv_adc_gain = inv_adc_gain.astype(np.int16)
	nonClampedPerColGainAdj = np.round(perColGainAdj)
	perColGainAdj = perColGainAdj.astype(np.int8)
	clampedGain = (perColGainAdj != nonClampedPerColGainAdj)
 
	return inv_adc_gain, perColGainAdj, clampedGain


def OffsetRaw2Lx5(offset_data, bitsPerPixel='bpp_10'):
	"""
	Converts Offset raw format to LX% format
	Inputs: 
		- offset_data. Per column, per 2 rows, per gain (1024x2x4) offset data
		- bitsPerPixel (optional). Flag set to 'bpp_9' or 'bpp_10', or 9 or 10 for 9
		or 10 bits per pixel. If bpp_9 is selected, offset_data will be 
		multiplied by 2 to match LX5 format. Default is 'bpp_10'

	Outputs:
		- perColOffsetAdj. 8b data to write to per-column gain memory (1024x2x4).
		- perColClamped. Logical flag indicating that the target gain is outside 
		the per-column inverse gain adjustment range, resulting in poor gain correction (1024x2x4)
	"""

	if bitsPerPixel == 'bpp_9':
		offset_data = offset_data * 2 # 9bpp, scale by 2
	else:
		pass # no scaling required

	perColOffsetAdj = np.round(offset_data).astype(np.int8) # convert to int8. Note MATLAB clamps data > 127 or <-128
	# Added round because 39.9 must become 40 not 39
	clampedOffset = (perColOffsetAdj != np.round(offset_data)) # check if any values were clamped

	return perColOffsetAdj, clampedOffset


def GeneratePerColRam(data):
	"""
	Pack 8bit per-column gain or offset data into ADSD3100 RAM format

	Inputs: 
		data - 8 bit array of size (1024,2,4) (columns, rows, gains)

	Outputs: 
		success -  Flag indicating the data was correctly processed
		ram_data - All subsampled modes packed into a single RAM blob, ready
		to write to adsd3100.
		ram_struct (optional) - Structure containing RAM entries for individual 
		subsampling modes.
	"""

	success = True
	ram_data = np.array([])

	nativePerColRes = [1024, 2, 4] # [col, row, gain]

	ColRowGainScale = np.array([[1,    1,   1], # A1D1 # per-column dimension scaling factors relative to native resolution
								[0.5,  0.5, 1], # A1D2
								[0.25, 0.5, 1], # A1D4
								[0.5,  2,   1], # A2D1
								[0.25, 1,   1], # A2D2
								[0.25, 2,   1]],dtype=np.float) # A4D1
	perColDims = nativePerColRes * ColRowGainScale

	RAM = dict(
				A1D1 = np.zeros([int(np.prod(perColDims[0,:])/2)],dtype=np.uint16),
				A1D2 = np.zeros([int(np.prod(perColDims[1,:])/2)],dtype=np.uint16),
				A1D4 = np.zeros([int(np.prod(perColDims[2,:])/2)],dtype=np.uint16),
				A2D1 = np.zeros([int(np.prod(perColDims[3,:])/2)],dtype=np.uint16),
				A2D2 = np.zeros([int(np.prod(perColDims[4,:])/2)],dtype=np.uint16),
				A4D1 = np.zeros([int(np.prod(perColDims[5,:])/2)],dtype=np.uint16),
				)

	data = data.astype(np.int8)
	pixData_tmp = np.transpose(data,[1,0,2]).flatten()
	pixData = np.transpose(np.reshape(pixData_tmp.view(np.uint32),[np.shape(data)[1],np.shape(data)[0]]))
	RAM['A1D1'] = np.transpose(pixData).flatten().view(np.uint16) # 8192b native (all cols even row, all cols odd row)
	RAM['A1D2'] = np.transpose(pixData[::2,0]).flatten().view(np.uint16) # 2048b (even cols even row)
	RAM['A1D4'] = np.transpose(pixData[::4,0]).flatten().view(np.uint16) # 1024b (0,4,8...cols even row)
	# (below) 8192b binned (even cols even row, odd cols even row, odd cols odd row, odd cols odd row)
	RAM['A2D1'] = (np.transpose(np.reshape(np.transpose(pixData),[2,512,2]),[0,2,1]).flatten()).view(np.uint16)
	RAM['A2D2'] = np.transpose(pixData[::4,:]).flatten().view(np.uint16) # 2048b (0,4,8...cols even row, 0,4,8...cols even row)

	# 4096b (0,4,8...cols even row, 1,5,9...cols even row, 0,4,8...cols odd row, 1,5,9...cols odd row)
	cols = np.transpose(np.vstack([np.array(range(0,1024,4)),np.array(range(1,1024,4))])).flatten()
	RAM['A4D1'] = (np.transpose(np.reshape(np.transpose(pixData[cols,:]),[2,256,2]),[0,2,1]).flatten()).view(np.uint16)

	ram_data = np.hstack([RAM['A1D1'], RAM['A1D2'], RAM['A1D4'], RAM['A2D1'], RAM['A2D2'], RAM['A4D1']])

	return success, ram_data, RAM
