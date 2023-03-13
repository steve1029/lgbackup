#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#

import struct
import numpy
import numpy as np
import sys
import pickle
from os import listdir
from os.path import isfile, join
import os
import yaml
import json
import zlib
from datetime import datetime


CAL_BLOCK_ID_ADDRVAL_REGLIST  = 'A'
CAL_BLOCK_ID_DEVICE_CONFIG    = 'C'
CAL_BLOCK_ID_PIXEL_DEFECT     = 'D'
CAL_BLOCK_ID_FOI_MASK         = 'F'
CAL_BLOCK_ID_GEOMETRIC        = 'G'
CAL_BLOCK_ID_ILLUM_PROFILE    = 'I'
CAL_BLOCK_ID_LSDACS           = 'L'
CAL_BLOCK_ID_P0               = 'P'
CAL_BLOCK_ID_REGISTRATION     = 'R'
CAL_BLOCK_ID_SPATIAL_TEMPCOR  = 'S'
CAL_BLOCK_ID_TEMPCORRECTION   = 'T'
CAL_BLOCK_ID_RELATIVE_ILLUM   = 'V'
CAL_BLOCK_ID_COLUMN_DEFECT    = '|'
CAL_BLOCK_ID_GAINCORRECTION   = 0xD7
CAL_BLOCK_ID_HDR              = 0xFF

CAL_HDR_V0_CONFIG_VER_STR_SIZE = 128
CAL_HDR_V0_SERIAL_NUM_STR_SIZE = 32
CAL_HDR_V0_CHIP_UNIQUE_ID_SIZE = 32

# ccb_fname = 'tester.ccb'

def CompileBlocks(binaryCal, binaryCal_dtype, sizeCal, nBlocksCal,CalibfolderName,barcode,chipUID,add_comp_P0, add_uncomp_P0):
	'''
		Function to compile calibration blocks and generate .ccb file

		Inputs:
		  binaryCal             - 2D array of binary of each block
		  binaryCal_dtype       - 2D array of binary data type of each block
		  sizeCal               - 1D array of size in bytes of each block
		  nBlocksCal            - 1D array of number of blocks of each block
		  CalibfolderName       - Name of calibration folder name
		  
		Outputs:
		  None
		
	'''

	# print(binaryCal, binaryCal_dtype, sizeCal, nBlocksCal)

	# Generate Header block
	Header = np.array([], dtype=object)
	Header_dtype = np.array([], dtype=object)

	sizeCalBlocks = np.sum(sizeCal)
	nBlocks = np.sum(nBlocksCal)
	sizeFileHeader = 16

	nBlocks_arr = np.array([nBlocks],dtype=np.uint16)
	Header = np.append(Header, nBlocks_arr)
	Header_dtype = np.append(Header_dtype, np.dtype(np.uint16))

	chip_id_arr = np.array([22833],dtype=np.uint16)
	Header = np.append(Header, chip_id_arr)
	Header_dtype = np.append(Header_dtype, np.dtype(np.uint16))

	nRows_arr = np.array([1024],dtype=np.uint16)
	Header = np.append(Header, nRows_arr)
	Header_dtype = np.append(Header_dtype, np.dtype(np.uint16))

	nCols_arr = np.array([1024],dtype=np.uint16)
	Header = np.append(Header, nCols_arr)
	Header_dtype = np.append(Header_dtype, np.dtype(np.uint16))

	configVersionString = '6109.7'

	if barcode == "-1":
		moduleName = os.path.basename(CalibfolderName)
		if len(moduleName) > 30:
			serialNumber = moduleName[-30:]
			print("\n\nWARNING: Serial number length > 30, truncated to last 30 characters\n\n")
		else:
			serialNumber = moduleName
		print("\nserialNumber in CCB:: " + serialNumber + '\n')
	else:
		serialNumber = barcode

	for i in range(CAL_HDR_V0_CONFIG_VER_STR_SIZE - len(configVersionString)):
		configVersionString = configVersionString + ' '

	configVersionString = list(configVersionString)
	for i in range(len(configVersionString)):
		configVersionString[i] = ord(configVersionString[i])

	for i in range(CAL_HDR_V0_SERIAL_NUM_STR_SIZE - len(serialNumber)):
		serialNumber = serialNumber + ' '

	serialNumber = list(serialNumber)
	for i in range(len(serialNumber)):
		serialNumber[i] = ord(serialNumber[i])

	for i in range(CAL_HDR_V0_CHIP_UNIQUE_ID_SIZE - len(chipUID)):
		chipUID = chipUID + ' '

	chipUID = list(chipUID)
	for i in range(len(chipUID)):
		chipUID[i] = ord(chipUID[i])

	configVersionString_arr = np.array(configVersionString,dtype=np.int8)
	Header = np.append(Header, configVersionString_arr)
	for i in range(CAL_HDR_V0_CONFIG_VER_STR_SIZE):
		Header_dtype = np.append(Header_dtype, np.dtype(np.int8))

	serialNumber_arr = np.array(serialNumber,dtype=np.int8)
	Header = np.append(Header, serialNumber_arr)
	for i in range(CAL_HDR_V0_SERIAL_NUM_STR_SIZE):
		Header_dtype = np.append(Header_dtype, np.dtype(np.int8))

	chipUID_arr = np.array(chipUID,dtype=np.int8)
	Header = np.append(Header, chipUID_arr)
	for i in range(CAL_HDR_V0_CHIP_UNIQUE_ID_SIZE):
		Header_dtype = np.append(Header_dtype, np.dtype(np.int8))

	controlAPIVersion_arr = np.array([67108867],dtype=np.uint32)
	Header = np.append(Header, controlAPIVersion_arr)
	Header_dtype = np.append(Header_dtype, np.dtype(np.uint32))

	firmwareVersion_arr = np.array([17045510],dtype=np.uint32)
	Header = np.append(Header, firmwareVersion_arr)
	Header_dtype = np.append(Header_dtype, np.dtype(np.uint32))

	calibVersionMajor_arr = np.array([6109],dtype=np.uint16)
	Header = np.append(Header, calibVersionMajor_arr)
	Header_dtype = np.append(Header_dtype, np.dtype(np.uint16))

	calibVersionMinor_arr = np.array([12007],dtype=np.uint16)
	Header = np.append(Header, calibVersionMinor_arr)
	Header_dtype = np.append(Header_dtype, np.dtype(np.uint16))

	configurationVersion_arr = np.array([400359431],dtype=np.uint32)
	Header = np.append(Header, configurationVersion_arr)
	Header_dtype = np.append(Header_dtype, np.dtype(np.uint32))

	info = dict()
	now = datetime.now()
	info['CheckSum'] = 0
	info['Year'] = int(now.strftime('%Y'))
	info['Month'] = int(now.strftime('%m'))
	info['Day'] = int(now.strftime('%d'))
	info['Hour'] = int(now.strftime('%H'))
	info['Minute'] = int(now.strftime('%M'))
	info['Second'] = int(now.strftime('%S'))
	info['Pass'] = 1

	Header_info = dict()
	Header_info['BlockID'] = CAL_BLOCK_ID_HDR
	Header_info['BlockVersion'] = 3
	Header_info['BlockCalibVer'] = 0
	Header_info['BlockSize'] = 216
	Header_info['CheckSum'] = 0
	Header_info['CalibrationYear'] = info['Year']
	Header_info['CalibrationMonth'] = info['Month']
	Header_info['CalibrationDay'] = info['Day']
	Header_info['CalibrationHour'] = info['Hour']
	Header_info['CalibrationMinute'] = info['Minute']
	Header_info['CalibrationSecond'] = info['Second']
	Header_info['CalibrationPass'] = 1

	Header, Header_dtype, Header_size, Header_nBlock = AddCalBlockInfo(Header_info, Header, Header_dtype, 216, 0)

	# Generate File_Header

	File_Header = np.array([],dtype=object)
	File_Header_dtype = np.array([],dtype=object)

	calibFileVersion_arr = np.array([771],dtype=np.uint16)
	File_Header = np.append(File_Header, calibFileVersion_arr)
	File_Header_dtype = np.append(File_Header_dtype, np.dtype(np.uint16))

	calibVersion_arr = np.array([769],dtype=np.uint16)
	File_Header = np.append(File_Header, calibVersion_arr)
	File_Header_dtype = np.append(File_Header_dtype, np.dtype(np.uint16))

	sizeOfCalibHeader_arr = np.array([240],dtype=np.uint16)
	File_Header = np.append(File_Header, sizeOfCalibHeader_arr)
	File_Header_dtype = np.append(File_Header_dtype, np.dtype(np.uint16))

	rsvd_arr = np.array([0],dtype=np.uint8)
	File_Header = np.append(File_Header, rsvd_arr)
	File_Header_dtype = np.append(File_Header_dtype, np.dtype(np.uint8))

	sizeOfCalibFileHeader_arr = np.array([16],dtype=np.uint8)
	File_Header = np.append(File_Header, sizeOfCalibFileHeader_arr)
	File_Header_dtype = np.append(File_Header_dtype, np.dtype(np.uint8))

	sizeOfCalibrationFile_arr = np.array([sizeCalBlocks + Header_size + 16],dtype=np.uint32)
	File_Header = np.append(File_Header, sizeOfCalibrationFile_arr)
	File_Header_dtype = np.append(File_Header_dtype, np.dtype(np.uint32))

	rsvdSize_arr = np.array([0],dtype=np.uint32)
	File_Header = np.append(File_Header, rsvdSize_arr)
	File_Header_dtype = np.append(File_Header_dtype, np.dtype(np.uint32))

	if add_comp_P0:
		ccb_P0_compression = "_compressed"
	else:
		ccb_P0_compression = ""

	ccb_file_name = os.path.join(CalibfolderName, os.path.basename(CalibfolderName)+ccb_P0_compression+'.ccb')

	if os.path.exists(ccb_file_name):
		now = datetime.now()
		print('\nALERT: ' + ccb_file_name + ' already exists')
		print(os.path.join(CalibfolderName, str(now.strftime('CCB_old_%Y%m%d_%H%M%S.ccb'))))
		try:
			os.rename(ccb_file_name, os.path.join(CalibfolderName, str(now.strftime('CCB_old_%Y%m%d_%H%M%S.ccb'))))
			print('ALERT: RENAMING OLD FOLDER: ' + ccb_file_name + ' as '+ os.path.join(CalibfolderName, str(now.strftime('CCB_old_%Y%m%d_%H%M%S.ccb'))) +'\n')
		except:
			os.remove(ccb_file_name)

	write_file = open(ccb_file_name, "wb")
	write_file.close()
	write_file = open(ccb_file_name, "ab")

	## Create the ccb file with checksum = 0

	for i in range(len(File_Header)):
		tmp = np.array([File_Header[i]], dtype=File_Header_dtype[i])
		tmp.tofile(write_file, format='%d')
		#print(File_Header_dtype[i])

	for i in range(len(Header)):
		tmp = np.array([Header[i]], dtype=Header_dtype[i])
		tmp.tofile(write_file, format='%d')

	# f = open(CalibfolderName + '/binaryCal.pkl', 'wb')
	# pickle.dump(binaryCal, f)
	# f.close()

	# f = open(CalibfolderName + '/binaryCal_dtype.pkl', 'wb')
	# pickle.dump(binaryCal_dtype, f)
	# f.close()

	for i in range(len(binaryCal)):
		tmp = np.array([binaryCal[i]], dtype=binaryCal_dtype[i])
		tmp.tofile(write_file, format='%d')


	write_file.close()

	# Header[4] is the checksum

	## Open the ccb file to generate a crc32 checksum

	f = open(ccb_file_name, "rb")
	s = f.read()

	checksum_crc32 = zlib.crc32(s)
	Header[4] = np.uint64(checksum_crc32)

	f.close()

	## Deleting the ccb file

	os.remove(ccb_file_name)


	## Create the ccb with updated checksum in header


	print('WRITING TO FILE: ' + ccb_file_name)

	write_file = open(ccb_file_name, "wb")
	write_file.close()
	write_file = open(ccb_file_name, "ab")

	## Create the ccb file with checksum = 0

	for i in range(len(File_Header)):
		tmp = np.array([File_Header[i]], dtype=File_Header_dtype[i])
		tmp.tofile(write_file, format='%d')
		#print(File_Header_dtype[i])

	for i in range(len(Header)):
		tmp = np.array([Header[i]], dtype=Header_dtype[i])
		tmp.tofile(write_file, format='%d')

	# f = open(CalibfolderName + '/binaryCal.pkl', 'wb')
	# pickle.dump(binaryCal, f)
	# f.close()

	# f = open(CalibfolderName + '/binaryCal_dtype.pkl', 'wb')
	# pickle.dump(binaryCal_dtype, f)
	# f.close()

	for i in range(len(binaryCal)):
		tmp = np.array([binaryCal[i]], dtype=binaryCal_dtype[i])
		tmp.tofile(write_file, format='%d')


	write_file.close()



	return os.path.join(CalibfolderName, ccb_file_name)


def AddCalBlockInfo(blockInfo, binary, binary_dtype, sizeBlock, nBlocks):
	'''
		Function to generate calibration information

		Inputs:
		  blockInfo             - Refer to dictionary mentioned below
		  binary                - Binary data of the calibration block
		  binary_dtype          - Binary data data-type of the calibration block
		  sizeBlock             - Size of block in bytes
		  nBlocks               - Number of blocks
		  
		Outputs:
		  CalBlock              - Updated binary data of the calibration block
		  CalBlock_dtype        - Updated binary data of the calibration block
		  sizeBlock             - Updated size of block in bytes
		  nBlocks               - Updated number of blocks
		
	'''


	# blockInfo -> Dictionary containing:
	# uint8_t BlockID
	# uint8_t BlockVersion
	# uint16_t BlockCalibVer
	# uint32_t BlockSize
	# uint64_t CheckSum
	# uint16_t CalibrationYear
	# uint8_t CalibrationMonth
	# uint8_t CalibrationDay
	# uint8_t CalibrationHour
	# uint8_t CalibrationMinute
	# uint8_t CalibrationSecond
	# uint8_t CalibrationPass

	sizeInfo = 24

	CalBlock = np.array([], dtype=object)
	CalBlock_dtype = np.array([], dtype=object)

	BlockID_arr = np.array([blockInfo['BlockID']],dtype=np.int8)
	CalBlock = np.append(CalBlock, BlockID_arr)
	CalBlock_dtype = np.append(CalBlock_dtype, np.dtype(np.int8))

	BlockFormatVer_arr = np.array([blockInfo['BlockVersion']],dtype=np.uint8)
	CalBlock = np.append(CalBlock, BlockFormatVer_arr)
	CalBlock_dtype = np.append(CalBlock_dtype, np.dtype(np.uint8))

	BlockCalibVer_arr = np.array([blockInfo['BlockCalibVer']],dtype=np.uint16)
	CalBlock = np.append(CalBlock, BlockCalibVer_arr)
	CalBlock_dtype = np.append(CalBlock_dtype, np.dtype(np.uint16))

	BlockSize_arr = np.array([blockInfo['BlockSize'] + sizeInfo],dtype=np.uint32)
	CalBlock = np.append(CalBlock, BlockSize_arr)
	CalBlock_dtype = np.append(CalBlock_dtype, np.dtype(np.uint32))

	Checksum_arr = np.array([blockInfo['CheckSum']],dtype=np.uint64)
	CalBlock = np.append(CalBlock, Checksum_arr)
	CalBlock_dtype = np.append(CalBlock_dtype, np.dtype(np.uint64))

	CalibrationYear_arr = np.array([blockInfo['CalibrationYear']],dtype=np.uint16)
	CalBlock = np.append(CalBlock, CalibrationYear_arr)
	CalBlock_dtype = np.append(CalBlock_dtype, np.dtype(np.uint16))

	CalibrationMonth_arr = np.array([blockInfo['CalibrationMonth']],dtype=np.uint8)
	CalBlock = np.append(CalBlock, CalibrationMonth_arr)
	CalBlock_dtype = np.append(CalBlock_dtype, np.dtype(np.uint8))

	CalibrationDay_arr = np.array([blockInfo['CalibrationDay']],dtype=np.uint8)
	CalBlock = np.append(CalBlock, CalibrationDay_arr)
	CalBlock_dtype = np.append(CalBlock_dtype, np.dtype(np.uint8))

	CalibrationHour_arr = np.array([blockInfo['CalibrationHour']],dtype=np.uint8)
	CalBlock = np.append(CalBlock, CalibrationHour_arr)
	CalBlock_dtype = np.append(CalBlock_dtype, np.dtype(np.uint8))

	CalibrationMinute_arr = np.array([blockInfo['CalibrationMinute']],dtype=np.uint8)
	CalBlock = np.append(CalBlock, CalibrationMinute_arr)
	CalBlock_dtype = np.append(CalBlock_dtype, np.dtype(np.uint8))

	CalibrationSecond_arr = np.array([blockInfo['CalibrationSecond']],dtype=np.uint8)
	CalBlock = np.append(CalBlock, CalibrationSecond_arr)
	CalBlock_dtype = np.append(CalBlock_dtype, np.dtype(np.uint8))

	CalibrationPass_arr = np.array([blockInfo['CalibrationPass']],dtype=np.uint8)
	CalBlock = np.append(CalBlock, CalibrationPass_arr)
	CalBlock_dtype = np.append(CalBlock_dtype, np.dtype(np.uint8))

	CalBlock = np.append(CalBlock, binary)
	CalBlock_dtype = np.append(CalBlock_dtype, binary_dtype)

	return CalBlock, CalBlock_dtype, blockInfo['BlockSize'] + sizeInfo, nBlocks


def GenerateGainOffsetBlock(GO_cal_dict):
	'''
		Function to generate Gain and Offset calibration block

		Inputs:
		  GO_cal_dict           - Dictionary containing gain and offset data
		  
		Outputs:
		  GOBlock               - Binary data of the calibration block
		  GOBlock_dtype         - Binary data of the calibration block
		  GO_size               - Size of block in bytes
		  GO_nBlock             - Number of blocks
		
	'''
	print("FOUND: Gain and Offset Calibration")

	GOBlock = np.array([], dtype=object)
	GOBlock_dtype = np.array([], dtype=object)

	GO_size = 0
	GO_nBlock = 1

	GainCalConfigVersion_arr = np.array([GO_cal_dict['gainCalConfigVersion']],dtype=np.uint16)
	GOBlock = np.append(GOBlock, GainCalConfigVersion_arr)
	GOBlock_dtype = np.append(GOBlock_dtype, np.dtype(np.uint16))
	GO_size = GO_size + 2

	SubsamplingDataPresent_arr = np.array([63],dtype=np.uint16)
	GOBlock = np.append(GOBlock, SubsamplingDataPresent_arr)
	GOBlock_dtype = np.append(GOBlock_dtype, np.dtype(np.uint16))
	GO_size = GO_size + 2

	ADC_IRamp_arr = np.array([GO_cal_dict['ADC']['iramp']],dtype=np.uint16)
	GOBlock = np.append(GOBlock, ADC_IRamp_arr)
	GOBlock_dtype = np.append(GOBlock_dtype, np.dtype(np.uint16))
	GO_size = GO_size + 2

	ADC_UpdnoOffset_arr = np.array([GO_cal_dict['ADC']['updnoffset']],dtype=np.uint16)
	GOBlock = np.append(GOBlock, ADC_UpdnoOffset_arr)
	GOBlock_dtype = np.append(GOBlock_dtype, np.dtype(np.uint16))
	GO_size = GO_size + 2

	gainComparator_keys = list(GO_cal_dict['gainComparator'].keys())

	Vref1Dac_arr = np.array([GO_cal_dict['gainComparator']['Vref1DAC']],dtype=np.uint8)
	GOBlock = np.append(GOBlock, Vref1Dac_arr)
	GOBlock_dtype = np.append(GOBlock_dtype, np.dtype(np.uint8))

	Vref1Set_arr = np.array([GO_cal_dict['gainComparator']['Vref1Set']],dtype=np.uint8)
	GOBlock = np.append(GOBlock, Vref1Set_arr)
	GOBlock_dtype = np.append(GOBlock_dtype, np.dtype(np.uint8))

	Vref2Dac_arr = np.array([GO_cal_dict['gainComparator']['Vref2DAC']],dtype=np.uint8)
	GOBlock = np.append(GOBlock, Vref2Dac_arr)
	GOBlock_dtype = np.append(GOBlock_dtype, np.dtype(np.uint8))

	Vref2Set_arr = np.array([GO_cal_dict['gainComparator']['Vref2Set']],dtype=np.uint8)
	GOBlock = np.append(GOBlock, Vref2Set_arr)
	GOBlock_dtype = np.append(GOBlock_dtype, np.dtype(np.uint8))

	Vref3Dac_arr = np.array([GO_cal_dict['gainComparator']['Vref3DAC']],dtype=np.uint8)
	GOBlock = np.append(GOBlock, Vref3Dac_arr)
	GOBlock_dtype = np.append(GOBlock_dtype, np.dtype(np.uint8))

	Vref3Set_arr = np.array([GO_cal_dict['gainComparator']['Vref3Set']],dtype=np.uint8)
	GOBlock = np.append(GOBlock, Vref3Set_arr)
	GOBlock_dtype = np.append(GOBlock_dtype, np.dtype(np.uint8))

	if 'VCMDAC' in gainComparator_keys:
		VCMDac_arr = np.array([GO_cal_dict['gainComparator']['VCMDAC']],dtype=np.uint8)
		GOBlock = np.append(GOBlock, VCMDac_arr)
		GOBlock_dtype = np.append(GOBlock_dtype, np.dtype(np.uint8))
	else:
		VCMDac_arr = np.array([0],dtype=np.uint8)
		GOBlock = np.append(GOBlock, VCMDac_arr)
		GOBlock_dtype = np.append(GOBlock_dtype, np.dtype(np.uint8))

	VCMSet_arr = np.array([GO_cal_dict['gainComparator']['VCMSet']],dtype=np.uint8)
	GOBlock = np.append(GOBlock, VCMSet_arr)
	GOBlock_dtype = np.append(GOBlock_dtype, np.dtype(np.uint8))
	GO_size = GO_size + 8

	InverseGlobalADCGain_arr = np.array([GO_cal_dict['inverseGlobalADCGain']],dtype=np.uint16)
	GOBlock = np.append(GOBlock, InverseGlobalADCGain_arr)
	for i in range(len(GO_cal_dict['inverseGlobalADCGain'])):
		GOBlock_dtype = np.append(GOBlock_dtype, np.dtype(np.uint16))

	GO_size = GO_size + len(GO_cal_dict['inverseGlobalADCGain'])*2

	PerColGainAdjustment_arr = np.array([GO_cal_dict['ramDataGain']],dtype=np.uint16)
	GOBlock = np.append(GOBlock, PerColGainAdjustment_arr)
	for i in range(len(GO_cal_dict['ramDataGain'])):
		GOBlock_dtype = np.append(GOBlock_dtype, np.dtype(np.uint16))

	GO_size = GO_size + len(GO_cal_dict['ramDataGain'])*2

	PerColOffsetAdjustment_arr = np.array([GO_cal_dict['ramDataOffset']],dtype=np.uint16)
	GOBlock = np.append(GOBlock, PerColOffsetAdjustment_arr)
	for i in range(len(GO_cal_dict['ramDataOffset'])):
		GOBlock_dtype = np.append(GOBlock_dtype, np.dtype(np.uint16))

	GO_size = GO_size + len(GO_cal_dict['ramDataOffset'])*2

	GO_info = dict()
	GO_info['BlockID'] = CAL_BLOCK_ID_GAINCORRECTION
	GO_info['BlockVersion'] = 1
	GO_info['BlockCalibVer'] = 12007
	GO_info['BlockSize'] = GO_size
	# do not consider info size. It will be added later
	try:
		info = GO_cal_dict['info']
	except:
		info = dict()
		now = datetime.now()
		info['CheckSum'] = 0
		info['Year'] = int(now.strftime('%Y'))
		info['Month'] = int(now.strftime('%m'))
		info['Day'] = int(now.strftime('%d'))
		info['Hour'] = int(now.strftime('%H'))
		info['Minute'] = int(now.strftime('%M'))
		info['Second'] = int(now.strftime('%S'))
		info['Pass'] = 1
	
	GO_info['CheckSum'] = info['CheckSum']
	GO_info['CalibrationYear'] = info['Year']
	GO_info['CalibrationMonth'] = info['Month']
	GO_info['CalibrationDay'] = info['Day']
	GO_info['CalibrationHour'] = info['Hour']
	GO_info['CalibrationMinute'] = info['Minute']
	GO_info['CalibrationSecond'] = info['Second']
	GO_info['CalibrationPass'] = info['Pass']

	GOBlock, GOBlock_dtype, GO_size, GO_nBlock = AddCalBlockInfo(GO_info, GOBlock, GOBlock_dtype, GO_size, GO_nBlock)

	return GOBlock, GOBlock_dtype, GO_size, GO_nBlock


def GenerateAddressValuePairBlock(AddressValueRegisterList):
	'''
		Function to generate AddressValueRegisterList block

		Inputs:
		  AddressValueRegisterList  - Dictionary containing AddressValueRegisterList data
		  
		Outputs:
		  AVRLBlock             - Binary data of the calibration block
		  AVRLBlock_dtype       - Binary data of the calibration block
		  AVRL_size             - Size of block in bytes
		  AVRL_nBlock           - Number of blocks
		
	'''
	print("FOUND: Address Value Register List")

	try:
		AddressValueRegisterList = AddressValueRegisterList['AddressValueRegisterList']
	except:
		pass

	AVRLBlock = np.array([], dtype=object)
	AVRLBlock_dtype = np.array([], dtype=object)

	nTuples_arr = np.array([len(AddressValueRegisterList)],dtype=np.uint16)
	AVRLBlock = np.append(AVRLBlock, nTuples_arr)
	AVRLBlock_dtype = np.append(AVRLBlock_dtype, np.dtype(np.uint16))

	rsvd_arr = np.array([0, 0, 0],dtype=np.uint16)
	AVRLBlock = np.append(AVRLBlock, rsvd_arr)
	for i in range(3):
		AVRLBlock_dtype = np.append(AVRLBlock_dtype, np.dtype(np.uint16))

	tuples_arr = np.array(np.array(AddressValueRegisterList).flatten(),dtype=np.uint16)
	AVRLBlock = np.append(AVRLBlock, tuples_arr)
	for i in range(len(np.array(AddressValueRegisterList).flatten())):
		AVRLBlock_dtype = np.append(AVRLBlock_dtype, np.dtype(np.uint16))

	AVRL_size = len(np.array(AddressValueRegisterList).flatten())*2 + 8
	AVRL_nBlock = 1


	AVRL_info = dict()
	AVRL_info['BlockID'] = ord(CAL_BLOCK_ID_ADDRVAL_REGLIST)
	AVRL_info['BlockVersion'] = 1
	AVRL_info['BlockCalibVer'] = 12007
	AVRL_info['BlockSize'] = AVRL_size
	# do not consider info size. It will be added later

	try:
		info = AddressValueRegisterList['info']
	except:
		info = dict()
		now = datetime.now()
		info['CheckSum'] = 0
		info['Year'] = int(now.strftime('%Y'))
		info['Month'] = int(now.strftime('%m'))
		info['Day'] = int(now.strftime('%d'))
		info['Hour'] = int(now.strftime('%H'))
		info['Minute'] = int(now.strftime('%M'))
		info['Second'] = int(now.strftime('%S'))
		info['Pass'] = 1

	AVRL_info['CheckSum'] = info['CheckSum']
	AVRL_info['CalibrationYear'] = info['Year']
	AVRL_info['CalibrationMonth'] = info['Month']
	AVRL_info['CalibrationDay'] = info['Day']
	AVRL_info['CalibrationHour'] = info['Hour']
	AVRL_info['CalibrationMinute'] = info['Minute']
	AVRL_info['CalibrationSecond'] = info['Second']
	AVRL_info['CalibrationPass'] = info['Pass']

	AVRLBlock, AVRLBlock_dtype, AVRL_size, AVRL_nBlock = AddCalBlockInfo(AVRL_info, AVRLBlock, AVRLBlock_dtype, AVRL_size, AVRL_nBlock)

	return AVRLBlock, AVRLBlock_dtype, AVRL_size, AVRL_nBlock


def GenerateP0TableBlock(P0Table, mode_info_arr, compressed_P0):
	'''
		Function to generate P0 calibration block

		Inputs:
		  P0Table               - P0 Table dims [h x c x nFreqs]
		  mode_info_arr         - Dictionary containing information specific to
		                          camera mode
		  compressed_P0         - boolean to denote if the P0 table input is
		                          compressed
		  
		Outputs:
		  P0_Block              - Binary data of the calibration block
		  P0_Block_dtype        - Binary data of the calibration block
		  P0_size               - Size of block in bytes
		  P0_nBlock             - Number of blocks
		
	'''

	P0_Block = np.array([], dtype=object)
	P0_Block_dtype = np.array([], dtype=object)

	P0_size = 0
	P0_nBlock = 1

	mode_arr = np.array([mode_info_arr[0]],dtype=np.uint8)
	P0_Block = np.append(P0_Block, mode_arr)
	P0_Block_dtype = np.append(P0_Block_dtype, np.dtype(np.uint8))
	P0_size = P0_size + 1

	freqIndex_arr = np.array([mode_info_arr[1]],dtype=np.uint8)
	P0_Block = np.append(P0_Block, freqIndex_arr)
	P0_Block_dtype = np.append(P0_Block_dtype, np.dtype(np.uint8))
	P0_size = P0_size + 1

	freq_arr = np.array([mode_info_arr[2]],dtype=np.uint16)
	P0_Block = np.append(P0_Block, freq_arr)
	P0_Block_dtype = np.append(P0_Block_dtype, np.dtype(np.uint16))
	P0_size = P0_size + 2

	IntegrationTime_arr = np.array([mode_info_arr[3]],dtype=np.uint16)
	P0_Block = np.append(P0_Block, IntegrationTime_arr)
	P0_Block_dtype = np.append(P0_Block_dtype, np.dtype(np.uint16))
	P0_size = P0_size + 2

	nRowsCoeffs_arr = np.array([mode_info_arr[4]],dtype=np.uint8)
	P0_Block = np.append(P0_Block, nRowsCoeffs_arr)
	P0_Block_dtype = np.append(P0_Block_dtype, np.dtype(np.uint8))
	P0_size = P0_size + 1

	nColCoeffs_arr = np.array([mode_info_arr[5]],dtype=np.uint8)
	P0_Block = np.append(P0_Block, nColCoeffs_arr)
	P0_Block_dtype = np.append(P0_Block_dtype, np.dtype(np.uint8))
	P0_size = P0_size + 1

	AnalogBinRows_arr = np.array([mode_info_arr[6]],dtype=np.uint8)
	P0_Block = np.append(P0_Block, AnalogBinRows_arr)
	P0_Block_dtype = np.append(P0_Block_dtype, np.dtype(np.uint8))
	P0_size = P0_size + 1

	AnalogBinCols_arr = np.array([mode_info_arr[7]],dtype=np.uint8)
	P0_Block = np.append(P0_Block, AnalogBinCols_arr)
	P0_Block_dtype = np.append(P0_Block_dtype, np.dtype(np.uint8))
	P0_size = P0_size + 1

	DigitalBinRows_arr = np.array([mode_info_arr[8]],dtype=np.uint8)
	P0_Block = np.append(P0_Block, DigitalBinRows_arr)
	P0_Block_dtype = np.append(P0_Block_dtype, np.dtype(np.uint8))
	P0_size = P0_size + 1

	DigitalBinCols_arr = np.array([mode_info_arr[9]],dtype=np.uint8)
	P0_Block = np.append(P0_Block, DigitalBinCols_arr)
	P0_Block_dtype = np.append(P0_Block_dtype, np.dtype(np.uint8))
	P0_size = P0_size + 1

	SubSamplingRows_arr = np.array([mode_info_arr[10]],dtype=np.uint8)
	P0_Block = np.append(P0_Block, SubSamplingRows_arr)
	P0_Block_dtype = np.append(P0_Block_dtype, np.dtype(np.uint8))
	P0_size = P0_size + 1

	SubSamplingCols_arr = np.array([mode_info_arr[11]],dtype=np.uint8)
	P0_Block = np.append(P0_Block, SubSamplingCols_arr)
	P0_Block_dtype = np.append(P0_Block_dtype, np.dtype(np.uint8))
	P0_size = P0_size + 1

	SoftwareBinRows_arr = np.array([mode_info_arr[12]],dtype=np.uint8)
	P0_Block = np.append(P0_Block, SoftwareBinRows_arr)
	P0_Block_dtype = np.append(P0_Block_dtype, np.dtype(np.uint8))
	P0_size = P0_size + 1

	SoftwareBinCols_arr = np.array([mode_info_arr[13]],dtype=np.uint8)
	P0_Block = np.append(P0_Block, SoftwareBinCols_arr)
	P0_Block_dtype = np.append(P0_Block_dtype, np.dtype(np.uint8))
	P0_size = P0_size + 1

	nRows_arr = np.array([mode_info_arr[14]],dtype=np.uint16)
	P0_Block = np.append(P0_Block, nRows_arr)
	P0_Block_dtype = np.append(P0_Block_dtype, np.dtype(np.uint16))
	P0_size = P0_size + 2

	nCols_arr = np.array([mode_info_arr[15]],dtype=np.uint16)
	P0_Block = np.append(P0_Block, nCols_arr)
	P0_Block_dtype = np.append(P0_Block_dtype, np.dtype(np.uint16))
	P0_size = P0_size + 2

	OffsetRows_arr = np.array([mode_info_arr[16]],dtype=np.uint16)
	P0_Block = np.append(P0_Block, OffsetRows_arr)
	P0_Block_dtype = np.append(P0_Block_dtype, np.dtype(np.uint16))
	P0_size = P0_size + 2

	OffsetCols_arr = np.array([mode_info_arr[17]],dtype=np.uint16)
	P0_Block = np.append(P0_Block, OffsetCols_arr)
	P0_Block_dtype = np.append(P0_Block_dtype, np.dtype(np.uint16))
	P0_size = P0_size + 2

	if compressed_P0:
		print("FOUND: Compressed P0 Table Mode:", mode_info_arr[0], "Freq:", mode_info_arr[2])
		P0RowCoefs = np.array(P0Table['P0RowCoefs'], dtype=np.float32)
		P0ColCoefs = np.array(P0Table['P0ColCoefs'], dtype=np.float32)
		P0ColCoefs = P0ColCoefs.reshape(len(P0ColCoefs),1)
		P0ColCoefs = np.append(np.zeros([len(P0ColCoefs),2], dtype=np.float32), P0ColCoefs, axis = 1)
		P0ColCoefs = P0ColCoefs.flatten()

		P0Coeffs = np.append(P0RowCoefs, P0ColCoefs)

		P0Coeffs_arr = np.array([P0Coeffs],dtype=np.float32)
		P0_Block = np.append(P0_Block, P0Coeffs_arr)

		tmp = [np.dtype(np.float32)] * len(P0Coeffs)
		P0_Block_dtype = np.append(P0_Block_dtype, tmp)
		# for i in range(len(P0Coeffs)):
		# 	P0_Block_dtype = np.append(P0_Block_dtype, np.dtype(np.float32))

		P0_size = P0_size + len(P0Coeffs) * 4

	else:
		print("FOUND: Uncompressed P0 Table Mode:", mode_info_arr[0], "Freq:", mode_info_arr[2])
		try:
			P0Table = P0Table['P0Table']
		except:
			pass

		P0Coeffs_arr = np.array(P0Table, dtype=np.float32).flatten()
		P0_Block = list(P0_Block)
		P0_Block.append(list(P0Coeffs_arr))
		P0_Block = np.array(P0_Block, dtype=object)
		P0_Block_dtype = np.append(P0_Block_dtype, [np.dtype(np.float32)])

		# for i in range(len(P0Coeffs_arr)):
		# 	P0_Block_dtype = np.append(P0_Block_dtype, np.dtype(np.float32))
		P0_size = P0_size + len(P0Coeffs_arr) * 4



	P0_info = dict()
	P0_info['BlockID'] = ord(CAL_BLOCK_ID_P0)
	if compressed_P0:
		P0_info['BlockVersion'] = 4
	else:
		P0_info['BlockVersion'] = 255
	P0_info['BlockCalibVer'] = 7005
	P0_info['BlockSize'] = P0_size
	# do not consider info size. It will be added later

	try:
		info = P0Table['info']
	except:
		info = dict()
		now = datetime.now()
		info['CheckSum'] = 0
		info['Year'] = int(now.strftime('%Y'))
		info['Month'] = int(now.strftime('%m'))
		info['Day'] = int(now.strftime('%d'))
		info['Hour'] = int(now.strftime('%H'))
		info['Minute'] = int(now.strftime('%M'))
		info['Second'] = int(now.strftime('%S'))
		info['Pass'] = 1

	P0_info['CheckSum'] = info['CheckSum']
	P0_info['CalibrationYear'] = info['Year']
	P0_info['CalibrationMonth'] = info['Month']
	P0_info['CalibrationDay'] = info['Day']
	P0_info['CalibrationHour'] = info['Hour']
	P0_info['CalibrationMinute'] = info['Minute']
	P0_info['CalibrationSecond'] = info['Second']
	P0_info['CalibrationPass'] = info['Pass']

	P0_Block, P0_Block_dtype, P0_size, P0_nBlock = AddCalBlockInfo(P0_info, P0_Block, P0_Block_dtype, P0_size, P0_nBlock)

	return P0_Block, P0_Block_dtype, P0_size, P0_nBlock


def GenerateGeometricBlock(GeoCal):
	'''
		Function to generate Geometric calibration block

		Inputs:
		  GeoCal                - Dictionary containing camera instrinsics data
		  
		Outputs:
		  GeoBlock              - Binary data of the calibration block
		  GeoBlock_dtype        - Binary data of the calibration block
		  Geo_size              - Size of block in bytes
		  Geo_nBlock            - Number of blocks
		
	'''
	print("FOUND: Geometric Calibration Parameters")

	GeoBlock = np.array([], dtype=object)
	GeoBlock_dtype = np.array([], dtype=object)

	Geo_size = 0
	Geo_nBlock = 1

	nRows_arr = np.array([1024],dtype=np.uint16)
	GeoBlock = np.append(GeoBlock, nRows_arr)
	GeoBlock_dtype = np.append(GeoBlock_dtype, np.dtype(np.uint16))
	Geo_size = Geo_size + 2

	nCols_arr = np.array([1024],dtype=np.uint16)
	GeoBlock = np.append(GeoBlock, nCols_arr)
	GeoBlock_dtype = np.append(GeoBlock_dtype, np.dtype(np.uint16))
	Geo_size = Geo_size + 2

	offsetRows_arr = np.array([0],dtype=np.uint16)
	GeoBlock = np.append(GeoBlock, offsetRows_arr)
	GeoBlock_dtype = np.append(GeoBlock_dtype, np.dtype(np.uint16))
	Geo_size = Geo_size + 2

	offsetCols_arr = np.array([0],dtype=np.uint16)
	GeoBlock = np.append(GeoBlock, offsetCols_arr)
	GeoBlock_dtype = np.append(GeoBlock_dtype, np.dtype(np.uint16))
	Geo_size = Geo_size + 2

	mode_arr = np.array([3],dtype=np.uint8)
	GeoBlock = np.append(GeoBlock, mode_arr)
	GeoBlock_dtype = np.append(GeoBlock_dtype, np.dtype(np.uint8))
	Geo_size = Geo_size + 1

	pixelBinning_arr = np.array([0],dtype=np.uint8)
	GeoBlock = np.append(GeoBlock, pixelBinning_arr)
	GeoBlock_dtype = np.append(GeoBlock_dtype, np.dtype(np.uint8))
	Geo_size = Geo_size + 1

	rsvd_arr = np.array([0],dtype=np.uint16)
	GeoBlock = np.append(GeoBlock, rsvd_arr)
	GeoBlock_dtype = np.append(GeoBlock_dtype, np.dtype(np.uint16))
	Geo_size = Geo_size + 2

	CameraModel_arr = np.array([2],dtype=np.uint32)
	GeoBlock = np.append(GeoBlock, CameraModel_arr)
	GeoBlock_dtype = np.append(GeoBlock_dtype, np.dtype(np.uint32))
	Geo_size = Geo_size + 4

	Fc1_arr = np.array([GeoCal['Fc1'][0]],dtype=np.float32)
	GeoBlock = np.append(GeoBlock, Fc1_arr)
	GeoBlock_dtype = np.append(GeoBlock_dtype, np.dtype(np.float32))
	Geo_size = Geo_size + 4

	Fc2_arr = np.array([GeoCal['Fc2'][0]],dtype=np.float32)
	GeoBlock = np.append(GeoBlock, Fc2_arr)
	GeoBlock_dtype = np.append(GeoBlock_dtype, np.dtype(np.float32))
	Geo_size = Geo_size + 4

	cc1_arr = np.array([GeoCal['cc1'][0]],dtype=np.float32)
	GeoBlock = np.append(GeoBlock, cc1_arr)
	GeoBlock_dtype = np.append(GeoBlock_dtype, np.dtype(np.float32))
	Geo_size = Geo_size + 4

	cc2_arr = np.array([GeoCal['cc2'][0]],dtype=np.float32)
	GeoBlock = np.append(GeoBlock, cc2_arr)
	GeoBlock_dtype = np.append(GeoBlock_dtype, np.dtype(np.float32))
	Geo_size = Geo_size + 4

	Cx_arr = np.array([GeoCal['Cx']],dtype=np.float32)
	GeoBlock = np.append(GeoBlock, Cx_arr)
	GeoBlock_dtype = np.append(GeoBlock_dtype, np.dtype(np.float32))
	Geo_size = Geo_size + 4

	Cy_arr = np.array([GeoCal['Cy']],dtype=np.float32)
	GeoBlock = np.append(GeoBlock, Cy_arr)
	GeoBlock_dtype = np.append(GeoBlock_dtype, np.dtype(np.float32))
	Geo_size = Geo_size + 4

	Kc1_arr = np.array([GeoCal['Kc1']],dtype=np.float32)
	GeoBlock = np.append(GeoBlock, Kc1_arr)
	GeoBlock_dtype = np.append(GeoBlock_dtype, np.dtype(np.float32))
	Geo_size = Geo_size + 4

	Kc2_arr = np.array([GeoCal['Kc2']],dtype=np.float32)
	GeoBlock = np.append(GeoBlock, Kc2_arr)
	GeoBlock_dtype = np.append(GeoBlock_dtype, np.dtype(np.float32))
	Geo_size = Geo_size + 4

	Kc3_arr = np.array([GeoCal['Kc3']],dtype=np.float32)
	GeoBlock = np.append(GeoBlock, Kc3_arr)
	GeoBlock_dtype = np.append(GeoBlock_dtype, np.dtype(np.float32))
	Geo_size = Geo_size + 4

	Kc4_arr = np.array([GeoCal['Kc4']],dtype=np.float32)
	GeoBlock = np.append(GeoBlock, Kc4_arr)
	GeoBlock_dtype = np.append(GeoBlock_dtype, np.dtype(np.float32))
	Geo_size = Geo_size + 4

	Kc5_arr = np.array([GeoCal['Kc5']],dtype=np.float32)
	GeoBlock = np.append(GeoBlock, Kc5_arr)
	GeoBlock_dtype = np.append(GeoBlock_dtype, np.dtype(np.float32))
	Geo_size = Geo_size + 4

	Kc6_arr = np.array([GeoCal['Kc6']],dtype=np.float32)
	GeoBlock = np.append(GeoBlock, Kc6_arr)
	GeoBlock_dtype = np.append(GeoBlock_dtype, np.dtype(np.float32))
	Geo_size = Geo_size + 4

	Tx_arr = np.array([GeoCal['Tx']],dtype=np.float32)
	GeoBlock = np.append(GeoBlock, Tx_arr)
	GeoBlock_dtype = np.append(GeoBlock_dtype, np.dtype(np.float32))
	Geo_size = Geo_size + 4

	Ty_arr = np.array([GeoCal['Ty']],dtype=np.float32)
	GeoBlock = np.append(GeoBlock, Ty_arr)
	GeoBlock_dtype = np.append(GeoBlock_dtype, np.dtype(np.float32))
	Geo_size = Geo_size + 4


	Geo_info = dict()
	Geo_info['BlockID'] = ord(CAL_BLOCK_ID_GEOMETRIC)
	Geo_info['BlockVersion'] = 3
	Geo_info['BlockCalibVer'] = 12007
	Geo_info['BlockSize'] = Geo_size
	# do not consider info size. It will be added later

	try:
		info = GeoCal['info']
	except:
		info = dict()
		now = datetime.now()
		info['CheckSum'] = 0
		info['Year'] = int(now.strftime('%Y'))
		info['Month'] = int(now.strftime('%m'))
		info['Day'] = int(now.strftime('%d'))
		info['Hour'] = int(now.strftime('%H'))
		info['Minute'] = int(now.strftime('%M'))
		info['Second'] = int(now.strftime('%S'))
		info['Pass'] = 1

	Geo_info['CheckSum'] = info['CheckSum']
	Geo_info['CalibrationYear'] = info['Year']
	Geo_info['CalibrationMonth'] = info['Month']
	Geo_info['CalibrationDay'] = info['Day']
	Geo_info['CalibrationHour'] = info['Hour']
	Geo_info['CalibrationMinute'] = info['Minute']
	Geo_info['CalibrationSecond'] = info['Second']
	Geo_info['CalibrationPass'] = info['Pass']


	GeoBlock, GeoBlock_dtype, Geo_size, Geo_nBlock = AddCalBlockInfo(Geo_info, GeoBlock, GeoBlock_dtype, Geo_size, Geo_nBlock)

	return GeoBlock, GeoBlock_dtype, Geo_size, Geo_nBlock


def GenerateLSDACBlock(LSDACSetting):
	'''
		Function to generate LSDAC block

		Inputs:
		  LSDACSetting          - Dictionary containing LSDAC data
		  
		Outputs:
		  LSDACBlock            - Binary data of the calibration block
		  LSDACBlock_dtype      - Binary data of the calibration block
		  LSDAC_size            - Size of block in bytes
		  LSDAC_nBlock          - Number of blocks
		
	'''
	print("FOUND: LSDAC Settings")

	Endaddress_LSDAC = 0x1D21

	LSDACBlock = np.array([], dtype=object)
	LSDACBlock_dtype = np.array([], dtype=object)

	LSDAC_size = 0
	LSDAC_nBlock = 1

	StartAddress_arr = np.array([Endaddress_LSDAC - 4*(len(LSDACSetting['value'])-1)],dtype=np.uint16)
	LSDACBlock = np.append(LSDACBlock, StartAddress_arr)
	LSDACBlock_dtype = np.append(LSDACBlock_dtype, np.dtype(np.uint16))
	LSDAC_size = LSDAC_size + 2

	nWrites_arr = np.array([len(LSDACSetting['value'])],dtype=np.uint16)
	LSDACBlock = np.append(LSDACBlock, nWrites_arr)
	LSDACBlock_dtype = np.append(LSDACBlock_dtype, np.dtype(np.uint16))
	LSDAC_size = LSDAC_size + 2

	ChecksumLSB_arr = np.array([LSDACSetting['checkSumLSB']],dtype=np.uint16)
	LSDACBlock = np.append(LSDACBlock, ChecksumLSB_arr)
	LSDACBlock_dtype = np.append(LSDACBlock_dtype, np.dtype(np.uint16))
	LSDAC_size = LSDAC_size + 2

	ChecksumMSB_arr = np.array([LSDACSetting['checkSumMSB']],dtype=np.uint16)
	LSDACBlock = np.append(LSDACBlock, ChecksumMSB_arr)
	LSDACBlock_dtype = np.append(LSDACBlock_dtype, np.dtype(np.uint16))
	LSDAC_size = LSDAC_size + 2

	Settings_arr = np.array(LSDACSetting['value'],dtype=np.uint16)
	LSDACBlock = np.append(LSDACBlock, Settings_arr)

	tmp = [np.dtype(np.uint16)] * len(Settings_arr)
	LSDACBlock_dtype = np.append(LSDACBlock_dtype, tmp)
	LSDAC_size = LSDAC_size + 2*len(Settings_arr)

	rsvd_arr = np.array([0],dtype=np.uint16)
	LSDACBlock = np.append(LSDACBlock, rsvd_arr)
	LSDACBlock_dtype = np.append(LSDACBlock_dtype, np.dtype(np.uint16))
	LSDAC_size = LSDAC_size + 2

	# print(len(Settings_arr))

	LSDAC_info = dict()
	LSDAC_info['BlockID'] = ord(CAL_BLOCK_ID_LSDACS)
	LSDAC_info['BlockVersion'] = 1
	LSDAC_info['BlockCalibVer'] = 12007
	LSDAC_info['BlockSize'] = LSDAC_size
	# do not consider info size. It will be added later

	try:
		info = LSDACSetting['info']
	except:
		info = dict()
		now = datetime.now()
		info['CheckSum'] = 0
		info['Year'] = int(now.strftime('%Y'))
		info['Month'] = int(now.strftime('%m'))
		info['Day'] = int(now.strftime('%d'))
		info['Hour'] = int(now.strftime('%H'))
		info['Minute'] = int(now.strftime('%M'))
		info['Second'] = int(now.strftime('%S'))
		info['Pass'] = 1

	LSDAC_info['CheckSum'] = info['CheckSum']
	LSDAC_info['CalibrationYear'] = info['Year']
	LSDAC_info['CalibrationMonth'] = info['Month']
	LSDAC_info['CalibrationDay'] = info['Day']
	LSDAC_info['CalibrationHour'] = info['Hour']
	LSDAC_info['CalibrationMinute'] = info['Minute']
	LSDAC_info['CalibrationSecond'] = info['Second']
	LSDAC_info['CalibrationPass'] = info['Pass']

	LSDACBlock, LSDACBlock_dtype, LSDAC_size, LSDAC_nBlock = AddCalBlockInfo(LSDAC_info, LSDACBlock, LSDACBlock_dtype, LSDAC_size, LSDAC_nBlock)

	return LSDACBlock, LSDACBlock_dtype, LSDAC_size, LSDAC_nBlock


def GenerateCalBlock(calDir, CalibfolderName, config_yaml, add_comp_P0, add_uncomp_P0):
	'''
		Function to sort and format data to generate individual
		calibration blocks

		Inputs:
		  calDir                - Calibration directory
		  CalibfolderName       - Name of calibration folder (unique name)
		  config_yaml           - Configuration yaml file
		  
		Outputs:
		  binaryCal             - Binary data of all the calibration blocks
		  binaryCal_dtype       - Binary data of all the calibration blocks
		  sizeCal               - Size of blocks in bytes
		  nBlocksCal            - Number of blocks
		
	'''
	binaryCal = np.array([], dtype=object)
	binaryCal_dtype = np.array([], dtype=object)
	sizeCal = np.array([])
	nBlocksCal = np.array([])

	with open(config_yaml) as y:
		yaml_file = yaml.load(y, Loader=yaml.SafeLoader)

	AVRL_dir = yaml_file['AddressValueRegisterList']['output_directory']
	AVRL_file = str(yaml_file['AddressValueRegisterList']['output_file_pkl'])
	
	GO_dir = yaml_file['GainOffsetCalibration']['output_directory']
	GO_file = str(yaml_file['GainOffsetCalibration']['output_file_pkl'])

	P0_dir = yaml_file['P0Calibration']['output_directory']
	P0_file_comp = str(yaml_file['P0Calibration']['output_file_compressed_pkl'])
	P0_file_uncomp = str(yaml_file['P0Calibration']['output_file_uncompressed_pkl'])

	Geo_dir = yaml_file['GeometricCalibration']['output_directory']
	Geo_file = str(yaml_file['GeometricCalibration']['output_file_pkl'])

	LSDAC_dir = yaml_file['LSDACSetting']['output_directory']
	LSDAC_file = yaml_file['LSDACSetting']['output_file_pkl']

	# print(calDir)

	if AVRL_dir in calDir:
		mypath = os.path.join(CalibfolderName, AVRL_dir)
		calFiles = [f for f in listdir(mypath) if isfile(join(mypath, f))]
		if AVRL_file + '.pkl' in calFiles:
			f = open(os.path.join(mypath, AVRL_dir + ".pkl"), "rb")
			AddressValueRegisterList = pickle.load(f)
			f.close()
			AVRL, AVRL_dtype, sizeAVRL, nBlocksAVRL = GenerateAddressValuePairBlock(AddressValueRegisterList)
			binaryCal = np.append(binaryCal, AVRL)
			binaryCal_dtype = np.append(binaryCal_dtype, AVRL_dtype)
			sizeCal = np.append(sizeCal, sizeAVRL)
			nBlocksCal = np.append(nBlocksCal, nBlocksAVRL)

	if GO_dir in calDir:
		mypath = os.path.join(CalibfolderName, GO_dir)
		calFiles = [f for f in listdir(mypath) if isfile(join(mypath, f))]
		if GO_file + '.pkl' in calFiles:
			f = open(os.path.join(mypath, GO_file + '.pkl'), "rb")
			GO_cal_dict = pickle.load(f)
			f.close()
			GO, GO_dtype, sizeGO, nBlocksGO = GenerateGainOffsetBlock(GO_cal_dict)
			binaryCal = np.append(binaryCal, GO)
			binaryCal_dtype = np.append(binaryCal_dtype, GO_dtype)
			sizeCal = np.append(sizeCal, sizeGO)
			nBlocksCal = np.append(nBlocksCal, nBlocksGO)

	if P0_dir in calDir:
		mypath = os.path.join(CalibfolderName, P0_dir)
		# Array (x axis)
		#			   mode: 
		#         freqIndex: 
		#              freq: 
		#   integrationTime: 
		#         nRowCoefs: 
		#         nColCoefs: 
		#     analogBinRows: 
		#     analogBinCols: 
		#    digitalBinRows: 
		#    digitalBinCols: 
		#   subsamplingRows: 
		#   subsamplingCols: 
		#   softwareBinRows: 
		#   softwareBinCols: 
		#             nRows: 
		#             nCols: 
		#        offsetRows: 
		#        offsetCols: 
		calFiles = [f for f in listdir(mypath) if isfile(join(mypath, f))]
		mode_info_arr = np.array([[0,1,14200,0,1,3,2,2,1,1,1,1,1,1,512,512,0,0],  # 0
								  [0,2,17750,0,1,3,2,2,1,1,1,1,1,1,512,512,0,0],  # 0
								  [1,1,19800,0,1,3,2,1,1,2,1,1,1,1,288,320,90,96],  # 1
								  [1,2,18900,0,1,3,2,1,1,2,1,1,1,1,288,320,90,96],  # 1
								  [1,3,5400,0,1,3,2,1,1,2,1,1,1,1,288,320,90,96],  # 1
								  [4,1,19800,0,1,3,1,1,1,1,1,1,1,1,576,640,180,192],  # 4
								  [4,2,18900,0,1,3,1,1,1,1,1,1,1,1,576,640,180,192],  # 4
								  [4,3,5400,0,1,3,1,1,1,1,1,1,1,1,576,640,180,192],  # 4
								  [5,1,19800,0,1,3,1,1,1,1,1,1,1,1,1024,1024,0,0],  # 5
								  [5,2,18900,0,1,3,1,1,1,1,1,1,1,1,1024,1024,0,0],  # 5
								  [5,3,5400,0,1,3,1,1,1,1,1,1,1,1,1024,1024,0,0],  # 5
								  [7,1,19800,0,1,3,2,2,1,1,1,1,1,1,512,512,0,0],  # 7
								  [7,2,18900,0,1,3,2,2,1,1,1,1,1,1,512,512,0,0],  # 7
								  [7,3,5400,0,1,3,2,2,1,1,1,1,1,1,512,512,0,0],  # 7
								  [10,1,19800,0,1,3,1,1,1,1,1,1,1,1,1024,1024,0,0],  # 10
								  [10,2,18900,0,1,3,1,1,1,1,1,1,1,1,1024,1024,0,0],  # 10
								  [10,3,5400,0,1,3,1,1,1,1,1,1,1,1,1024,1024,0,0]])  # 10
		
		if (P0_file_comp + '0.pkl' in calFiles) and add_comp_P0:
			f = open(os.path.join(mypath, P0_file_comp+"0.pkl"), "rb")
			P0Table_comp_mode_0 = pickle.load(f)
			f.close()
			dict_keys = list(P0Table_comp_mode_0.keys())
			for i in range(2):
				mode_offset = 0
				compressed_P0 = True
				P0_0, P0_0_dtype, sizeP0_0, nBlocksP0_0 = GenerateP0TableBlock(P0Table_comp_mode_0[dict_keys[i]], 
																				mode_info_arr[mode_offset + i],
																				compressed_P0)
				binaryCal = np.append(binaryCal, P0_0)
				binaryCal_dtype = np.append(binaryCal_dtype, P0_0_dtype)
				sizeCal = np.append(sizeCal, sizeP0_0)
				nBlocksCal = np.append(nBlocksCal, nBlocksP0_0)
		
		elif (P0_file_uncomp+'0.pkl' in calFiles) and add_uncomp_P0:
			f = open(os.path.join(mypath, P0_file_uncomp+"0.pkl"), "rb")
			P0Table_mode_0 = pickle.load(f)
			f.close()
			try:
				P0Table_mode_0 = P0Table_mode_0['P0Table']
			except:
				pass

			for i in range(2):
				mode_offset = 0
				compressed_P0 = False
				P0_0, P0_0_dtype, sizeP0_0, nBlocksP0_0 = GenerateP0TableBlock(P0Table_mode_0[:,:,i], 
																				mode_info_arr[mode_offset + i],
																				compressed_P0)
				binaryCal = np.append(binaryCal, P0_0)
				binaryCal_dtype = np.append(binaryCal_dtype, P0_0_dtype)
				sizeCal = np.append(sizeCal, sizeP0_0)
				nBlocksCal = np.append(nBlocksCal, nBlocksP0_0)
		else:
			pass

		if (P0_file_comp + '1.pkl' in calFiles) and add_comp_P0:
			f = open(os.path.join(mypath, P0_file_comp+"1.pkl"), "rb")
			P0Table_comp_mode_1 = pickle.load(f)
			f.close()
			dict_keys = list(P0Table_comp_mode_1.keys())
			for i in range(3):
				mode_offset = 2
				compressed_P0 = True
				P0_1, P0_1_dtype, sizeP0_1, nBlocksP0_1 = GenerateP0TableBlock(P0Table_comp_mode_1[dict_keys[i]], 
																				mode_info_arr[mode_offset + i],
																				compressed_P0)
				binaryCal = np.append(binaryCal, P0_1)
				binaryCal_dtype = np.append(binaryCal_dtype, P0_1_dtype)
				sizeCal = np.append(sizeCal, sizeP0_1)
				nBlocksCal = np.append(nBlocksCal, nBlocksP0_1)
		
		elif (P0_file_uncomp+'1.pkl' in calFiles) and add_uncomp_P0:
			f = open(os.path.join(mypath, P0_file_uncomp+"1.pkl"), "rb")
			P0Table_mode_1 = pickle.load(f)
			f.close()
			try:
				P0Table_mode_1 = P0Table_mode_1['P0Table']
			except:
				pass

			for i in range(3):
				mode_offset = 2
				compressed_P0 = False
				P0_1, P0_1_dtype, sizeP0_1, nBlocksP0_1 = GenerateP0TableBlock(P0Table_mode_1[:,:,i], 
																				mode_info_arr[mode_offset + i],
																				compressed_P0)
				binaryCal = np.append(binaryCal, P0_1)
				binaryCal_dtype = np.append(binaryCal_dtype, P0_1_dtype)
				sizeCal = np.append(sizeCal, sizeP0_1)
				nBlocksCal = np.append(nBlocksCal, nBlocksP0_1)
		else:
			pass


		if (P0_file_comp + '4.pkl' in calFiles) and add_comp_P0:
			f = open(os.path.join(mypath, P0_file_comp+"4.pkl"), "rb")
			P0Table_comp_mode_4 = pickle.load(f)
			f.close()
			dict_keys = list(P0Table_comp_mode_4.keys())
			for i in range(3):
				mode_offset = 5
				compressed_P0 = True
				P0_4, P0_4_dtype, sizeP0_4, nBlocksP0_4 = GenerateP0TableBlock(P0Table_comp_mode_4[dict_keys[i]], 
																				mode_info_arr[mode_offset + i],
																				compressed_P0)
				binaryCal = np.append(binaryCal, P0_4)
				binaryCal_dtype = np.append(binaryCal_dtype, P0_4_dtype)
				sizeCal = np.append(sizeCal, sizeP0_4)
				nBlocksCal = np.append(nBlocksCal, nBlocksP0_4)
		
		elif (P0_file_uncomp+'4.pkl' in calFiles) and add_uncomp_P0:
			f = open(os.path.join(mypath, P0_file_uncomp+"4.pkl"), "rb")
			P0Table_mode_4 = pickle.load(f)
			f.close()
			try:
				P0Table_mode_4 = P0Table_mode_4['P0Table']
			except:
				pass
			for i in range(3):
				mode_offset = 5
				compressed_P0 = False
				P0_4, P0_4_dtype, sizeP0_4, nBlocksP0_4 = GenerateP0TableBlock(P0Table_mode_4[:,:,i], 
																				mode_info_arr[mode_offset + i],
																				compressed_P0)
				binaryCal = np.append(binaryCal, P0_4)
				binaryCal_dtype = np.append(binaryCal_dtype, P0_4_dtype)
				sizeCal = np.append(sizeCal, sizeP0_4)
				nBlocksCal = np.append(nBlocksCal, nBlocksP0_4)
		else:
			pass


		if (P0_file_comp+'5.pkl' in calFiles) and add_comp_P0:
			f = open(os.path.join(mypath, P0_file_comp+"5.pkl"), "rb")
			P0Table_comp_mode_5 = pickle.load(f)
			f.close()
			dict_keys = list(P0Table_comp_mode_5.keys())
			for i in range(3):
				mode_offset = 8
				compressed_P0 = True
				P0_5, P0_5_dtype, sizeP0_5, nBlocksP0_5 = GenerateP0TableBlock(P0Table_comp_mode_5[dict_keys[i]], 
																				mode_info_arr[mode_offset + i],
																				compressed_P0)
				binaryCal = np.append(binaryCal, P0_5)
				binaryCal_dtype = np.append(binaryCal_dtype, P0_5_dtype)
				sizeCal = np.append(sizeCal, sizeP0_5)
				nBlocksCal = np.append(nBlocksCal, nBlocksP0_5)
		
		elif (P0_file_uncomp+'5.pkl' in calFiles) and add_uncomp_P0:
			f = open(os.path.join(mypath, P0_file_uncomp+"5.pkl"), "rb")
			P0Table_mode_5 = pickle.load(f)
			f.close()
			try:
				P0Table_mode_5 = P0Table_mode_5['P0Table']
			except:
				pass
			for i in range(3):
				mode_offset = 8
				compressed_P0 = False
				P0_5, P0_5_dtype, sizeP0_5, nBlocksP0_5 = GenerateP0TableBlock(P0Table_mode_5[:,:,i], 
																				mode_info_arr[mode_offset + i],
																				compressed_P0)
				binaryCal = np.append(binaryCal, P0_5)
				binaryCal_dtype = np.append(binaryCal_dtype, P0_5_dtype)
				sizeCal = np.append(sizeCal, sizeP0_5)
				nBlocksCal = np.append(nBlocksCal, nBlocksP0_5)
		else:
			pass


		if (P0_file_comp+'7.pkl' in calFiles) and add_comp_P0:
			f = open(os.path.join(mypath, P0_file_comp+"7.pkl"), "rb")
			P0Table_comp_mode_7 = pickle.load(f)
			f.close()
			dict_keys = list(P0Table_comp_mode_7.keys())
			for i in range(3):
				mode_offset = 11
				compressed_P0 = True
				P0_7, P0_7_dtype, sizeP0_7, nBlocksP0_7 = GenerateP0TableBlock(P0Table_comp_mode_7[dict_keys[i]], 
																				mode_info_arr[mode_offset + i],
																				compressed_P0)
				binaryCal = np.append(binaryCal, P0_7)
				binaryCal_dtype = np.append(binaryCal_dtype, P0_7_dtype)
				sizeCal = np.append(sizeCal, sizeP0_7)
				nBlocksCal = np.append(nBlocksCal, nBlocksP0_7)
		
		elif (P0_file_uncomp+'7.pkl' in calFiles) and add_uncomp_P0:
			f = open(os.path.join(mypath, P0_file_uncomp+"7.pkl"), "rb")
			P0Table_mode_7 = pickle.load(f)
			f.close()
			try:
				P0Table_mode_7 = P0Table_mode_7['P0Table']
			except:
				pass
			for i in range(3):
				mode_offset = 11
				compressed_P0 = False
				P0_7, P0_7_dtype, sizeP0_7, nBlocksP0_7 = GenerateP0TableBlock(P0Table_mode_7[:,:,i], 
																				mode_info_arr[mode_offset + i],
																				compressed_P0)
				binaryCal = np.append(binaryCal, P0_7)
				binaryCal_dtype = np.append(binaryCal_dtype, P0_7_dtype)
				sizeCal = np.append(sizeCal, sizeP0_7)
				nBlocksCal = np.append(nBlocksCal, nBlocksP0_7)

		else:
			pass


		if (P0_file_comp+'10.pkl' in calFiles) and add_comp_P0:
			f = open(os.path.join(mypath,P0_file_comp+"10.pkl"), "rb")
			P0Table_comp_mode_10 = pickle.load(f)
			f.close()
			dict_keys = list(P0Table_comp_mode_10.keys())
			for i in range(3):
				mode_offset = 14
				compressed_P0 = True
				P0_10, P0_10_dtype, sizeP0_10, nBlocksP0_10 = GenerateP0TableBlock(P0Table_comp_mode_10[dict_keys[i]], 
																					mode_info_arr[mode_offset + i],
																					compressed_P0)
				binaryCal = np.append(binaryCal, P0_10)
				binaryCal_dtype = np.append(binaryCal_dtype, P0_10_dtype)
				sizeCal = np.append(sizeCal, sizeP0_10)
				nBlocksCal = np.append(nBlocksCal, nBlocksP0_10)
		
		elif (P0_file_uncomp+'10.pkl' in calFiles) and add_uncomp_P0:
			f = open(os.path.join(mypath,P0_file_uncomp+"10.pkl"), "rb")
			P0Table_mode_10 = pickle.load(f)
			f.close()
			try:
				P0Table_mode_10 = P0Table_mode_10['P0Table']
			except:
				pass
			for i in range(3):
				mode_offset = 14
				compressed_P0 = False
				P0_10, P0_10_dtype, sizeP0_10, nBlocksP0_10 = GenerateP0TableBlock(P0Table_mode_10[:,:,i], 
																					mode_info_arr[mode_offset + i],
																					compressed_P0)
				binaryCal = np.append(binaryCal, P0_10)
				binaryCal_dtype = np.append(binaryCal_dtype, P0_10_dtype)
				sizeCal = np.append(sizeCal, sizeP0_10)
				nBlocksCal = np.append(nBlocksCal, nBlocksP0_10)

		else:
			pass



	if Geo_dir in calDir:
		mypath = os.path.join(CalibfolderName, Geo_dir)
		calFiles = [f for f in listdir(mypath) if isfile(join(mypath, f))]
		if Geo_file+'.pkl' in calFiles:
			f = open(os.path.join(mypath, Geo_file+".pkl"), "rb")
			GeoCal = pickle.load(f)
			f.close()
			Geo, Geo_dtype, sizeGeo, nBlocksGeo = GenerateGeometricBlock(GeoCal)
			binaryCal = np.append(binaryCal, Geo)
			binaryCal_dtype = np.append(binaryCal_dtype, Geo_dtype)
			sizeCal = np.append(sizeCal, sizeGeo)
			nBlocksCal = np.append(nBlocksCal, nBlocksGeo)

	if LSDAC_dir in calDir:
		mypath = os.path.join(CalibfolderName, LSDAC_dir)
		calFiles = [f for f in listdir(mypath) if isfile(join(mypath, f))]
		if LSDAC_file+'.pkl' in calFiles:
			f = open(os.path.join(mypath, LSDAC_file+".pkl"), "rb")
			LSDACSetting = pickle.load(f)
			f.close()
			LSDAC, LSDAC_dtype, sizeLSDAC, nBlocksLSDAC = GenerateLSDACBlock(LSDACSetting)
			binaryCal = np.append(binaryCal, LSDAC)
			binaryCal_dtype = np.append(binaryCal_dtype, LSDAC_dtype)
			sizeCal = np.append(sizeCal, sizeLSDAC)
			nBlocksCal = np.append(nBlocksCal, nBlocksLSDAC)


	return binaryCal, binaryCal_dtype, sizeCal, nBlocksCal


def GenerateDepthFlashJson(directory, sensorFW, comp_ccb):
	"""
		Generates json files for depth frame collection and
		flashing ccb to module

		Inputs:
		  directory             - Directory to which json files are to be saved
		  sensorFW              - name of cfg file to be used
		  comp_ccb              - name of compressed ccb to be flashed
		  
		Outputs:
		  flashConfigPath       - name of flash json config path
		  depthConfigPath       - name of depth json config path
	"""
	Flashdict = dict()
	Flashdict["sensorFirmware"] = str(os.path.abspath(sensorFW))
	# Use CCB from NVM instead of from calibration folder
	#Flashdict["CCB_Calibration"] = str(os.path.abspath(comp_ccb))
	Flashdict["VAUX_POWER_VOLTAGE"] = "22"
	Flashdict["MAX_RANGE"] = "1500"
	Flashdict["MIN_RANGE"] = "10"
	Flashdict["AB_Max_RANGE"] = "2048"

	FlashJsonObject = json.dumps(Flashdict, indent=4)

	flashConfigPath = os.path.join(directory, 'flashConfig.json')
	with open(flashConfigPath, 'w') as outfile:
		outfile.write(FlashJsonObject)

	Depthdict = dict()
	Depthdict["VAUX_POWER_VOLTAGE"] = "22"
	Depthdict["MAX_RANGE"] = "1500"
	Depthdict["MIN_RANGE"] = "10"
	Depthdict["AB_Max_RANGE"] = "2048"

	DepthJsonObject = json.dumps(Depthdict, indent=4)

	depthConfigPath = os.path.join(directory, 'depthConfig.json')
	with open(depthConfigPath, 'w') as outfile:
		outfile.write(DepthJsonObject)

	return flashConfigPath, depthConfigPath



def CCB_write(config_file, CalibfolderName, barcode_ip="-1"):
	'''
		Generates .ccb calibration file

		Inputs:
		  config_file           - name of configuration (.cfg) file
		  CalibfolderName       - name of calbration folder
		  
		Outputs:
		  None
		
	'''

	with open(config_file) as y:
		file = yaml.load(y, Loader=yaml.SafeLoader)


	add_comp_P0 = file['CCB']['add_compressed_P0']
	add_uncomp_P0 = file['CCB']['add_uncompressed_P0']


	mypath = CalibfolderName
	# calFiles = [f for f in listdir(mypath) if isfile(join(mypath, f))]
	calDir = os.listdir(mypath)


	# Adding chipUID
	ICQ_file = "Connectivity_ICQ"
	ICQpath = os.path.join(CalibfolderName, ICQ_file)
	if os.path.exists(ICQpath):
		calFiles = [f for f in listdir(ICQpath) if isfile(join(ICQpath, f))]
		if ICQ_file + '.pkl' in calFiles:
			f = open(os.path.join(ICQpath, ICQ_file + ".pkl"), "rb")
			ICQ_dict = pickle.load(f)
			f.close()
			chipUID = (str(ICQ_dict['EFUSE_LOTID'][0]) + "_" + str(ICQ_dict['EFUSE_WAFER'][0]) + "_" 
						+ str(ICQ_dict['EFUSE_DIE_X'][0]) + "_" + str(ICQ_dict['EFUSE_DIE_Y'][0]))
		else:				
			#Chip ID not read back or ICQ test not completed
			chipUID = 'PXXXX.XX_W_XX_YY'
	else:
		chipUID = 'PXXXX.XX_W_XX_YY'

		
	if barcode_ip == 'erase':
		barcode_ip = "00000000000000000000000000000000"


	if add_comp_P0:

		with open(os.path.join(CalibfolderName, 'header.yaml')) as y:
			header = yaml.load(y, Loader=yaml.SafeLoader)

		header['barcode'] = barcode_ip
		header['chipUID'] = chipUID

		binaryCal, binaryCal_dtype, sizeCal, nBlocksCal = GenerateCalBlock(calDir, CalibfolderName, config_file , add_comp_P0=True, add_uncomp_P0=False)
		ccb_compressed_fname = CompileBlocks(binaryCal, binaryCal_dtype, sizeCal, nBlocksCal,CalibfolderName,barcode = header['barcode'],chipUID = header['chipUID'],add_comp_P0=True, add_uncomp_P0=False)
		header['compressed_ccb'] = ccb_compressed_fname
		
		flashConfigPath, depthConfigPath = GenerateDepthFlashJson(os.path.join(CalibfolderName, 'SessionConfig'), header['sensor_firmware'], ccb_compressed_fname)
		header['flashConfigJson'] = os.path.abspath(flashConfigPath)
		header['depthConfigJson'] = os.path.abspath(depthConfigPath)

		with open(os.path.join(CalibfolderName,'header.yaml'),'w') as f:
			yaml.dump(header, f)


	if add_uncomp_P0:

		with open(os.path.join(CalibfolderName, 'header.yaml')) as y:
			header = yaml.load(y, Loader=yaml.SafeLoader)

		header['barcode'] = barcode_ip

		binaryCal, binaryCal_dtype, sizeCal, nBlocksCal = GenerateCalBlock(calDir, CalibfolderName, config_file , add_comp_P0=False, add_uncomp_P0=True)
		ccb_fname = CompileBlocks(binaryCal, binaryCal_dtype, sizeCal, nBlocksCal,CalibfolderName,barcode = header['barcode'],chipUID = header['chipUID'],add_comp_P0=False, add_uncomp_P0=True)

		header['ccb'] = ccb_fname


		with open(os.path.join(CalibfolderName,'header.yaml'),'w') as f:
			yaml.dump(header, f)



if __name__ == "__main__":
	a = input("Enter name of calibration file directory: ")
	CCB_write(config_file = "./config/CalibrationConfig.yaml", CalibfolderName = a)