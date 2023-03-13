#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#

import numpy as np
import numpy.matlib
import csv
import os
import time
import matplotlib.pyplot as plt
import pickle
import sys
from . import CalibAPI as tof
import cv2
from . import Raw2Lx5 as R2L
from datetime import datetime
import logging
import yaml

class GainOffset:
	def __init__(self, GO_settings, file, CalibfolderName):
		self.GO_settings = GO_settings
		self.cal_dict = dict()
		self.origSetting = dict()
		self.CalibfolderName = CalibfolderName
		self.folderName = os.path.join(CalibfolderName, str(file['GainOffsetCalibration']['output_directory']))
		self.cfg_file = file['GainOffsetCalibration']['configuration_file']
		module_class = file['module_class']

		try:
			calibration_dir_path = file['calibration_directory']
		except:
			calibration_dir_path = ""

		if os.path.abspath(self.cfg_file) != self.cfg_file:
			self.cfg_file = os.path.join(calibration_dir_path, self.cfg_file)

		if not os.path.exists(self.folderName):
			os.mkdir(self.folderName)
		else:
			now = datetime.now()
			print('\nALERT: ' + self.folderName + ' already exists')
			old_folderName = self.folderName + str(now.strftime('_old_%Y%m%d_%H%M%S'))
			os.rename(self.folderName, old_folderName)
			print('ALERT: RENAMING OLD FOLDER: ' + self.folderName + ' as '+ old_folderName +'\n')
			os.mkdir(self.folderName)

		time.sleep(0.1)
		logging.basicConfig(filename=os.path.join(self.folderName, 'GainOffset.log'), 
							filemode='w', format='%(asctime)s - %(levelname)s - %(message)s',
							level=logging.INFO)
		print('Created log file')

		print("module_class = " + str(module_class))
		logging.info("module_class = " + str(module_class))

		print("configuration file cfg = " + str(self.cfg_file))
		logging.info("configuration file cfg = " + str(self.cfg_file))

		self.yaml_file = file
		self.ADC_cal_settings = file['GainOffsetCalibration']['ADC_calibration']
		self.Comp_DAC_cal_settings = file['GainOffsetCalibration']['Comp_DAC_calibration']

		self.API = tof.CalibAPI(module_class = module_class)
		self.API.ConnectSensor(Mode=3, cfg = self.cfg_file)


	def const_hex(self, val):
		'''
		Formats hex values to 0x0000 form
		
		Inputs:
		  val                   - hex value to be formatted
		  
		Outputs:
		  val                   - formatted hex value
		
		'''
		val = np.uint16(val)
		return '0x' + '0'*(max(0,4-len(str(hex(val))[2:]))) + str(hex(val))[2:]


	def ConfigureChip(self):
		'''
		Configures the camera before per column Gain and Offset calibration
		
		Inputs:
		  None
		  
		Outputs:
		  True
		
		'''
		val = self.API.regread(274)
		success= val == 22833
		self.API.error_assert(success, "Cannot communicate with chip")

		# ## TODO confirm chip  info
		# status, cfgver = self.API.N_APIComm(2, self.API.API['IO']['config_version']['get'])
		# self.API.error_assert(status, "Cannot get Config version")

		val = self.API.regread(0x0528)
		# self.API.error_assert(status, "Cannot determine if multigain settings have already been applied")

		if (int(val)&6) == 6:
			print('\n\nMulti-gain calibration is already applied\n\n')
			logging.info('\n\nMulti-gain calibration is already applied\n\n')
		else:
			success = self.API.regrmw(0x0528, 0x0FFFF, 0x0006)

		success = success and self.API.regwrite(0x0014, 0x0000) # blindly unclock gate all digital
		success = success and self.API.regwrite(0x0500, 0x1002) # Configure read from LX5 DRAM		
		success = success and self.API.regwrite(0x0502, 0x0006) 
		s, val = self.API.regreadburst(0x0506, 5)
		self.cal_dict['gainCalConfigVersion'] = int(val[4])

		val = self.API.regread(0x0D1A)
		# self.API.error_assert(success, "Failed to determine 9b/10b mode")

		self.tenBitMode = bool(not self.API.bitget(int(val),3)) # get bit in 3rd position from LSB (4 in MATLAB)
		self.cal_dict['ADC'] = dict()


		ADC_dict = dict(
						sensormodeID = self.ADC_cal_settings['sensormodeID'], # passive IR
						nFrames = self.ADC_cal_settings['nFrames'],
						DAC_AMPTEST0 = self.ADC_cal_settings['DAC_AMPTEST0'], # ~1.61V
						DAC_AMPTEST1 = self.ADC_cal_settings['DAC_AMPTEST1'], # ~2.05V
						iramp = self.ADC_cal_settings['iramp'],
						ADC_FS = self.ADC_cal_settings['ADC_FS'], # ADC Full Scale = 2Vpp
						Cin = self.ADC_cal_settings['Cin'], # Ampgain setting of 0.99x
						Cfb = self.ADC_cal_settings['Cfb'],
						AmpGainMeas = self.ADC_cal_settings['AmpGainMeas'], # actual amp gain (measured using two single ended probes)
						cleanup = self.ADC_cal_settings['cleanup'] # cleanup after calibration, i,e restore original settings
						)

		#Comp_DAC_cal_settings  =file['Comp_DAC_calibration']

		compDict = dict(
						NFRAMES = self.Comp_DAC_cal_settings['NFRAMES'], # number of frames to collect per DAC step
						doplot = self.Comp_DAC_cal_settings['doplot'], # flag to generate plots and intermediate outputs
						compCalThres = self.Comp_DAC_cal_settings['compCalThres'], # Threshold for number of comparator samples per column that must have transitioned to the next gain (default if not specified is 99.5%)
						compCalSatLimit = self.Comp_DAC_cal_settings['compCalSatLimit'], # Fail gain calibration if more than compCalSatLimit pixels are saturated within any column at the original comparator DAC setting (default if not specified is 2 pixels per column)
						compCalMargin = self.Comp_DAC_cal_settings['compCalMargin'] # Required margin from max/min ADC value.  This margin is used when adjusting comparator DAC after finding the existing switching point (default if not specified is 5% of full scale)
						)

		# Calibrate ADC ramp and up/dn counters
		self.cal_dict['ADC']['iramp'], self.cal_dict['ADC']['updnoffset'] = self.ADCCalib(ADC_dict)
		print('ADC Calibration values: \nADC iramp: ', self.cal_dict['ADC']['iramp'], '\nADC updnoffset: ', self.cal_dict['ADC']['updnoffset'])
		logging.info('ADC Calibration values: \nADC iramp: ' + str(self.cal_dict['ADC']['iramp']) + '\nADC updnoffset: ' + str(self.cal_dict['ADC']['updnoffset']))


		# Calibrate gain comparators
		compDac, self.info = self.ComparatorDACCal(compDict)

		self.cal_dict['gainComparator'] = dict()
		self.cal_dict['gainComparator']['Vref1DAC'] = compDac[0]
		self.cal_dict['gainComparator']['Vref1Set'] = not np.isnan(compDac[0])
		self.cal_dict['gainComparator']['Vref2DAC'] = compDac[1]
		self.cal_dict['gainComparator']['Vref2Set'] = not np.isnan(compDac[1])
		self.cal_dict['gainComparator']['Vref3DAC'] = compDac[2]
		self.cal_dict['gainComparator']['Vref3Set'] = not np.isnan(compDac[2])
		if len(compDac) > 3:
			self.cal_dict['gainComparator']['VCMDAC'] = compDac[2]
			self.cal_dict['gainComparator']['VCMSet'] = not np.isnan(compDac[2])
		else:
			self.cal_dict['gainComparator']['VCMSet'] = False

		print('\nSAVING INTERMEDIATE FILES...\n')
		logging.info('\nSAVING INTERMEDIATE FILES...\n')
		f = open(os.path.join(self.folderName, 'GO_cal_dict_intermediate.pkl'), 'wb')
		pickle.dump(self.cal_dict, f)
		f.close()

		f = open(os.path.join(self.folderName, 'GO_info_intermediate.pkl'), 'wb')
		pickle.dump(self.info, f)
		f.close()

		return True


	def PerColGOCalib(self):
		'''
		Performs Per column Gain and Offset calibration
		
		Inputs:
		  None
		  
		Outputs:
		  cal_dict              - Dictionary containing calibrated values
		  info                  - Dictionary containing calibration info
		
		'''

		cal_dict = self.cal_dict
		info = self.info

		self.API.DumpTimingTool('WriteToChip', 0, info['DE_Struct'])

		print("\n-------Per-column Gain and Offset Calibration-------")
		logging.info("\n-------Per-column Gain and Offset Calibration-------")

		tmp = info['dacDiffVoltageTargetPre']
		tmp = np.append(tmp, np.reshape(tmp[:,2],(len(tmp[:,2]),1)), axis=1)
		info['dacDiffVoltageTargetPre'] = tmp # Use 3rd gain transition voltage to calibrate 4th gain
		info['DacSel'] = np.append(info['DacSel'], info['DacSel'][2])
		info['normalizedSatPerCol'] = 0

		forceSingleGain = [0x2A00, 0x2C00, 0x3400, 0x5400]
		dumpTimingLastState = ''
		info['gainCalVoltageDiff'] = np.zeros([4,2])

		NFRAMES = self.GO_settings['NFRAMES']
		compCalSatLimit = self.GO_settings['compCalSatLimit']
		ADC_FS = self.GO_settings['ADC_FS']
		doplot = self.GO_settings['doplot']

		ta, tb, SAT_test = self.API.GetFramesPCGO(NFRAMES) #
		ADC_Native = np.zeros(np.append(np.shape(SAT_test), (4,2)))


		s=self.API.regwrite(0x0014,0x0000)

		for gainTransition in range(3,-1,-1):
			for posNeg in range(1,-1,-1):
				print('\n--- Gain', gainTransition, '---')
				logging.info('\n--- Gain' + str(gainTransition) + '---')
				print('Setting %17s to target: %3.0fmV'
					%(info['DacSel'][gainTransition],info['dacDiffVoltageTargetPre'][posNeg,gainTransition]*1e3))
				logging.info(str('Setting %17s to target: %3.0fmV'
					%(info['DacSel'][gainTransition],info['dacDiffVoltageTargetPre'][posNeg,gainTransition]*1e3)))
				_, _, _ = self.API.SetDiffDacSfVoltage(info['DacCal'],info['DacSel'][gainTransition],
													info['dacDiffVoltageTargetPre'][posNeg,gainTransition],info['CMwindow'])
				success = self.API.regrmw(0x012A, forceSingleGain[gainTransition], 0x7E00) # set comp_ref_disable[5:0] to force single gain
				self.API.error_assert(success, 'Failed to force single gain')

				if info['DacSel'][gainTransition].upper() == 'CMREF-AMPTEST0':
					
					if dumpTimingLastState != 'CMREF-AMPTEST0': # configure DE timing to use DAC inputs for both native and binned modes
						tmp = self.API.DumpTimingTransformation(info['DE_Struct'],'native_resolution',[],'ampTestSel_dacAmpTest1_dacAmpTest1_dacCMREF_dacAmpTest0')
						tmp1 = self.API.DumpTimingTransformation(tmp,'binned',[],'ampTestSel_dacAmpTest1_dacAmpTest1_dacCMREF_dacAmpTest0')
						self.API.DumpTimingTool('WriteToChip', 0, tmp1)
					info['gainCalVoltageDiff'][gainTransition][posNeg] = self.API.CamMeasureAnaTestVoltage('DACwithSF_CMREF-AMPTEST0')
					count = 5
					while((abs(info['gainCalVoltageDiff'][gainTransition][posNeg]) < 0.01) and (abs(info['gainCalVoltageDiff'][gainTransition][posNeg]) > 3.00) and (count > 1)):
						time.sleep(1)
						self.API.testBoardADC()
						info['gainCalVoltageDiff'][gainTransition][posNeg] = self.API.CamMeasureAnaTestVoltage('DACwithSF_CMREF-AMPTEST0')
						count = count - 1

				elif info['DacSel'][gainTransition].upper() == 'AMPTEST1-AMPTEST0':

					if dumpTimingLastState != 'AMPTEST1-AMPTEST0':
						tmp = self.API.DumpTimingTransformation(info['DE_Struct'],'native_resolution',[],'ampTestSel_dacCMREF_dacCMREF_dacAmpTest1_dacAmpTest0')
						tmp1 = self.API.DumpTimingTransformation(tmp,'binned',[],'ampTestSel_dacCMREF_dacCMREF_dacAmpTest1_dacAmpTest0')
						self.API.DumpTimingTool('WriteToChip', 0, tmp1)
					info['gainCalVoltageDiff'][gainTransition][posNeg] = self.API.CamMeasureAnaTestVoltage('DACwithSF_AMPTEST1-AMPTEST0')
					while((abs(info['gainCalVoltageDiff'][gainTransition][posNeg]) < 0.01) and (abs(info['gainCalVoltageDiff'][gainTransition][posNeg]) > 3.00) and (count > 1)):
						time.sleep(1)
						self.API.testBoardADC()
						info['gainCalVoltageDiff'][gainTransition][posNeg] = self.API.CamMeasureAnaTestVoltage('DACwithSF_AMPTEST1-AMPTEST0')
						count = count - 1

				else:
					raise Exception('Incorrect DacSel value')
					logging.error('Incorrect DacSel value')

				dumpTimingLastState = info['DacSel'][gainTransition].upper()
				print('Measured voltage: %6.1fmV' %(info['gainCalVoltageDiff'][gainTransition,posNeg]*1000))
				logging.info(str('Measured voltage: %6.1fmV' %(info['gainCalVoltageDiff'][gainTransition,posNeg]*1000)))

				GT, ADC_tmp, SAT = self.API.GetFramesPCGO(NFRAMES)
				# GT = ta
				# ADC_tmp = tb 
				# SAT = SAT_test

				if np.any(GT != gainTransition):
					raise Exception('Incorrect forced comparator gain setting returned')
					logging.error('Incorrect forced comparator gain setting returned')

				if np.any(SAT):
					normalizedSatPerCol = (np.sum(np.reshape(SAT.transpose(1,0,2,3), (list(np.shape(SAT))[1], -1)),1) / 
											(np.shape(SAT)[0] * np.shape(SAT)[2]* np.shape(SAT)[3]))

					print("Pixels saturated!", np.max(normalizedSatPerCol), "limit:",compCalSatLimit)
					logging.warning("Pixels saturated!" + str(np.max(normalizedSatPerCol)) + "limit:"+ str(compCalSatLimit))
					if np.max(normalizedSatPerCol) > compCalSatLimit:
						raise Exception('Saturation occurred for gain')
						logging.error('Saturation occurred for gain')
					else:
						print('Saturation occurred for gain, but was within allowed limit')
						logging.warning('Saturation occurred for gain, but was within allowed limit')
						ADC_tmp = ADC_tmp.astype(np.float)
						ADC_tmp[SAT] = np.nan

				ADC_Native[:,:,:,:,gainTransition,posNeg] = ADC_tmp


		print('\nMeasuring per column offsets (native dump)')
		logging.info('\nMeasuring per column offsets (native dump)')

		self.API.DumpTimingTool('WriteToChip', 0, info['DE_Struct'])


		status = self.API.UpdateUseCaseFrameSetting(3, 'DRAINSTYLE', 1)
		self.API.error_assert(status, 'Unable to set drain style')
		s = self.API.regwrite(0x0014, 0x0000)

		s = self.API.regwrite(0x0E1C, 0x0000)
		s = self.API.regwrite(0x0E1E, 0x0010)
		s = self.API.regwrite(0x0E20, 0x0000)
		s = self.API.regwrite(0x0E22, 0x0010)
		s = self.API.regwrite(0x0E24, 0x0000)
		s = self.API.regwrite(0x0E26, 0x0010)
		s = self.API.regwrite(0x0E28, 0x0000)
		s = self.API.regwrite(0x0E2A, 0x0010)
		s = self.API.regwrite(0x0E2C, 0x0000)
		s = self.API.regwrite(0x0E2E, 0x0010)
		_, _, SAT_test = self.API.GetFramesPCGO(NFRAMES) 

		self.API.error_assert(success, 'Cannot write amp_mux registers')
		measuredOffsetNative = np.zeros([1024,2,4])

		for gainTransition in range(3,-1,-1):
			success = self.API.regrmw(0x012A, forceSingleGain[gainTransition], 0x7E00) # set comp_ref_disable[5:0] to force single gain
			time.sleep(0.1)
			# print('regrmw value', gainTransition, '-->', forceSingleGain[gainTransition])
			self.API.error_assert(success, 'Failed to force single gain')
			_, ADC_tmp, SAT = self.API.GetFramesPCGO(NFRAMES)
			print('Max ADC:', np.max(ADC_tmp))
			logging.info('Max ADC:' + str(np.max(ADC_tmp)))
			print('Min ADC:', np.min(ADC_tmp))
			logging.info('Min ADC:' + str(np.min(ADC_tmp)))
			print('Mean ADC:', np.mean(ADC_tmp))
			logging.info('Mean ADC:' + str(np.mean(ADC_tmp)))
			# np.save('Offsetframe' + str(gainTransition), ADC_tmp)

			if np.any(SAT):
				normalizedSatPerCol = (np.sum(np.reshape(SAT.transpose(1,0,2,3), (list(np.shape(SAT))[1], -1)),1) / 
										(np.shape(SAT)[0] * np.shape(SAT)[2]* np.shape(SAT)[3]))

				if np.max(normalizedSatPerCol) > compCalSatLimit:
					raise Exception('Saturation occurred for gain')
					logging.error('Saturation occurred for gain')
				else:
					print('Saturation occurred for gain, but was within allowed limit')
					logging.warning('Saturation occurred for gain, but was within allowed limit')
					ADC_tmp = ADC_tmp.astype(np.float)
					ADC_tmp[SAT] = np.nan

			ADC_tmp = np.squeeze(np.transpose(ADC_tmp, (1,0,2,3)))
			measuredOffsetNative[:,0,gainTransition] = np.nanmean(ADC_tmp[:, ::2],1) # even rows offset
			measuredOffsetNative[:,1,gainTransition] = np.nanmean(ADC_tmp[:, 1::2],1) # even rows offset

		del GT, SAT, ADC_tmp

		ADC_Native = np.transpose(ADC_Native, (1,0,4,5,2,3)) # change to order: [column, row, gain,upper/lower value, phase, frame#]

		f = open(os.path.join(self.folderName, 'ADC_Native.pkl'), 'wb')
		pickle.dump(ADC_Native,f)
		f.close()

		tmp = np.transpose(ADC_Native[:,:,:,0,:,:], (2,0,1,3,4))
		info['ADC_Native_max'] = np.max(np.reshape(tmp,(4,-1)), 1)
		info['ADC_Native_max'][np.any(np.isnan(np.reshape(tmp,(4,-1))), axis=1)]
		info['ADC_Native_mean_high'] = np.nanmean(np.reshape(tmp,(4,-1)), 1)
		tmp = np.transpose(ADC_Native[:,:,:,1,:,:], (2,0,1,3,4))
		info['ADC_Native_min'] = np.min(np.reshape(tmp,(4,-1)), 1)
		info['ADC_Native_min'][np.any(np.isnan(np.reshape(tmp,(4,-1))), axis=1)]
		info['ADC_Native_mean_low'] = np.nanmean(np.reshape(tmp,(4,-1)), 1)

		print('Gain cal: Min ADC val per gain (highest gain to lowest gain):       %3.0f, %3.0f, %3.0f, %3.0f' % tuple(info['ADC_Native_min']))
		logging.info(str('Gain cal: Min ADC val per gain (highest gain to lowest gain):       %3.0f, %3.0f, %3.0f, %3.0f' % tuple(info['ADC_Native_min'])))

		print('Gain cal: Mean low ADC val per gain (highest gain to lowest gain):  %3.0f, %3.0f, %3.0f, %3.0f' % tuple(info['ADC_Native_mean_low']))
		logging.info(str('Gain cal: Mean low ADC val per gain (highest gain to lowest gain):  %3.0f, %3.0f, %3.0f, %3.0f' % tuple(info['ADC_Native_mean_low'])))

		print('Gain cal: Mean high ADC val per gain (highest gain to lowest gain): %3.0f, %3.0f, %3.0f, %3.0f' % tuple(info['ADC_Native_mean_high']))
		logging.info(str('Gain cal: Mean high ADC val per gain (highest gain to lowest gain): %3.0f, %3.0f, %3.0f, %3.0f' % tuple(info['ADC_Native_mean_high'])))

		print('Gain cal: Max ADC val per gain (highest gain to lowest gain):       %3.0f, %3.0f, %3.0f, %3.0f' % tuple(info['ADC_Native_max']))
		logging.info(str('Gain cal: Max ADC val per gain (highest gain to lowest gain):       %3.0f, %3.0f, %3.0f, %3.0f' % tuple(info['ADC_Native_max'])))

		ADC_Native = np.nanmean(ADC_Native, axis = 5)  # take mean of dimensions [frame#, phase]
		ADC_Native = np.squeeze(np.nanmean(ADC_Native, axis = 4))

		tmp = list(np.shape(ADC_Native))
		tmp[1] = 2
		ADC_out = np.zeros(tmp) # take mean of every 2nd row (dimensions [col, even/odd row, gain, upper/lower value])

		ADC_out[:,0,:,:] = np.nanmean(ADC_Native[:,::2,:,:], axis=1)
		ADC_out[:,1,:,:] = np.nanmean(ADC_Native[:,1::2,:,:], axis=1)

		tmp_divisor = np.reshape(info['gainCalVoltageDiff'][:,1] - info['gainCalVoltageDiff'][:,0],(1,1,4))
		measuredGainNative = (ADC_out[:,:,:,1] - ADC_out[:,:,:,0]) / tmp_divisor

		info['ampgainMeasuredNative'] = np.squeeze(np.mean(np.median(measuredGainNative,axis=0),axis=0)) / (2**(9+self.tenBitMode)/(ADC_FS*2))
		print('Measured ampgains (native dump): %.2fx, %.2fx, %.2fx, %.2fx' %tuple(info['ampgainMeasuredNative']))
		logging.info(str('Measured ampgains (native dump): %.2fx, %.2fx, %.2fx, %.2fx' %tuple(info['ampgainMeasuredNative'])))
		expectedAmpGain = self.API.ReadAmpGainSetting()

		# perform a gross check that gains are within 20% of expected value
		measuredGainLimit = 0.2
		ampGainLimit = ((info['ampgainMeasuredNative']).flatten() / np.array(expectedAmpGain).flatten() > (1+measuredGainLimit))
		if np.any(ampGainLimit):
			raise Exception('Measured gain above limit')
			logging.error('Measured gain above limit')
		ampGainLimit = ((info['ampgainMeasuredNative']).flatten() / np.array(expectedAmpGain).flatten() < (1-measuredGainLimit))
		if np.any(ampGainLimit):
			raise Exception('Measured gain below limit')
			logging.error('Measured gain below limit')

		f1 = open(os.path.join(self.folderName, "measuredGainNative.pkl"), 'wb')
		measuredGainNative_dict = dict()
		measuredGainNative_dict['measuredGainNative'] = measuredGainNative
		measuredGainNative_dict['tenBitMode'] = self.tenBitMode
		measuredGainNative_dict['ADC_FS'] = ADC_FS
		pickle.dump(measuredGainNative_dict, f1)
		f1.close()

		f2 = open(os.path.join(self.folderName, "measuredOffsetNative.pkl"), 'wb')
		measuredOffsetNative_dict = dict()
		measuredOffsetNative_dict['measuredOffsetNative'] = measuredOffsetNative
		measuredOffsetNative_dict['tenBitMode'] = self.tenBitMode
		measuredOffsetNative_dict['ADC_FS'] = ADC_FS
		pickle.dump(measuredOffsetNative_dict, f2)
		f2.close()


		# Gain and Offset Correction
		highestGainMeas = np.mean(np.median(measuredGainNative[:,:,0],axis=0))
		nominalGain = np.append(np.array([1]), highestGainMeas/ np.mean(np.median(measuredGainNative[:,:,1:4],axis=0),axis=0))
		targetGainNative = highestGainMeas / measuredGainNative

		inverseGlobalADCGain, perColGainAdj, clampedGain = R2L.GainRaw2Lx5(targetGainNative, nominalGain)
		if np.any(clampedGain) == True:
			raise Exception('WSF:GainOffset:PerColumnGainClamped')
			logging.error('WSF:GainOffset:PerColumnGainClamped')

		success, ramDataGain, perColumnGainAdjustment = R2L.GeneratePerColRam(perColGainAdj)
		# ramDataGain is in uint16 format and ready to write to RAM -> 12800 x 2bytes
		# perColumnGainAdjustment is a dictionary with data per mode
		# ramDataGain = [A1D1;A1D2;A1D4;A2D1;A2D2;A4D1]

		if self.tenBitMode:
			perColOffset, clampedOffset = R2L.OffsetRaw2Lx5(measuredOffsetNative-512,'bpp_10')
		else:
			perColOffset, clampedOffset = R2L.OffsetRaw2Lx5(measuredOffsetNative-256,'bpp_9')

		if np.any(clampedOffset) == True:
			raise Exception('WSF:GainOffset:PerColumnOffsetClamped')
			logging.error('WSF:GainOffset:PerColumnOffsetClamped')

		success, ramDataOffset, perColumnOffsetAdjustment = R2L.GeneratePerColRam(perColOffset)
		# ramDataOffset is in uint16 format and ready to write to RAM -> 12800 x 2bytes
		# perColumnOffsetAdjustment is a dictionary with data per mode
		# ramDataOffset = [A1D1;A1D2;A1D4;A2D1;A2D2;A4D1]
		self.cal_dict['inverseGlobalADCGain'] = inverseGlobalADCGain
		self.cal_dict['perColumnGainAdjustment'] = perColumnGainAdjustment
		self.cal_dict['perColumnOffsetAdjustment'] = perColumnOffsetAdjustment
		self.cal_dict['ramDataGain'] = ramDataGain
		self.cal_dict['ramDataOffset'] = ramDataOffset

		self.cal_dict['info'] = dict()
		now = datetime.now()
		self.cal_dict['info']['CheckSum'] = 0
		self.cal_dict['info']['Year'] = int(now.strftime('%Y'))
		self.cal_dict['info']['Month'] = int(now.strftime('%m'))
		self.cal_dict['info']['Day'] = int(now.strftime('%d'))
		self.cal_dict['info']['Hour'] = int(now.strftime('%H'))
		self.cal_dict['info']['Minute'] = int(now.strftime('%M'))
		self.cal_dict['info']['Second'] = int(now.strftime('%S'))
		self.cal_dict['info']['configuration_file'] = self.cfg_file

		limits_file = self.yaml_file['limits_file']

		try:
			calibration_dir_path = self.yaml_file['calibration_directory']
		except:
			calibration_dir_path = ""

		if os.path.abspath(limits_file) != limits_file:
			limits_file = os.path.join(calibration_dir_path, limits_file)

		try:
			with open(limits_file) as y:
				limits = yaml.load(y, Loader=yaml.SafeLoader)['GainOffsetCalibration']
		except:
			raise Exception("\n\nWARNING: Incorrect/non-existing limits file\n\n")

		self.cal_dict['gain_offset_pass'] = True #Initialize to True

		adc_delay = np.round(self.cal_dict['ADC']['linear_delay'],3)
		if limits['adc_delay'][0] < adc_delay < limits['adc_delay'][1]:
			adc_delay_status = "PASSED"
		else:
			adc_delay_status = "FAILED"
			print('\nADC offset delay FAIL\n')
			self.cal_dict['gain_offset_pass'] = False
		self.cal_dict['ADC']['linear_delay'] = [adc_delay, adc_delay_status, limits['adc_delay'][0], limits['adc_delay'][1]]

		MeasuredGain0 = np.round(info['ampgainMeasuredNative'][0],3)
		if MeasuredGain0 > limits['Gain0_MeasNative'][0]:
			Gain0_Native_status = "PASSED"
		else:
			Gain0_Native_status = "FAILED"
			print('\nGain0 Measured Native FAIL\n')
			self.cal_dict['gain_offset_pass'] = False
		self.cal_dict['Gain0_MeasNative'] = [MeasuredGain0, Gain0_Native_status, limits['Gain0_MeasNative'][0], limits['Gain0_MeasNative'][1]]

		MeasuredGain1 = np.round(info['ampgainMeasuredNative'][1],3)
		if MeasuredGain1 > limits['Gain1_MeasNative'][0]:
			Gain1_Native_status = "PASSED"
		else:
			Gain1_Native_status = "FAILED"
			print('\nGain1 Measured Native FAIL\n')
			self.cal_dict['gain_offset_pass'] = False
		self.cal_dict['Gain1_MeasNative'] = [MeasuredGain1, Gain1_Native_status, limits['Gain1_MeasNative'][0], limits['Gain1_MeasNative'][1]]

		MeasuredGain2 = np.round(info['ampgainMeasuredNative'][2],3)
		if MeasuredGain2 > limits['Gain2_MeasNative'][0]:
			Gain2_Native_status = "PASSED"
		else:
			Gain2_Native_status = "FAILED"
			print('\nGain2 Measured Native FAIL\n')
			self.cal_dict['gain_offset_pass'] = False
		self.cal_dict['Gain2_MeasNative'] = [MeasuredGain2, Gain2_Native_status, limits['Gain2_MeasNative'][0], limits['Gain2_MeasNative'][1]]

		MeasuredGain3 = np.round(info['ampgainMeasuredNative'][3],3)
		if MeasuredGain3 > limits['Gain3_MeasNative'][0]:
			Gain3_Native_status = "PASSED"
		else:
			Gain3_Native_status = "FAILED"
			print('\nGain3 Measured Native FAIL\n')
			self.cal_dict['gain_offset_pass'] = False
		self.cal_dict['Gain3_MeasNative'] = [MeasuredGain3, Gain3_Native_status, limits['Gain3_MeasNative'][0], limits['Gain3_MeasNative'][1]]

		Gain0_ADC_min = np.round(self.info['ADC_Native_min'][0],3)
		if limits['Gain0_ADC_min'][0] < Gain0_ADC_min < limits['Gain0_ADC_min'][1]:
			Gain0_ADC_min_status = "PASSED"
		else:
			Gain0_ADC_min_status = "FAILED"
			print('\nGain0 ADC min margin FAIL\n')
			self.cal_dict['gain_offset_pass'] = False
		self.cal_dict['Gain0_ADC_min'] = [Gain0_ADC_min, Gain0_ADC_min_status, limits['Gain0_ADC_min'][0], limits['Gain0_ADC_min'][1]]

		Gain0_ADC_max = np.round(self.info['ADC_Native_max'][0],3)
		if limits['Gain0_ADC_max'][0] < Gain0_ADC_max < limits['Gain0_ADC_max'][1]:
			Gain0_ADC_max_status = "PASSED"
		else:
			Gain0_ADC_max_status = "FAILED"
			print('\nGain0 ADC max margin FAIL\n')
			self.cal_dict['gain_offset_pass'] = False
		self.cal_dict['Gain0_ADC_max'] = [Gain0_ADC_max, Gain0_ADC_max_status, limits['Gain0_ADC_max'][0], limits['Gain0_ADC_max'][1]]

		Gain1_ADC_min = np.round(self.info['ADC_Native_min'][1],3)
		if limits['Gain1_ADC_min'][0] < Gain1_ADC_min < limits['Gain1_ADC_min'][1]:
			Gain1_ADC_min_status = "PASSED"
		else:
			Gain1_ADC_min_status = "FAILED"
			print('\nGain1 ADC min margin FAIL\n')
			self.cal_dict['gain_offset_pass'] = False
		self.cal_dict['Gain1_ADC_min'] = [Gain1_ADC_min, Gain1_ADC_min_status, limits['Gain1_ADC_min'][0], limits['Gain1_ADC_min'][1]]

		Gain1_ADC_max = np.round(self.info['ADC_Native_max'][1],3)
		if limits['Gain1_ADC_max'][0] < Gain1_ADC_max < limits['Gain1_ADC_max'][1]:
			Gain1_ADC_max_status = "PASSED"
		else:
			Gain1_ADC_max_status = "FAILED"
			print('\nGain1 ADC max margin FAIL\n')
			self.cal_dict['gain_offset_pass'] = False
		self.cal_dict['Gain1_ADC_max'] = [Gain1_ADC_max, Gain1_ADC_max_status, limits['Gain1_ADC_max'][0], limits['Gain1_ADC_max'][1]]

		Gain2_ADC_min = np.round(self.info['ADC_Native_min'][2],3)
		if limits['Gain2_ADC_min'][0] < Gain2_ADC_min < limits['Gain2_ADC_min'][1]:
			Gain2_ADC_min_status = "PASSED"
		else:
			Gain2_ADC_min_status = "FAILED"
			print('\nGain2 ADC min margin FAIL\n')
			self.cal_dict['gain_offset_pass'] = False
		self.cal_dict['Gain2_ADC_min'] = [Gain2_ADC_min, Gain2_ADC_min_status, limits['Gain2_ADC_min'][0], limits['Gain2_ADC_min'][1]]

		Gain2_ADC_max = np.round(self.info['ADC_Native_max'][2],3)
		if limits['Gain2_ADC_max'][0] < Gain2_ADC_max < limits['Gain2_ADC_max'][1]:
			Gain2_ADC_max_status = "PASSED"
		else:
			Gain2_ADC_max_status = "FAILED"
			print('\nGain2 ADC max margin FAIL\n')
			self.cal_dict['gain_offset_pass'] = False
		self.cal_dict['Gain2_ADC_max'] = [Gain2_ADC_max, Gain2_ADC_max_status, limits['Gain2_ADC_max'][0], limits['Gain2_ADC_max'][1]]

		comp_vref1 = self.info['voltage_comp_vref1']
		if comp_vref1 > limits['comp_vref1'][0]:
			comp_vref1_status = "PASSED"
		else:
			comp_vref1_status = "FAILED"
			print('\nCompVref1 voltage FAIL\n')
			self.cal_dict['gain_offset_pass'] = False
		self.cal_dict['comp_vref1'] = [comp_vref1, comp_vref1_status, limits['comp_vref1'][0], limits['comp_vref1'][1]]

		comp_vref2 = self.info['voltage_comp_vref2']
		if comp_vref2 > limits['comp_vref2'][0]:
			comp_vref2_status = "PASSED"
		else:
			comp_vref2_status = "FAILED"
			print('\nCompVref2 voltage FAIL\n')
			self.cal_dict['gain_offset_pass'] = False
		self.cal_dict['comp_vref2'] = [comp_vref2, comp_vref2_status, limits['comp_vref2'][0], limits['comp_vref2'][1]]

		comp_vref3 = self.info['voltage_comp_vref3']
		if comp_vref3 > limits['comp_vref3'][0]:
			comp_vref3_status = "PASSED"
		else:
			comp_vref3_status = "FAILED"
			print('\nCompVref3 voltage FAIL\n')
			self.cal_dict['gain_offset_pass'] = False
		self.cal_dict['comp_vref3'] = [comp_vref3, comp_vref3_status, limits['comp_vref3'][0], limits['comp_vref3'][1]]

		self.cal_dict['info']['Pass'] = self.cal_dict['gain_offset_pass']

		f1 = open(os.path.join(self.folderName, self.yaml_file['GainOffsetCalibration']['output_file_pkl'] + '.pkl'), 'wb')
		pickle.dump(self.cal_dict, f1)
		f1.close()

		f2 = open(os.path.join(self.folderName, "GO_info.pkl"), 'wb')
		pickle.dump(self.info, f2)
		f2.close()

		# ### Writing individual results ###

		# csv_headers = ['Test', 'Value', 'Result', 'Min Limit', 'Max Limit']
			
		# cr = csv.writer(open(os.path.join(self.folderName, 'GO_preliminary_results.csv'), 'w', newline=''))
		# cr.writerow(csv_headers)
		# #cr.writerow(['gain_offset_pass',self.cal_dict["info"]["Pass"]])
		# cr.writerow(['adc_iramp',self.cal_dict['ADC']['iramp']])
		# cr.writerow(['adc_delay',self.cal_dict['ADC']['linear_delay']])
		# cr.writerow(['adc_up_dn_delay',self.cal_dict['ADC']['updnoffset']])

		# cr.writerow(['Gain0_MeasNative',self.cal_dict['Gain0_MeasNative']])
		# cr.writerow(['Gain1_MeasNative',self.cal_dict['Gain1_MeasNative']])
		# cr.writerow(['Gain2_MeasNative',self.cal_dict['Gain2_MeasNative']])
		# cr.writerow(['Gain3_MeasNative',self.cal_dict['Gain3_MeasNative']])

		# cr.writerow(['GainCal_0_ADC_min',self.cal_dict['Gain0_ADC_min']])
		# cr.writerow(['GainCal_0_ADC_mean_low',np.round(self.info['ADC_Native_mean_low'][0],3)])
		# cr.writerow(['GainCal_0_ADC_mean_high',np.round(self.info['ADC_Native_mean_high'][0],3)])
		# cr.writerow(['GainCal_0_ADC_max',self.cal_dict['Gain0_ADC_max']])

		# cr.writerow(['GainCal_1_ADC_min',self.cal_dict['Gain1_ADC_min']])
		# cr.writerow(['GainCal_1_ADC_mean_low',np.round(self.info['ADC_Native_mean_low'][1],3)])
		# cr.writerow(['GainCal_1_ADC_mean_high',np.round(self.info['ADC_Native_mean_high'][1],3)])
		# cr.writerow(['GainCal_1_ADC_max',self.cal_dict['Gain1_ADC_max']])

		# cr.writerow(['GainCal_2_ADC_min',self.cal_dict['Gain2_ADC_min']])
		# cr.writerow(['GainCal_2_ADC_mean_low',np.round(self.info['ADC_Native_mean_low'][2],3)])
		# cr.writerow(['GainCal_2_ADC_mean_high',np.round(self.info['ADC_Native_mean_high'][2],3)])
		# cr.writerow(['GainCal_2_ADC_max',self.cal_dict['Gain2_ADC_max']])

		# cr.writerow(['GainCal_3_ADC_min',np.round(self.info['ADC_Native_min'][3],3)])
		# cr.writerow(['GainCal_3_ADC_mean_low',np.round(self.info['ADC_Native_mean_low'][3],3)])
		# cr.writerow(['GainCal_3_ADC_mean_high',np.round(self.info['ADC_Native_mean_high'][3],3)])
		# cr.writerow(['GainCal_3_ADC_max',np.round(self.info['ADC_Native_max'][3],3)])

		# cr.writerow(['Vref1DAC',np.round(self.cal_dict['gainComparator']['Vref1DAC'],2)])
		# cr.writerow(['Vref2DAC',np.round(self.cal_dict['gainComparator']['Vref2DAC'],2)])
		# cr.writerow(['Vref3DAC',np.round(self.cal_dict['gainComparator']['Vref3DAC'],2)])

		# cr.writerow(['voltage_comp_vref1',self.cal_dict['comp_vref1']])
		# cr.writerow(['voltage_comp_vref2',self.cal_dict['comp_vref2']])
		# cr.writerow(['voltage_comp_vref3',self.cal_dict['comp_vref3']])

		# try:
		# 	cr.writerow(['VCMDAC',np.round(self.cal_dict['gainComparator']['VCMDAC'],2)])
		# except:
		# 	cr.writerow(['VCMDAC','NaN'])

		# cr.writerow(['inverseGlobalADCGain_0',self.cal_dict['inverseGlobalADCGain'][0]])
		# cr.writerow(['inverseGlobalADCGain_1',self.cal_dict['inverseGlobalADCGain'][1]])
		# cr.writerow(['inverseGlobalADCGain_2',self.cal_dict['inverseGlobalADCGain'][2]])
		# cr.writerow(['inverseGlobalADCGain_3',self.cal_dict['inverseGlobalADCGain'][3]])


		##################################


		print('Calibration file with', self.cal_dict.keys())
		c = csv.writer(open(os.path.join(self.folderName, self.yaml_file['GainOffsetCalibration']['output_file_csv'] +'.csv'), 'w', newline=''))

		c.writerow([self.const_hex(self.cal_dict['ADC']['updnoffset'])])
		c.writerow([self.const_hex(self.cal_dict['ADC']['iramp'])])

		c.writerow([self.const_hex(self.cal_dict['gainComparator']['Vref1DAC'])])
		c.writerow([self.const_hex(self.cal_dict['gainComparator']['Vref1Set'])])
		c.writerow([self.const_hex(self.cal_dict['gainComparator']['Vref2DAC'])])
		c.writerow([self.const_hex(self.cal_dict['gainComparator']['Vref2Set'])])
		c.writerow([self.const_hex(self.cal_dict['gainComparator']['Vref3DAC'])])
		c.writerow([self.const_hex(self.cal_dict['gainComparator']['Vref3Set'])])

		if 'VCMDAC' in list(self.cal_dict['gainComparator'].keys()):
			c.writerow([self.const_hex(self.cal_dict['gainComparator']['VCMDAC'])])
		else:
			c.writerow([self.const_hex(0x0000)])

		c.writerow([self.const_hex(self.cal_dict['gainComparator']['VCMSet'])])

		c.writerow([self.const_hex(self.cal_dict['inverseGlobalADCGain'][0])])
		c.writerow([self.const_hex(self.cal_dict['inverseGlobalADCGain'][1])])
		c.writerow([self.const_hex(self.cal_dict['inverseGlobalADCGain'][2])])
		c.writerow([self.const_hex(self.cal_dict['inverseGlobalADCGain'][3])])

		tmp = self.cal_dict['ramDataGain']
		for i in range(len(tmp)):
			c.writerow([self.const_hex(tmp[i])])

		tmp = self.cal_dict['ramDataOffset']
		for i in range(len(tmp)):
			c.writerow([self.const_hex(tmp[i])])



		c = csv.writer(open(os.path.join('_tempCalFiles', 'GO_cal_dict.csv'), 'w', newline=''))

		c.writerow([self.const_hex(self.cal_dict['ADC']['updnoffset'])])
		c.writerow([self.const_hex(self.cal_dict['ADC']['iramp'])])

		c.writerow([self.const_hex(self.cal_dict['gainComparator']['Vref1DAC'])])
		c.writerow([self.const_hex(self.cal_dict['gainComparator']['Vref1Set'])])
		c.writerow([self.const_hex(self.cal_dict['gainComparator']['Vref2DAC'])])
		c.writerow([self.const_hex(self.cal_dict['gainComparator']['Vref2Set'])])
		c.writerow([self.const_hex(self.cal_dict['gainComparator']['Vref3DAC'])])
		c.writerow([self.const_hex(self.cal_dict['gainComparator']['Vref3Set'])])

		if 'VCMDAC' in list(self.cal_dict['gainComparator'].keys()):
			c.writerow([self.const_hex(self.cal_dict['gainComparator']['VCMDAC'])])
		else:
			c.writerow([self.const_hex(0x0000)])

		c.writerow([self.const_hex(self.cal_dict['gainComparator']['VCMSet'])])

		c.writerow([self.const_hex(self.cal_dict['inverseGlobalADCGain'][0])])
		c.writerow([self.const_hex(self.cal_dict['inverseGlobalADCGain'][1])])
		c.writerow([self.const_hex(self.cal_dict['inverseGlobalADCGain'][2])])
		c.writerow([self.const_hex(self.cal_dict['inverseGlobalADCGain'][3])])

		tmp = self.cal_dict['ramDataGain']
		for i in range(len(tmp)):
			c.writerow([self.const_hex(tmp[i])])

		tmp = self.cal_dict['ramDataOffset']
		for i in range(len(tmp)):
			c.writerow([self.const_hex(tmp[i])])

		print("\nGAIN AND OFFSET CALIBRATION COMPLETED!")
		logging.info("\nGAIN AND OFFSET CALIBRATION COMPLETED!")
		
		return self.cal_dict, self.info

	
	def ADCCalib(self, ADC_dict):
		'''
		Performs ADC calibration
		
		Inputs:
		  ADC_dict              - input dictionary.
		  
		Outputs:
		  calib_iramp           - ADC ramp.
		  adc_delay             - ADC delay
		
		'''
		self.API.testBoardADC()
		print("\n-------ADC Calibration-------\n")
		logging.info("\n-------ADC Calibration-------\n")
		self.origSetting['reg'] = dict()
		self.origSetting['reg']['addr'] = np.array([],dtype=np.uint16)
		self.origSetting['reg']['val'] = np.array([],dtype=np.uint16)

		sensormodeID = ADC_dict['sensormodeID'] # passive IR
		nFrames = ADC_dict['nFrames']
		DAC_AMPTEST0 = ADC_dict['DAC_AMPTEST0']
		DAC_AMPTEST1 = ADC_dict['DAC_AMPTEST1']
		iramp = ADC_dict['iramp']
		ADC_FS = ADC_dict['ADC_FS']
		Cin = ADC_dict['Cin']
		Cfb = ADC_dict['Cfb']
		AmpGainMeas = ADC_dict['AmpGainMeas']
		cleanup = ADC_dict['cleanup']


		val = self.API.regread(0x0D1A)
		# self.API.error_assert(success, "Failed to determine 9b/10b mode")

		tenBitMode = bool(not self.API.bitget(int(val),3)) # get bit in 3rd position from LSB (4 in MATLAB)

	# 	#######################
	# 	## iramp calibration ##
	# 	#######################

	# 	# 1->Raw output (Direct copy from frame buffer, linear ADC + GT)
		status, origDrainStyleVal = self.API.UpdateUseCaseFrameSetting(ADC_dict['sensormodeID'], 'DRAINSTYLE', 1)
		self.API.error_assert(status, "Unable to set drainstyle")

		# Initialize outputs
		# pre-setup

		origDAC_AMPTEST0 = self.API.dac_amptest0_get()
		origDAC_AMPTEST1 = self.API.dac_amptest1_get()
		origDAC_CMREF = self.API.dac_cmref_get()

		success = self.API.dac_amptest0_set(ADC_dict['DAC_AMPTEST0'])
		self.API.error_assert(success, "Cannot set dac_amptest0")
		success = self.API.dac_amptest1_set(ADC_dict['DAC_AMPTEST1'])
		self.API.error_assert(success, "Cannot set dac_amptest1")
		success = self.API.dac_cmref_set(255)
		self.API.error_assert(success, "Cannot set dac_cmref")

		success = self.API.tregwrite(0x010E, self.API.repmatdec2Hex(ADC_dict['Cin'], 4)) # '1111'
		self.API.error_assert(success, "Cannot write to reg 0x010E Cin")
		success = self.API.tregwrite(0x010C, self.API.repmatdec2Hex(ADC_dict['Cfb'], 4)) # 'DDDD'
		self.API.error_assert(success, "Cannot write to reg 0x010C Cfb")

		success = self.API.tregwrite(0x0160, 0x0012) # ana_test_mux = test_sfp (amp_testn_sf)/test_sfn (amp_testp_sf)
		self.API.error_assert(success, "Cannot set register 0x0160")
		success = self.API.tregwrite(0x0D1A, 0x0004, 0x0007) # enable overflow, disable saturation encoding
		self.API.error_assert(success, "Cannot set register 0x0D1A")

		success, amp_mux_sel = self.API.regreadburst(0x0E1C, 10, 'increment') # save amp_mux_sel register values before overwriting with wave WAVE_FUNCTION_0
		self.API.error_assert(success, "Unable to read original AMP mux sel register values")

		for k in range(len(amp_mux_sel)):
			self.origSetting['reg']['addr'] = np.append(self.origSetting['reg']['addr'], 0x0E1C + k*2)
			self.origSetting['reg']['val'] = np.append(self.origSetting['reg']['val'], amp_mux_sel[k])

		self.origSetting['reg']['addr'] = np.append(self.origSetting['reg']['addr'], 0x0234)
		temp = self.API.regread(0x0234) # Read original REG_LP_CTRL register value
		self.origSetting['reg']['val'] = np.append(self.origSetting['reg']['val'], temp)

		success, origSettingWaveFunction0 = self.API.GetIndirectRegister('WAVE_FUNCTION_0')
		self.API.error_assert(success, 'Unable to read original WAVE_FUNCTION_0 register value')

		#success = self.API.SetIndirectRegister('WAVE_FUNCTION_0', 25)
		s = self.API.regwrite(0x0E1C, 0x0200)
		s = self.API.regwrite(0x0E1E, 0x0008)
		s = self.API.regwrite(0x0E20, 0x0200)
		s = self.API.regwrite(0x0E22, 0x0008)
		s = self.API.regwrite(0x0E24, 0x0200)
		s = self.API.regwrite(0x0E26, 0x0008)
		s = self.API.regwrite(0x0E28, 0x0200)
		s = self.API.regwrite(0x0E2A, 0x0008)
		s = self.API.regwrite(0x0E2C, 0x0200)
		s = self.API.regwrite(0x0E2E, 0x0008)
		s = self.API.regwrite(0x0234, 0xFDFF)


		s = self.API.regrmw(0x0E18, 0x0000 , 0x7C20) # <><><><><><><>     

		self.API.error_assert(success, 'Cannot write amp_mux registers')

		activeDeSignals = ['amp_clk0', 'amp_clk1', 'cmfb_clk', 'adc_in0', 'adc_in1',
							'adc_capsel', 'adc_rampdir', 'adc_ramppause', 'adc_data_valid']

		DE_StructOrig = self.API.DumpTimingTool('ReadfromChip')
		f = open(os.path.join(self.folderName, 'DE_StructOrig.pkl'),'wb')
		pickle.dump(DE_StructOrig, f)
		f.close()

		DE_Struct = self.API.DumpTimingTransformation(DE_StructOrig, 'native_resolution',activeDeSignals,'copyToSub_Sampled_2x')
		DE_Struct = self.API.DumpTimingTransformation(DE_Struct,'sub_sampled_2x',[],'ampTestSel_dacCMREF_dacCMREF_dacAmpTest0_dacAmpTest1')
		DE_Struct = self.API.DumpTimingTransformation(DE_Struct, 'native_resolution',activeDeSignals,'copyToSub_Sampled_4x')
		DE_Struct = self.API.DumpTimingTransformation(DE_Struct,'sub_sampled_4x',[],'ampTestSel_dacCMREF_dacCMREF_dacAmpTest1_dacAmpTest0')
		self.API.DumpTimingTool('WriteToChip', 0, DE_Struct)

		# self.API.legacy_ADC_calib_dump_timings() # alternate matlab implementation

		DE_StartEnd = np.zeros([2,2], dtype=np.uint16)
		success, DE_StartEnd[:,0] = self.API.regreadburst(0x0E36, 2,'increment')
		#DE_StartEnd[:,0] = tmp[0] # returned as a 2d array
		self.API.error_assert(success, 'Failed to read native DE values')
		success, DE_StartEnd[:,1]= self.API.regreadburst(0x0E3C, 2,'increment')
		#DE_StartEnd[:,1] = tmp[0]
		self.API.error_assert(success, 'Failed to read 2x subsampled DE values')

		h = self.API.height
		w = self.API.width

		img = np.zeros([h ,w ,nFrames, 2],dtype=np.double)
		delta_img = np.zeros([h,w,nFrames,len(iramp)],dtype=np.double)
		mean_delta = np.zeros(len(iramp))

		self.origSetting['reg']['addr'] = np.append(self.origSetting['reg']['addr'], 0x0E30)
		temp = self.API.regread(0x0E30)
		self.origSetting['reg']['val'] = np.append(self.origSetting['reg']['val'], temp)

		self.origSetting['reg']['addr'] = np.append(self.origSetting['reg']['addr'], 0x0E32)
		temp = self.API.regread(0x0E32)
		self.origSetting['reg']['val'] = np.append(self.origSetting['reg']['val'], temp)

		for y in range(len(iramp)-1, -1,-1): # 3 2 1 0
		#for y in range(4): # 3 2 1 0
			success = self.API.regwrite(0x0104, int(iramp[y]))
			self.API.error_assert(success, 'Cannot write to reg 0x0104 ADC iramp control')

			for x in range(1,-1,-1): # 1,0
				success = self.API.tregwrite(0x0014, 0x0000)
				self.API.error_assert(success, 'Failed to unclock gate DE') # blindly unclock gate all digital
				#arr_tmp = self.API.convert2matlabArray(DE_StartEnd[:,x])
				success = self.API.regwriteburst(0x0E30, DE_StartEnd[:,x], 'increment') 
				# overwrite "native_resolution" start and end address with either "sub_sampled_2x" or "sub_sampled_4x"
				self.API.error_assert(success, 'Failed to write native DE values')
				status, img_tmp = self.API.GetFramesADCCalib(nFrames)
				self.API.error_assert(status, 'Cannot get frames for ADC iramp calibration')
				for i in range(nFrames):
					img[:,:,i,x] = np.bitwise_and(np.array(img_tmp[:,:,i], dtype=np.uint16), 1023) # keep lower 10 bits (ADC data)
					img[:,:,i,x] = np.double(img[:,:,i,x])
					tmp = ~np.isnan(img[:,:,i,x])
					tmp[tmp] = img[:,:,i,x][tmp] < 3 # equivalent of tmp = img < 3
					img[:,:,i,x][tmp] = np.nan
					img[0:2,:,i,x] = np.nan # ignore first 2 rows
					
			delta_img[:,:,:,y] = img[:,:,:,1] - img[:,:,:,0]
			mean_delta[y] = np.nanmean(delta_img[:,:,:,y])
			# delta_img[:,:,:,y][delta_img[:,:,:,y] < 0.5*mean_delta[y]] = np.nan
			# mean_delta[y] = np.nanmean(delta_img[:,:,:,y])
			print('Mean ADC value:', np.nanmean(delta_img[:,:,:,y]))
			logging.info('Mean ADC value:' + str(np.nanmean(delta_img[:,:,:,y])))
			print('Maximum ADC value:', np.nanmax(delta_img[:,:,:,y]))
			logging.info('Minimum ADC value:' + str(np.nanmax(delta_img[:,:,:,y])))
			print('Minimum ADC value:', np.nanmin(delta_img[:,:,:,y]))
			logging.info('Maximum ADC value:' + str(np.nanmin(delta_img[:,:,:,y])))


		# status, tetetet = self.API.GetFramesADCCalib(nFrames)
		success = self.API.tregwrite(0x0014, 0x0000) # Work around

		b1 = self.API.CamMeasureAnaTestVoltage('DACwithSF_AMPTEST1-AMPTEST0')
		b2 = self.API.CamMeasureAnaTestVoltage('DACwithSF_AMPTEST0-AMPTEST1')
		# print(mean_delta)

		# mean_delta = np.array([255.48918137, 231.19768989, 211.06942671, 194.22303369])
		# b1 = 0.475
		# b2 = -0.4735

		Vin_ADC = (b1-b2)*AmpGainMeas
		targetCode = Vin_ADC * (2**(9 + tenBitMode) - 1) / ADC_FS

		if len(iramp) >= 3:
			p = np.polyfit(mean_delta, iramp, 2) # quadratic fit is preferred if we have multiple samples
		else:
			p = np.polyfit(mean_delta, iramp, 1) # else linear fit

		calib_iramp = np.round(np.polyval(p, targetCode),4)

		success = ((calib_iramp >= np.min(iramp)) and (calib_iramp <= np.max(iramp)))
		self.API.error_assert(success, 'ADCirampRangeExceeded')

		success = self.API.regwrite(0x0104, int(calib_iramp))
		print('Calibrated ADC iramp value:', calib_iramp)
		logging.info('Calibrated ADC iramp value:' + str(calib_iramp))

		############################
		## ADC offset calibration ##
		############################

		# Set adc_mux to Vcm
		success = self.API.tregwrite(0x0100, 0x0040, 0x0070)
		self.API.error_assert(success, 'Cannot Reg RMW adc_ctrl0_s1')
		success = self.API.tregwrite(0x0192, 0x0000, 0x0380) # bit[9] is inverted on camera C0 compared to B0 (adc_muxn_inb[2])
		self.API.error_assert(success, 'Cannot Reg RMW ana_serial_spare_2')

		# Read the default delays
		adc_delay = self.API.regread(0x0102) # ADC delay
		# self.API.error_assert(s, 'Cannot read reg 0x0102 - adc_delay')
		adc_delay = self.ADC_Delay_LFSR(adc_delay, reverse=True) # convert register LFSR to linear value

		status, VcmImg_pre = self.API.GetFramesADCCalib(nFrames)
		self.API.error_assert(status, 'Cannot grab frames for ADC offset calibration')

		# Extract 10 ADC bits
		VcmImg = VcmImg_pre
		VcmImg = np.array(VcmImg, dtype=np.double) % 1024 # keep lower 10 bits (ADC data)
		VcmImg[VcmImg < 3] = np.nan

		# plt.figure()
		# plt.imshow(VcmImg[800:1024,800:1024])
		# plt.title('VcmImg[800:1024,800:1024]')
		# plt.xlabel('Imager columns')
		# plt.ylabel('Imager rows')
		# plt.colorbar(label='ADC offset')
		# plt.show()

		# plt.plot(VcmImg[800:1024,933,:])
		# plt.title('VcmImg[900:1024,933]')
		# plt.show()

		# plt.plot(VcmImg[933,800:1024,:])
		# plt.title('VcmImg[933,900:1024]')
		# plt.show()


		# Compute new ADC delay value
		targetCenterVal = 2**(8 + tenBitMode) + 1
		targetCenterVal = targetCenterVal + (6 * 2**tenBitMode)

		adc_delay = np.nanmean(VcmImg) + adc_delay - targetCenterVal

		success = (adc_delay >= 3) and (adc_delay <= 511)
		self.API.error_assert(success, 'Calibrated linear ADC offset counter value is out of range [3, 511]')
		print('Calibrated ADC delay (linear value):', adc_delay)
		logging.info('Calibrated ADC delay (linear value):' + str(adc_delay))
		self.cal_dict['ADC']['linear_delay'] = adc_delay


		adc_delay = self.ADC_Delay_LFSR(np.round(adc_delay))

		success = self.API.regwrite(0x0102, int(adc_delay))
		print('Calibrated ADC up/dn delay control:', adc_delay)
		logging.info('Calibrated ADC up/dn delay control:' + str(adc_delay))

		success = success and self.API.regwrite(0x0014, 0x0000)
		if cleanup:
			self.API.DumpTimingTool('WriteToChip', 0, DE_StructOrig)
			#success = self.API.SetIndirectRegister('WAVE_FUNCTION_0', origSettingWaveFunction0)
			s = self.API.regwrite(0x0E1C, 0x0200)
			s = self.API.regwrite(0x0E1E, 0x0008)
			s = self.API.regwrite(0x0E20, 0x0200)
			s = self.API.regwrite(0x0E22, 0x0008)
			s = self.API.regwrite(0x0E24, 0x0200)
			s = self.API.regwrite(0x0E26, 0x0008)
			s = self.API.regwrite(0x0E28, 0x0200)
			s = self.API.regwrite(0x0E2A, 0x0008)
			s = self.API.regwrite(0x0E2C, 0x0200)
			s = self.API.regwrite(0x0E2E, 0x0008)
			s = self.API.regwrite(0x0234, 0xFDFF)

			#self.API.error_assert(success, 'Cannot restore WAVE_FUNCTION_0 register')
			self.API.WriteOrigSettings()
			status = self.API.UpdateUseCaseFrameSetting(ADC_dict['sensormodeID'], 'DRAINSTYLE', origDrainStyleVal)
			self.API.error_assert(status, 'Unable to reset drainstyle')
			#success = self.API.APIComm(1, self.API.API['main']['ioctrl'], self.API.API['IO']['sensor_mode']['set'], origSensorMode)
			self.API.error_assert(success, 'Cannot set original sensor mode')

			#success = success and self.API.N_APIComm(1, self.API.API['IO']['dac_amptest0']['set'], origDAC_AMPTEST0)
			success = self.API.dac_amptest0_set(np.uint8(origDAC_AMPTEST0))
			self.API.error_assert(success, 'Cannot set original DAC_AMPTEST0')
			#success = success and self.API.N_APIComm(1, self.API.API['IO']['dac_amptest1']['set'], origDAC_AMPTEST1)
			success = self.API.dac_amptest1_set(np.uint8(origDAC_AMPTEST1))
			self.API.error_assert(success, 'Cannot set original DAC_AMPTEST1')
			#success = success and self.API.N_APIComm(1, self.API.API['IO']['dac_cmref']['set'], origDAC_CMREF)
			success = self.API.dac_cmref_set(origDAC_CMREF)
			self.API.error_assert(success, 'Cannot set original DAC_CMREF')



		return calib_iramp, adc_delay


	def ComparatorDACCal(self, compDict):
		'''
		Performs Comparator DAC calibration
		
		Inputs:
		  compDict              - input dictionary.
		  
		Outputs:
		  compDAC               - array of calibrated comparator DAC values.
		  info                  - dictionary containing information regarding
		                          the calibration
		
		'''


		self.API.testBoardADC()
		s = self.API.regwrite(0x0014,0x0000)
		print("\n-------Comparator DAC Calibration-------\n")
		logging.info("\n-------Comparator DAC Calibration-------\n")
		NFRAMES = compDict['NFRAMES']
		doplot = compDict['doplot']
		compCalThres = compDict['compCalThres']
		compCalSatLimit = compDict['compCalSatLimit']
		compCalMargin = compDict['compCalMargin']

		#self.API.regwrite(0x012E,0x0406)
		#self.API.regwrite(0x0130,0x1F0D)
		self.API.regwrite(0x012E,0x0404)
		self.API.regwrite(0x0130,0x1F0C)
		#self.API.regwrite(0x012E,0x050A)
		#self.API.regwrite(0x0130, 0x1F0F)
		#self.API.regwrite(0x0130, 0x1F0F)
		# self.API.regwrite(0x012E,0x1F1F) # Work around, having a high value will result in saturation -> 0204

		sensormodeID = 3 # Use passive IR
		ADC_FS = 1 # ADC full scale = +/- 1.00V
		DACSweepFineResolution = [5e-3, 5e-3, 10e-3] # Fine target resolution DAC sweep = 5mV for two higher gains, 10mV for lowest gain
		DACSweepCoarseResolution = [10e-3, 10e-3, 20e-3] # Coarse resolution DAC sweep = 10mV for two higher gains, 20mV for lowest gain
		ampgain = self.API.ReadAmpGainSetting() # estimate for expected amp gains

		print('ampgain:', ampgain)
		logging.info('ampgain:' + str(ampgain))

		val = self.API.regread(0x0D1A) # determine if we are using 9b or 10b data
		# 	self.API.error_assert(success, "Failed to determine 9b/10b mode")
		tenBitMode = bool(not self.API.bitget(int(val),3))

		compDacOrig = np.zeros(3, dtype='int')

		val = self.API.regread(0x012E)

		# 	self.API.error_assert(success, 'Failed to read comparator DAC register')
		compDacOrig[0] = int(val) & int('0x00FF', 0)
		compDacOrig[1] = (int(val) & int('0xFF00', 0)) >> 8
		val = self.API.regread(0x0130)
		# 	self.API.error_assert(success, 'Failed to read DAC register')
		compDacOrig[2] = int(val) & int('0x00FF', 0)

		# 1->Raw output (Direct copy from frame buffer, linear ADC + GT)
		status = self.API.UpdateUseCaseFrameSetting(sensormodeID, 'DRAINSTYLE', 1)

		self.API.error_assert(status, 'Unable to set drain style')
		s = self.API.regwrite(0x0014, 0x0000)
		# success = self.API.SetIndirectRegister('WAVE_FUNCTION_0', 25)
		# self.API.error_assert(success, 'Cannot write amp_mux registers')
		s = self.API.regwrite(0x0E1C, 0x0200)
		s = self.API.regwrite(0x0E1E, 0x0008)
		s = self.API.regwrite(0x0E20, 0x0200)
		s = self.API.regwrite(0x0E22, 0x0008)
		s = self.API.regwrite(0x0E24, 0x0200)
		s = self.API.regwrite(0x0E26, 0x0008)
		s = self.API.regwrite(0x0E28, 0x0200)
		s = self.API.regwrite(0x0E2A, 0x0008)
		s = self.API.regwrite(0x0E2C, 0x0200)
		s = self.API.regwrite(0x0E2E, 0x0008)
		s = self.API.regwrite(0x0234, 0xFDFF)


		success = self.API.regrmw(0x012A, 0x0000, 0x7E00 ) # clear comp_ref_disable[5:0] to enable multigain
		self.API.error_assert(success, 'Failed to enable all gain comparators')

	
		DE_Struct = self.API.DumpTimingTool('ReadfromChip')

		DacCal = dict()
		CMwindow = [2.10, 2.15] # Define the Common Mode range of the DACs (post source follower) for both the "reset" and "integrate" samples
		DacSel = ['CMREF-AMPTEST0','CMREF-AMPTEST0','AMPTEST1-AMPTEST0'] # Define which pair of DACs to use for each of the 3 gain comparators
		# CMREF-AMPTEST0 provides more resolution (since CMREF step size is smaller), while AMPTEST1-AMPTEST0 provides more range
		forceCompDac = [0x2800, 0x2400, 0x1400]

		dacDiffVoltagePost = np.ones([2,3]) * np.nan
		dacDiffVoltagePre = np.ones([2,3]) * np.nan # variable to store pre/post transition voltages. dimensions [positive/negative comparator, gain comparator 0-2]

		# initial guess for comparator voltage is 90% of expected ADC full scale
		# quantized to target sweep resolution
		dacDiffVoltagePost[0,:] = np.round(0.9*ADC_FS/ampgain[0:3]/DACSweepFineResolution) * DACSweepFineResolution

		# define an upper limit of 120% of expected ADC full scale
		dacDiffVoltageLimit = np.ceil(1.2*ADC_FS/ampgain[0:3]/DACSweepFineResolution) * DACSweepFineResolution

		GT_pos = np.zeros(3,dtype=object)
		ADC_pos = np.zeros(3,dtype=object)
		SAT_pos = np.zeros(3,dtype=object)
		GT_neg = np.zeros(3,dtype=object)
		ADC_neg = np.zeros(3,dtype=object)
		SAT_neg = np.zeros(3,dtype=object)


		for gainTransition in range(2, -1,-1): # 2,1,0
			success = self.API.regrmw(0x012A, forceCompDac[gainTransition], 0x7E00)
			self.API.error_assert(success,'Unable to disable comparators that are not currently under test')

			## Testing positive comparator

			print('\n---Gain', gainTransition, '---')
			logging.info('\n---Gain ' + str(gainTransition) + ' ---')
			print('\nTesting positive comparator\n')
			logging.info('\nTesting positive comparator\n')
			if DacSel[gainTransition].upper() == 'CMREF-AMPTEST0':
				tmp = self.API.DumpTimingTransformation(DE_Struct,'native_resolution',[],'ampTestSel_dacAmpTest1_dacAmpTest1_dacCMREF_dacAmpTest0')
				self.API.DumpTimingTool('WriteToChip', 0, tmp)
			elif DacSel[gainTransition].upper() == 'AMPTEST1-AMPTEST0':
				tmp = self.API.DumpTimingTransformation(DE_Struct,'native_resolution',[],'ampTestSel_dacCMREF_dacCMREF_dacAmpTest1_dacAmpTest0')
				self.API.DumpTimingTool('WriteToChip', 0, tmp)
			else:
				raise Exception("Incorrect DacSel value")
				logging.error('Incorrect DacSel value')
			
			print("Setting", DacSel[gainTransition], "to target: ", str(dacDiffVoltagePost[0,gainTransition]*1e3) + "mV")
			logging.info("Setting " + str(DacSel[gainTransition]) + " to target: " + str(dacDiffVoltagePost[0,gainTransition]*1e3) + "mV")


			DacCal, _, _ = self.API.SetDiffDacSfVoltage(DacCal,DacSel[gainTransition],dacDiffVoltagePost[0,gainTransition],CMwindow)


			# print(DacCal['RangeCMREF'], DacCal['RangeAMPTEST0'], DacCal['RangeAMPTEST1'])
			GT_tmp, ADC_tmp, SAT_tmp = self.API.GetFramesCompDACalib(NFRAMES)

			gainTagTransitionPercentPerCol = np.sum(np.reshape(GT_tmp.transpose(1,0,2,3), (list(np.shape(GT_tmp))[1], -1))==(gainTransition+1),1) / list(np.shape(GT_tmp))[0] * 100
			print('Gain tag measured over entire sensor: %8.4f (column highest: %6.2f, column lowest: %6.2f)' 
				  %(np.sum(GT_tmp.flatten() == (gainTransition+1))/np.size(GT_tmp)*100, np.max(gainTagTransitionPercentPerCol), np.min(gainTagTransitionPercentPerCol)))
			logging.info(str('Gain tag measured over entire sensor: %8.4f (column highest: %6.2f, column lowest: %6.2f)' 
				  %(np.sum(GT_tmp.flatten() == (gainTransition+1))/np.size(GT_tmp)*100, np.max(gainTagTransitionPercentPerCol), np.min(gainTagTransitionPercentPerCol))))
			
			# while any(vec(GT_tmp)<gainTransition)  --> while not all pixels transitioned to the desired gain setting, increase voltage
			while(np.any(np.mean(np.reshape(GT_tmp.transpose(1,0,2,3), (list(np.shape(ADC_tmp))[1], -1)),1) / (gainTransition+1) < compCalThres)):

				print('Coarse search l1 any', np.min(np.mean(np.reshape(GT_tmp.transpose(1,0,2,3), (list(np.shape(ADC_tmp))[1], -1)),1) / (gainTransition+1)), '< Thresh', compCalThres)

				GT_pos[gainTransition] = np.reshape(np.transpose(GT_tmp,[1,0,2,3]),[np.shape(GT_tmp)[1],-1]) # store previous measurement at pre-transition voltage
				ADC_pos[gainTransition] = np.reshape(np.transpose(ADC_tmp,[1,0,2,3]),[np.shape(ADC_tmp)[1],-1])
				SAT_pos[gainTransition] = np.reshape(np.transpose(SAT_tmp,[1,0,2,3]),[np.shape(SAT_tmp,)[1],-1])
				dacDiffVoltagePre[0, gainTransition] = dacDiffVoltagePost[0, gainTransition]

				dacDiffVoltagePost[0, gainTransition] = dacDiffVoltagePost[0,gainTransition] + DACSweepCoarseResolution[gainTransition] # increase voltage by DACSweepCoarseResolution
				dacDiffVoltagePost[0, gainTransition] = (np.round(dacDiffVoltagePost[0, gainTransition] / DACSweepFineResolution[gainTransition])
														*DACSweepFineResolution[gainTransition]) # quantize to target sweep resolution

				success = np.abs(dacDiffVoltagePost[0,gainTransition]) < dacDiffVoltageLimit[gainTransition]
				self.API.error_assert(success, 'comparator DAC voltage exceeded')

				print('Setting %18s to target: %3.0fmV' %(DacSel[gainTransition],dacDiffVoltagePost[0,gainTransition]*1e3))
				logging.info(str('Setting %18s to target: %3.0fmV' %(DacSel[gainTransition],dacDiffVoltagePost[0,gainTransition]*1e3)))
				_, _, _ = self.API.SetDiffDacSfVoltage(DacCal,DacSel[gainTransition],dacDiffVoltagePost[0,gainTransition],CMwindow)
				GT_tmp, ADC_tmp, SAT_tmp = self.API.GetFramesCompDACalib(NFRAMES)

				gainTagTransitionPercentPerCol = np.sum(np.reshape(GT_tmp.transpose(1,0,2,3), (list(np.shape(GT_tmp))[1], -1))==(gainTransition+1),1) / list(np.shape(GT_tmp))[0] * 100
				print('Gain tag measured over entire sensor: %8.4f (column highest: %6.2f, column lowest: %6.2f)' 
				  	%(np.sum(GT_tmp.flatten() == (gainTransition+1))/np.size(GT_tmp)*100, np.max(gainTagTransitionPercentPerCol), np.min(gainTagTransitionPercentPerCol)))
				logging.info(str('Gain tag measured over entire sensor: %8.4f (column highest: %6.2f, column lowest: %6.2f)' 
				  	%(np.sum(GT_tmp.flatten() == (gainTransition+1))/np.size(GT_tmp)*100, np.max(gainTagTransitionPercentPerCol), np.min(gainTagTransitionPercentPerCol))))
			
			if np.isnan(dacDiffVoltagePre[0,gainTransition]): # we only found high value, so need to search for an inital low value
				dacDiffVoltagePre[0,gainTransition] = dacDiffVoltagePost[0,gainTransition]
				while(np.all(((np.mean(np.reshape(GT_tmp.transpose(1,0,2,3), [np.shape(ADC_tmp)[1], -1]),1) / (gainTransition +1)) >= compCalThres))):

					print('Coarse search l1 nan all', np.min(np.mean(np.reshape(GT_tmp.transpose(1,0,2,3), [np.shape(ADC_tmp)[1], -1]),1) / (gainTransition +1)), '>= Thresh', compCalThres)

					dacDiffVoltagePost[0,gainTransition] = dacDiffVoltagePre[0,gainTransition]
					dacDiffVoltagePre[0, gainTransition] = dacDiffVoltagePre[0,gainTransition] - DACSweepCoarseResolution[gainTransition] # decrease voltage by DACSweepCoarseResolution
					dacDiffVoltagePre[0, gainTransition] = (np.round(dacDiffVoltagePre[0, gainTransition] / DACSweepFineResolution[gainTransition])
															*DACSweepFineResolution[gainTransition]) # quantize to target sweep resolution

					success = np.abs(dacDiffVoltagePost[0,gainTransition]) < dacDiffVoltageLimit[gainTransition]
					self.API.error_assert(success, 'comparator DAC voltage exceeded')

					print('Setting %18s to target: %3.0fmV' %(DacSel[gainTransition],dacDiffVoltagePre[0,gainTransition]*1e3))
					logging.info(str('Setting %18s to target: %3.0fmV' %(DacSel[gainTransition],dacDiffVoltagePre[0,gainTransition]*1e3)))
					_, _, _ = self.API.SetDiffDacSfVoltage(DacCal,DacSel[gainTransition],dacDiffVoltagePre[0,gainTransition],CMwindow)
					
					GT_tmp, ADC_tmp, SAT_tmp = self.API.GetFramesCompDACalib(NFRAMES)
					gainTagTransitionPercentPerCol = np.sum(np.reshape(GT_tmp.transpose(1,0,2,3), (list(np.shape(GT_tmp))[1], -1))==(gainTransition+1),1) / list(np.shape(GT_tmp))[0] * 100
					print('Gain tag measured over entire sensor: %8.4f (column highest: %6.2f, column lowest: %6.2f)' 
				  		%(np.sum(GT_tmp.flatten() == (gainTransition+1))/np.size(GT_tmp)*100, np.max(gainTagTransitionPercentPerCol), np.min(gainTagTransitionPercentPerCol)))
					logging.info(str('Gain tag measured over entire sensor: %8.4f (column highest: %6.2f, column lowest: %6.2f)' 
				  		%(np.sum(GT_tmp.flatten() == (gainTransition+1))/np.size(GT_tmp)*100, np.max(gainTagTransitionPercentPerCol), np.min(gainTagTransitionPercentPerCol))))

				GT_pos[gainTransition] = np.reshape(np.transpose(GT_tmp,[1,0,2,3]),[np.shape(GT_tmp)[1],-1])
				ADC_pos[gainTransition] = np.reshape(np.transpose(ADC_tmp,[1,0,2,3]),[np.shape(ADC_tmp)[1],-1])
				SAT_pos[gainTransition] = np.reshape(np.transpose(SAT_tmp,[1,0,2,3]),[np.shape(SAT_tmp,)[1],-1])

			# Binary fine search
			while(np.round(np.abs(dacDiffVoltagePost[0,gainTransition] - dacDiffVoltagePre[0,gainTransition]) / DACSweepFineResolution[gainTransition]) > 1):
				# perform fine search ('round' required to prevent rounding errors in subtraction)
				dacDiffVoltageTest = np.mean([dacDiffVoltagePost[0,gainTransition],dacDiffVoltagePre[0,gainTransition]])
				dacDiffVoltageTest = np.round(dacDiffVoltageTest / DACSweepFineResolution[gainTransition]) * DACSweepFineResolution[gainTransition]

				success = np.abs(dacDiffVoltagePost[0,gainTransition]) < dacDiffVoltageLimit[gainTransition]
				self.API.error_assert(success, 'comparator DAC voltage exceeded')

				print('Setting %18s to target: %3.0fmV' %(DacSel[gainTransition],dacDiffVoltageTest*1e3))
				logging.info(str('Setting %18s to target: %3.0fmV' %(DacSel[gainTransition],dacDiffVoltageTest*1e3)))
				_, _, _ = self.API.SetDiffDacSfVoltage(DacCal,DacSel[gainTransition],dacDiffVoltageTest,CMwindow)

				GT_tmp, ADC_tmp, SAT_tmp = self.API.GetFramesCompDACalib(NFRAMES)
				gainTagTransitionPercentPerCol = np.sum(np.reshape(GT_tmp.transpose(1,0,2,3), (list(np.shape(GT_tmp))[1], -1))==(gainTransition+1),1) / list(np.shape(GT_tmp))[0] * 100
				print('Gain tag measured over entire sensor: %8.4f (column highest: %6.2f, column lowest: %6.2f)' 
				  	%(np.sum(GT_tmp.flatten() == (gainTransition+1))/np.size(GT_tmp)*100, np.max(gainTagTransitionPercentPerCol), np.min(gainTagTransitionPercentPerCol)))
				logging.info(str('Gain tag measured over entire sensor: %8.4f (column highest: %6.2f, column lowest: %6.2f)' 
				  	%(np.sum(GT_tmp.flatten() == (gainTransition+1))/np.size(GT_tmp)*100, np.max(gainTagTransitionPercentPerCol), np.min(gainTagTransitionPercentPerCol))))

				if (np.any(((np.mean(np.reshape(GT_tmp.transpose(1,0,2,3), [np.shape(ADC_tmp)[1], -1]),1) / (gainTransition +1)) < compCalThres))):
					GT_pos[gainTransition] = np.reshape(np.transpose(GT_tmp,[1,0,2,3]),[np.shape(GT_tmp)[1],-1])
					ADC_pos[gainTransition] = np.reshape(np.transpose(ADC_tmp,[1,0,2,3]),[np.shape(ADC_tmp)[1],-1])
					SAT_pos[gainTransition] = np.reshape(np.transpose(SAT_tmp,[1,0,2,3]),[np.shape(SAT_tmp,)[1],-1])
					dacDiffVoltagePre[0,gainTransition] = dacDiffVoltageTest
				else:
					dacDiffVoltagePost[0,gainTransition] = dacDiffVoltageTest

			## Find transition point for negative comparator
			print('\nTesting negative comparator\n')
			logging.info('\nTesting negative comparator\n')
			dacDiffVoltagePost[1, gainTransition] = -1* dacDiffVoltagePost[0, gainTransition]
			print('Setting %18s to target: %3.0fmV' %(DacSel[gainTransition],dacDiffVoltagePost[1,gainTransition]*1e3))
			logging.info(str('Setting %18s to target: %3.0fmV' %(DacSel[gainTransition],dacDiffVoltagePost[1,gainTransition]*1e3)))
			_, _, _ = self.API.SetDiffDacSfVoltage(DacCal,DacSel[gainTransition],dacDiffVoltagePost[1,gainTransition],CMwindow)
			GT_tmp, ADC_tmp, SAT_tmp = self.API.GetFramesCompDACalib(NFRAMES)

			gainTagTransitionPercentPerCol = np.sum(np.reshape(GT_tmp.transpose(1,0,2,3), (list(np.shape(GT_tmp))[1], -1))==(gainTransition+1),1) / list(np.shape(GT_tmp))[0] * 100
			print('Gain tag measured over entire sensor: %8.4f (column highest: %6.2f, column lowest: %6.2f)' 
				  %(np.sum(GT_tmp.flatten() == (gainTransition+1))/np.size(GT_tmp)*100, np.max(gainTagTransitionPercentPerCol), np.min(gainTagTransitionPercentPerCol)))
			logging.info(str('Gain tag measured over entire sensor: %8.4f (column highest: %6.2f, column lowest: %6.2f)' 
				  %(np.sum(GT_tmp.flatten() == (gainTransition+1))/np.size(GT_tmp)*100, np.max(gainTagTransitionPercentPerCol), np.min(gainTagTransitionPercentPerCol))))
		
			while(np.any(np.mean(np.reshape(GT_tmp.transpose(1,0,2,3), (list(np.shape(ADC_tmp))[1], -1)),1) / (gainTransition+1) < compCalThres)):
				GT_neg[gainTransition] = np.reshape(np.transpose(GT_tmp,[1,0,2,3]),[np.shape(GT_tmp)[1],-1]) # store previous measurement at pre-transition voltage
				ADC_neg[gainTransition] = np.reshape(np.transpose(ADC_tmp,[1,0,2,3]),[np.shape(ADC_tmp)[1],-1])
				SAT_neg[gainTransition] = np.reshape(np.transpose(SAT_tmp,[1,0,2,3]),[np.shape(SAT_tmp,)[1],-1])
				dacDiffVoltagePre[1, gainTransition] = dacDiffVoltagePost[1, gainTransition]

				dacDiffVoltagePost[1, gainTransition] = dacDiffVoltagePost[1,gainTransition] - DACSweepFineResolution[gainTransition] 
				# decrease voltage by DACSweepFineResolution (using fine search only since it should be similar to positive DAC)
				dacDiffVoltagePost[1, gainTransition] = (np.round(dacDiffVoltagePost[1, gainTransition] / DACSweepFineResolution[gainTransition])
														*DACSweepFineResolution[gainTransition]) # quantize to target sweep resolution


				success = np.abs(dacDiffVoltagePost[1,gainTransition]) < dacDiffVoltageLimit[gainTransition]
				self.API.error_assert(success, 'comparator DAC voltage exceeded')

				print('Setting %18s to target: %3.0fmV' %(DacSel[gainTransition],dacDiffVoltagePost[1,gainTransition]*1e3))
				logging.info(str('Setting %18s to target: %3.0fmV' %(DacSel[gainTransition],dacDiffVoltagePost[1,gainTransition]*1e3)))
				_, _, _ = self.API.SetDiffDacSfVoltage(DacCal,DacSel[gainTransition],dacDiffVoltagePost[1,gainTransition],CMwindow)
				GT_tmp, ADC_tmp, SAT_tmp = self.API.GetFramesCompDACalib(NFRAMES)

				gainTagTransitionPercentPerCol = np.sum(np.reshape(GT_tmp.transpose(1,0,2,3), (list(np.shape(GT_tmp))[1], -1))==(gainTransition+1),1) / list(np.shape(GT_tmp))[0] * 100
				print('Gain tag measured over entire sensor: %8.4f (column highest: %6.2f, column lowest: %6.2f)' 
				  	%(np.sum(GT_tmp.flatten() == (gainTransition+1))/np.size(GT_tmp)*100, np.max(gainTagTransitionPercentPerCol), np.min(gainTagTransitionPercentPerCol)))
				logging.info(str('Gain tag measured over entire sensor: %8.4f (column highest: %6.2f, column lowest: %6.2f)' 
				  	%(np.sum(GT_tmp.flatten() == (gainTransition+1))/np.size(GT_tmp)*100, np.max(gainTagTransitionPercentPerCol), np.min(gainTagTransitionPercentPerCol))))

			if (np.isnan(dacDiffVoltagePre[1,gainTransition])):
				dacDiffVoltagePre[1,gainTransition] = dacDiffVoltagePost[1,gainTransition] + DACSweepFineResolution[gainTransition] 
				# increase voltage by DACSweepFineResolution
				success = np.abs(dacDiffVoltagePost[1,gainTransition]) < dacDiffVoltageLimit[gainTransition]
				self.API.error_assert(success, 'comparator DAC voltage exceeded')
				print('Setting %18s to target: %3.0fmV' %(DacSel[gainTransition],dacDiffVoltagePre[1,gainTransition]*1e3))
				logging.info(str('Setting %18s to target: %3.0fmV' %(DacSel[gainTransition],dacDiffVoltagePre[1,gainTransition]*1e3)))
				_, _, _ = self.API.SetDiffDacSfVoltage(DacCal,DacSel[gainTransition],dacDiffVoltagePre[1,gainTransition],CMwindow)
				GT_tmp, ADC_tmp, SAT_tmp = self.API.GetFramesCompDACalib(NFRAMES)
				gainTagTransitionPercentPerCol = np.sum(np.reshape(GT_tmp.transpose(1,0,2,3), (list(np.shape(GT_tmp))[1], -1))==(gainTransition+1),1) / list(np.shape(GT_tmp))[0] * 100
				print('Gain tag measured over entire sensor: %8.4f (column highest: %6.2f, column lowest: %6.2f)' 
				  	%(np.sum(GT_tmp.flatten() == (gainTransition+1))/np.size(GT_tmp)*100, np.max(gainTagTransitionPercentPerCol), np.min(gainTagTransitionPercentPerCol)))
				logging.info(str('Gain tag measured over entire sensor: %8.4f (column highest: %6.2f, column lowest: %6.2f)' 
				  	%(np.sum(GT_tmp.flatten() == (gainTransition+1))/np.size(GT_tmp)*100, np.max(gainTagTransitionPercentPerCol), np.min(gainTagTransitionPercentPerCol))))

				while(np.all(((np.mean(np.reshape(GT_tmp.transpose(1,0,2,3), [np.shape(ADC_tmp)[1], -1]),1) / (gainTransition +1)) >= compCalThres))):
					dacDiffVoltagePost[1,gainTransition] = dacDiffVoltagePre[1,gainTransition] # store previous measurement as pre-transition voltage 
					dacDiffVoltagePre[1, gainTransition] = dacDiffVoltagePre[1,gainTransition] + DACSweepFineResolution[gainTransition] # increase voltage by DACSweepFIneResolution
					dacDiffVoltagePre[1, gainTransition] = (np.round(dacDiffVoltagePre[1, gainTransition] / DACSweepFineResolution[gainTransition])
															*DACSweepFineResolution[gainTransition]) # quantize to target sweep resolution
					
					success = np.abs(dacDiffVoltagePost[1,gainTransition]) < dacDiffVoltageLimit[gainTransition]
					self.API.error_assert(success, 'comparator DAC voltage exceeded')

					print('Setting %18s to target: %3.0fmV' %(DacSel[gainTransition],dacDiffVoltagePre[1,gainTransition]*1e3))
					logging.info(str('Setting %18s to target: %3.0fmV' %(DacSel[gainTransition],dacDiffVoltagePre[1,gainTransition]*1e3)))
					_, _, _ = self.API.SetDiffDacSfVoltage(DacCal,DacSel[gainTransition],dacDiffVoltagePre[1,gainTransition],CMwindow)
					
					GT_tmp, ADC_tmp, SAT_tmp = self.API.GetFramesCompDACalib(NFRAMES)
					gainTagTransitionPercentPerCol = np.sum(np.reshape(GT_tmp.transpose(1,0,2,3), (list(np.shape(GT_tmp))[1], -1))==(gainTransition+1),1) / list(np.shape(GT_tmp))[0] * 100
					print('Gain tag measured over entire sensor: %8.4f (column highest: %6.2f, column lowest: %6.2f)' 
				  		%(np.sum(GT_tmp.flatten() == (gainTransition+1))/np.size(GT_tmp)*100, np.max(gainTagTransitionPercentPerCol), np.min(gainTagTransitionPercentPerCol)))
					logging.info(str('Gain tag measured over entire sensor: %8.4f (column highest: %6.2f, column lowest: %6.2f)' 
				  		%(np.sum(GT_tmp.flatten() == (gainTransition+1))/np.size(GT_tmp)*100, np.max(gainTagTransitionPercentPerCol), np.min(gainTagTransitionPercentPerCol))))

				GT_neg[gainTransition] = np.reshape(np.transpose(GT_tmp,[1,0,2,3]),[np.shape(GT_tmp)[1],-1])
				ADC_neg[gainTransition] = np.reshape(np.transpose(ADC_tmp,[1,0,2,3]),[np.shape(ADC_tmp)[1],-1])
				SAT_neg[gainTransition] = np.reshape(np.transpose(SAT_tmp,[1,0,2,3]),[np.shape(SAT_tmp,)[1],-1])

			# print(dacDiffVoltagePre)
			# print(dacDiffVoltagePost)
		success = self.API.regrmw(0x012A, 0x0000, 0x7E00)
		self.API.error_assert(success, 'Unable to enable comparators for normal operation')

		f = open(os.path.join(self.folderName, 'DacCalibration.pkl'),'wb')
		pickle.dump(DacCal,f)
		f.close()


		maxADCperCol = np.zeros([3, np.shape(ADC_pos[0])[0]])
		minADCperCol = np.zeros([3, np.shape(ADC_neg[0])[0]])
		sum_SAT_pos = np.zeros([3, np.shape(SAT_pos[0])[0]])
		sum_SAT_neg = np.zeros([3, np.shape(SAT_neg[0])[0]])


		for i in range(3):
			maxADCperCol[i] = np.max(ADC_pos[i], axis = 1)
			minADCperCol[i] = np.min(ADC_neg[i], axis = 1)
			sum_SAT_pos[i] = np.sum(SAT_pos[i], axis = 1)
			sum_SAT_neg[i] = np.sum(SAT_neg[i], axis = 1)

		anySaturationDetected = np.any(np.append(sum_SAT_pos, sum_SAT_neg, axis=1), axis=1)

		if np.any(anySaturationDetected):
			if doplot:
				plt.figure()
				for i in range(3):
					plt.plot(range(0,1024), sum_SAT_pos[i] / np.shape(SAT_pos[i])[1], label = '(+) Gain '+ str(i))
					plt.plot(range(0,1024), sum_SAT_neg[i] / np.shape(SAT_neg[i])[1], label = '(-) Gain '+ str(i))
				plt.legend()
				plt.xlabel('Pixel col (0-1023)')
				plt.savefig(os.path.join(self.folderName, 'ComparatorSaturation.png'))

			mean_divisor = np.shape(SAT_pos[i])[1]
			perColSaturation = np.max(np.transpose([sum_SAT_pos/mean_divisor, sum_SAT_neg/mean_divisor]), axis = 2)
			numColsExceedSatLimit = np.sum(perColSaturation > compCalSatLimit, axis = 1)
			if np.any(numColsExceedSatLimit):
				plt.savefig(os.path.join(self.folderName, 'Saturation.png'))
				self.API.error_assert(False,'Saturation limit exceeded for comparator DAC calibration')
			else:
				print('\nWARNING! : Saturation detected but below limt\n')
				logging.warning('\nWARNING! : Saturation detected but below limt\n')


		MinMaxADCValueperCol = dict()
		MinMaxADCValueperCol['maxADCperCol'] = maxADCperCol
		MinMaxADCValueperCol['minADCperCol'] = minADCperCol
		f = open(os.path.join(self.folderName, 'MinMaxADCValueperCol.pkl'), 'wb')
		pickle.dump(MinMaxADCValueperCol, f)
		f.close()

		measADCmargin = np.array([np.min(minADCperCol,1) - 3, 2**(9+tenBitMode)-1 - np.max(maxADCperCol,1)]) / (2**(8+tenBitMode))
		measADCmargin = np.min(measADCmargin, axis = 0)
		measADCmargin[anySaturationDetected] = 0

		print('Comp cal: Min ADC val per gain (highest gain to lowest gain): %3.0f, %3.0f, %3.0f' %tuple(np.min(minADCperCol, axis = 1)))
		logging.info(str('Comp cal: Min ADC val per gain (highest gain to lowest gain): %3.0f, %3.0f, %3.0f' %tuple(np.min(minADCperCol, axis = 1))))
		print('Comp cal: Max ADC val per gain (highest gain to lowest gain): %3.0f, %3.0f, %3.0f' %tuple(np.max(maxADCperCol, axis = 1)))
		logging.info(str('Comp cal: Max ADC val per gain (highest gain to lowest gain): %3.0f, %3.0f, %3.0f' %tuple(np.max(maxADCperCol, axis = 1))))

		# Measure the slope of each comparator DAC.  TODO: Characterize compDac_mVperLSB over a number of units and
		# replace with a hardcoded value instead of measuring each camera chip individually
		# self.API.testBoardADC()
		dacLow = 0
		dacHigh = 63
		numVoltageSamples = 1

		v = np.zeros(2)
		compDac_mVperLSB = np.zeros(3)

		success = self.API.regrmw(0x012E, dacLow, 0x00FF)
		v[0] = self.API.CamMeasureAnaTestVoltage('comp_vref0',numVoltageSamples)
		count = 10
		while((abs(v[0]) < 0.01) and (abs(v[0]) > 3.00) and count > 1):
			time.sleep(1)
			v[0] = self.API.CamMeasureAnaTestVoltage('comp_vref0',numVoltageSamples)
			count = count -1
		success = success and self.API.regrmw(0x012E, dacHigh , 0x00FF)
		v[1] = self.API.CamMeasureAnaTestVoltage('comp_vref0',numVoltageSamples)
		count = 10
		while((abs(v[1]) < 0.01) and (abs(v[0]) > 3.00) and count > 1):
			time.sleep(1)
			v[1] = self.API.CamMeasureAnaTestVoltage('comp_vref0',numVoltageSamples)
			count = count -1
		compDac_mVperLSB[0] = (v[1]-v[0]) / (dacHigh-dacLow) * 1e3

		success = success and self.API.regrmw(0x012E, dacLow*256, 0xFF00)
		v[0] = self.API.CamMeasureAnaTestVoltage('comp_vref1',numVoltageSamples)
		success = success and self.API.regrmw(0x012E, dacHigh*256, 0xFF00)
		v[1] = self.API.CamMeasureAnaTestVoltage('comp_vref1',numVoltageSamples)
		compDac_mVperLSB[1] = (v[1]-v[0]) / (dacHigh-dacLow) * 1e3

		success = success and self.API.regrmw(0x0130, dacLow, 0x00FF)
		v[0] = self.API.CamMeasureAnaTestVoltage('comp_vref2',numVoltageSamples)
		success = success and self.API.regrmw(0x0130, dacHigh, 0x00FF)
		v[1] = self.API.CamMeasureAnaTestVoltage('comp_vref2',numVoltageSamples)
		compDac_mVperLSB[2] = (v[1]-v[0]) / (dacHigh-dacLow) * 1e3               # Measured mV/DAC code
		self.API.error_assert(success, 'Failed to measure comparator DAC voltages')

		estimatedADCperCompDACLSB = compDac_mVperLSB * 1e-3 * ampgain[0:3] / ADC_FS # estimate (normalized) ADC change per LSB of comparator DAC change

		compDacAdjusted = compDacOrig + np.floor((measADCmargin - np.array(compCalMargin)) / estimatedADCperCompDACLSB)

		compDac = np.clip(compDacAdjusted,0,63)

		if np.any(compDac != compDacAdjusted):
			print('WARNING: Comparator DAC clamped!')

		success = self.API.regwrite(0x012E, np.uint16(compDac[1]*256+compDac[0]))
		success = success and self.API.regrmw(0x0130, int(compDac[2]) , 0x003F)
		self.API.error_assert(success, 'Failed to write comparator DAC settings to chip')

		print('comp_ref_dac1: %.0f' %(compDac[0]))
		logging.info(str('comp_ref_dac1: %.0f' %(compDac[0])))
		print('comp_ref_dac2: %.0f' %(compDac[1]))
		logging.info(str('comp_ref_dac2: %.0f' %(compDac[1])))
		print('comp_ref_dac3: %.0f' %(compDac[2]))
		logging.info(str('comp_ref_dac3: %.0f' %(compDac[2])))

		#Adjust DAC diff voltage based on new comparator thresholds before measuring ADC margin 
		compDacAdjustment = compDacAdjusted - compDacOrig 
		dacDiffVoltagePre[0,] = dacDiffVoltagePre[0,] + (compDacAdjustment*compDac_mVperLSB*1e-3)
		dacDiffVoltagePre[1,] = dacDiffVoltagePre[1,] - (compDacAdjustment*compDac_mVperLSB*1e-3)

		info = dict()
		info['compDacOrig'] = compDacOrig
		info['compCalSaturationDetected'] = anySaturationDetected
		info['DacClamped'] = compDac != compDacAdjusted
		info['dacDiffVoltageTargetPre'] = dacDiffVoltagePre
		info['DE_Struct'] = DE_Struct
		info['DacCal'] = DacCal
		info['DacSel'] = DacSel
		info['CMwindow'] = CMwindow
		info['compMinADC'] = np.min(minADCperCol,axis=1)
		info['compMaxADC'] = np.max(maxADCperCol,axis=1)

		info['voltage_comp_vref1'] = self.API.CamMeasureAnaTestVoltage('comp_vref0') * 1000 #in mV
		info['voltage_comp_vref2'] = self.API.CamMeasureAnaTestVoltage('comp_vref1') * 1000 #in mV
		info['voltage_comp_vref3'] = self.API.CamMeasureAnaTestVoltage('comp_vref2') * 1000 #in mV
		mean_divisor = float(np.shape(SAT_pos[i])[1])
		info['compCalSatPerGainPerColPerPosNeg'] = np.transpose([sum_SAT_pos/mean_divisor, sum_SAT_neg/mean_divisor], (1,2,0))
		# number of saturated pixels. Dimensions: [gain transition, column, positive/negative comparator]

		return compDac, info

	def ADC_Delay_LFSR(self, inputVal, reverse=False):
		'''
		Translate ADC delay decimal value to 9-bit LFSR seed value based on LFSR polynomial: x9 + x5 + 1
		Also supports reverse lookup (to convert LFSR to decimal)
		
		Inputs:
		  inputVal           - input value
		  reverse (optional) - if true, reverse lookup. i.e. inputVal is a LFSR value and
		                       outputVal is the corresponding decimal value
		Outputs:
		  outputVal             - output value.  
		
		Note: outVal is returned empty if inputVal is invalid.
		'''
		# Generate table
		lfsrSeq9 = np.zeros(2**9 - 1, dtype=np.uint16)
		mask = int('0x01FE', 0)
		for i in range(2**9 - 2):
			curVal = lfsrSeq9[i]
			bit0Val = not((self.API.bitget(curVal, 3))^(self.API.bitget(curVal, 8))) # a ^ b is a XOR b in python
			newVal = (curVal << 1) & mask
			lfsrSeq9[i+1] = ((newVal >> 1) << 1) + bit0Val # equivalent of bitset(newVal, 1, bit1Val)

		# ADC counter adds an additional 2 cycle offset that is compensated for in
		# the lookup here.  Additionally, a LFSR value of zero is invalid in the
		# design

		if not(reverse):
			if (inputVal < 3) or (inputVal > 512):
				raise Exception('Linear adc_delay value is outside valid range [3-512]')
				logging.error('Linear adc_delay value is outside valid range [3-512]')
			else:
				outputVal = lfsrSeq9[int(inputVal - 2)] # ADC counter design has an offset of 2, but python indexes start at 0

		else:
			if (inputVal == 0):
				raise Exception('An adc_delay LFSR value of 0 is invalid in the chip design')
				logging.error('An adc_delay LFSR value of 0 is invalid in the chip design')
			else:
				try:
					outputVal = list(lfsrSeq9).index(inputVal) + 2
				except:
					raise Exception('Invalid 9b adc_delay LFSR value specified:', inputVal)
					logging.error('Invalid 9b adc_delay LFSR value specified: ' + str(inputVal))

		return outputVal
