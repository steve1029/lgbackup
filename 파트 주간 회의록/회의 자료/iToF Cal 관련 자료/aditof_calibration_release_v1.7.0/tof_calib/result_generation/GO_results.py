#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#
import numpy as np
import pickle
import os
import matplotlib.pyplot as plt


def add_results_GO(CalibfolderName, calibResult, file, result_dir):
	'''
		Adds results of gain and offset calibration

		Inputs:
		  CalibfolderName       - Name of calibration output directory
		  calibResult           - Calibration result dict
		  file                  - Name of .yaml config file
		  result_dir            - Name of result directory
		  
		Outputs:
		  calibResult           - Updated calibration result dict
		
	'''
	print("Generating results for Gain and Offset Calibration...")
	GO_dir = os.path.join(CalibfolderName, file['GainOffsetCalibration']['output_directory'])
	# GO_output = os.path.join(CalibfolderName, file['GainOffsetCalibration']['output_file_pkl'])

	if os.path.exists(GO_dir):
		if os.path.exists(os.path.join(GO_dir, file['GainOffsetCalibration']['output_file_pkl'] + '.pkl')) and os.path.exists(os.path.join(GO_dir,'GO_info.pkl')):
			calibResult['gain_offset_exists'] = True
			f = open(os.path.join(GO_dir, file['GainOffsetCalibration']['output_file_pkl'] + '.pkl'), 'rb')
			GO_cal_dict = pickle.load(f)
			f.close()
			f = open(os.path.join(GO_dir,'GO_info.pkl'), 'rb')
			GO_info = pickle.load(f)
			f.close()

			calibResult['gain_offset'] = dict()
			calibResult['gain_offset_pass'] = GO_cal_dict["info"]["Pass"]
			calibResult['gain_offset']['adc_iramp'] = GO_cal_dict['ADC']['iramp']
			calibResult['gain_offset']['adc_delay'] = GO_cal_dict['ADC']['linear_delay']
			calibResult['gain_offset']['adc_up_dn_delay'] = GO_cal_dict['ADC']['updnoffset']

			calibResult['gain_offset']['Gain0_MeasNative'] = np.round(GO_info['ampgainMeasuredNative'][0],3)
			calibResult['gain_offset']['Gain1_MeasNative'] = np.round(GO_info['ampgainMeasuredNative'][1],3)
			calibResult['gain_offset']['Gain2_MeasNative'] = np.round(GO_info['ampgainMeasuredNative'][2],3)
			calibResult['gain_offset']['Gain3_MeasNative'] = np.round(GO_info['ampgainMeasuredNative'][3],3)

			calibResult['gain_offset']['GainCal_0_ADC_min'] = np.round(GO_info['ADC_Native_min'][0],3)
			calibResult['gain_offset']['GainCal_0_ADC_mean_low'] = np.round(GO_info['ADC_Native_mean_low'][0],3)
			calibResult['gain_offset']['GainCal_0_ADC_mean_high'] = np.round(GO_info['ADC_Native_mean_high'][0],3)
			calibResult['gain_offset']['GainCal_0_ADC_max'] = np.round(GO_info['ADC_Native_max'][0],3)

			calibResult['gain_offset']['GainCal_1_ADC_min'] = np.round(GO_info['ADC_Native_min'][1],3)
			calibResult['gain_offset']['GainCal_1_ADC_mean_low'] = np.round(GO_info['ADC_Native_mean_low'][1],3)
			calibResult['gain_offset']['GainCal_1_ADC_mean_high'] = np.round(GO_info['ADC_Native_mean_high'][1],3)
			calibResult['gain_offset']['GainCal_1_ADC_max'] = np.round(GO_info['ADC_Native_max'][1],3)

			calibResult['gain_offset']['GainCal_2_ADC_min'] = np.round(GO_info['ADC_Native_min'][2],3)
			calibResult['gain_offset']['GainCal_2_ADC_mean_low'] = np.round(GO_info['ADC_Native_mean_low'][2],3)
			calibResult['gain_offset']['GainCal_2_ADC_mean_high'] = np.round(GO_info['ADC_Native_mean_high'][2],3)
			calibResult['gain_offset']['GainCal_2_ADC_max'] = np.round(GO_info['ADC_Native_max'][2],3)

			calibResult['gain_offset']['GainCal_3_ADC_min'] = np.round(GO_info['ADC_Native_min'][3],3)
			calibResult['gain_offset']['GainCal_3_ADC_mean_low'] = np.round(GO_info['ADC_Native_mean_low'][3],3)
			calibResult['gain_offset']['GainCal_3_ADC_mean_high'] = np.round(GO_info['ADC_Native_mean_high'][3],3)
			calibResult['gain_offset']['GainCal_3_ADC_max'] = np.round(GO_info['ADC_Native_max'][3],3)

			calibResult['gain_offset']['Vref1DAC'] = np.round(GO_cal_dict['gainComparator']['Vref1DAC'],2)
			calibResult['gain_offset']['Vref2DAC'] = np.round(GO_cal_dict['gainComparator']['Vref2DAC'],2)
			calibResult['gain_offset']['Vref3DAC'] = np.round(GO_cal_dict['gainComparator']['Vref3DAC'],2)

			calibResult['gain_offset']['voltage_comp_vref1'] = GO_info['voltage_comp_vref1']
			calibResult['gain_offset']['voltage_comp_vref2'] = GO_info['voltage_comp_vref2']
			calibResult['gain_offset']['voltage_comp_vref3'] = GO_info['voltage_comp_vref3']

			try:
				calibResult['gain_offset']['VCMDAC'] = np.round(GO_cal_dict['gainComparator']['VCMDAC'],2)
			except:
				calibResult['gain_offset']['VCMDAC'] = 'NaN'

			calibResult['gain_offset']['inverseGlobalADCGain_0'] = GO_cal_dict['inverseGlobalADCGain'][0]
			calibResult['gain_offset']['inverseGlobalADCGain_1'] = GO_cal_dict['inverseGlobalADCGain'][1]
			calibResult['gain_offset']['inverseGlobalADCGain_2'] = GO_cal_dict['inverseGlobalADCGain'][2]
			calibResult['gain_offset']['inverseGlobalADCGain_3'] = GO_cal_dict['inverseGlobalADCGain'][3]
		else:
			calibResult['gain_offset_pass'] = False

		if os.path.exists(os.path.join(GO_dir, 'MinMaxADCValueperCol.pkl')) and calibResult['gain_offset_pass']:
			f = open(os.path.join(GO_dir, 'MinMaxADCValueperCol.pkl'), 'rb')
			MinMaxADCValueperCol = pickle.load(f)
			f.close()

			maxADCperCol = MinMaxADCValueperCol['maxADCperCol']
			minADCperCol = MinMaxADCValueperCol['minADCperCol']

			plt.figure()
			for i in range(3):
				plt.plot(range(0,1024), maxADCperCol[i], 'x-', label = '(+)Gain '+ str(i))
				plt.plot(range(0,1024), minADCperCol[i], 'x-', label = '(-)Gain '+ str(i))
			plt.legend()
			plt.xlabel('Pixel col (0-1023)')
			plt.ylabel('ADC value')
			plt.title('Maximum ADC value per column at comparator switching threshold')
			plt.savefig(os.path.join(result_dir, 'MaxADCValueperCol.png'))
			plt.close()
			calibResult['gain_offset']['MinMaxADCValueperCol'] = os.path.join('Results', 'MaxADCValueperCol.png')

		if os.path.exists(os.path.join(GO_dir, 'measuredGainNative.pkl')) and calibResult['gain_offset_pass']:
			f = open(os.path.join(GO_dir, 'measuredGainNative.pkl'), 'rb')
			measuredGainNative_dict = pickle.load(f)
			f.close()

			measuredGainNative = measuredGainNative_dict['measuredGainNative']
			tenBitMode = measuredGainNative_dict['tenBitMode']
			ADC_FS = measuredGainNative_dict['ADC_FS']

			plt.figure()
			for i in range(4):
				plt.plot(range(0,1024),measuredGainNative[:,0,i] / (2**(9+tenBitMode)/(ADC_FS*2)),'x-',label='g' + str(i) + ' even row')
				plt.plot(range(0,1024),measuredGainNative[:,1,i] / (2**(9+tenBitMode)/(ADC_FS*2)),'x-',label='g' + str(i) + ' odd row')
				# plt.scatter(range(0,1024),measuredGainNative[:,0,i] / (2**(9+self.tenBitMode)/(ADC_FS*2)),label='g' + str(i) + ' even row')
				# plt.scatter(range(0,1024),measuredGainNative[:,1,i] / (2**(9+self.tenBitMode)/(ADC_FS*2)),label='g' + str(i) + ' odd row')
			plt.title('Gain')
			plt.legend()
			plt.xlabel('Column number [0-1023]')
			plt.ylabel('Gain')
			plt.savefig(os.path.join(result_dir, 'MeasuredGainNative.png'))
			plt.close()
			calibResult['gain_offset']['measuredGainNative'] = os.path.join('Results', 'MeasuredGainNative.png')

		if os.path.exists(os.path.join(GO_dir, 'measuredOffsetNative.pkl')) and calibResult['gain_offset_pass']:
			f = open(os.path.join(GO_dir, 'measuredOffsetNative.pkl'), 'rb')
			measuredOffsetNative_dict = pickle.load(f)
			f.close()

			measuredOffsetNative = measuredOffsetNative_dict['measuredOffsetNative']
			tenBitMode = measuredOffsetNative_dict['tenBitMode']
			ADC_FS = measuredOffsetNative_dict['ADC_FS']

			for i in range(4):
				plt.plot(range(0,1024),measuredOffsetNative[:,0,i],'x',label='g' + str(i) + ' even row')
				plt.plot(range(0,1024),measuredOffsetNative[:,1,i],'x',label='g' + str(i) + ' odd row')
				# plt.scatter(range(0,1024),measuredOffsetNative[:,0,i],marker,label='g' + str(i) + ' even row')
				# plt.scatter(range(0,1024),measuredOffsetNative[:,1,i],,label='g' + str(i) + ' odd row')
			plt.title('Offset')
			plt.legend()
			plt.xlabel('Column number [0-1023]')
			plt.ylabel('Offset')
			plt.savefig(os.path.join(result_dir, 'MeasuredOffsetNative.png'))
			plt.close()
			calibResult['gain_offset']['measuredOffsetNative'] = os.path.join('Results', 'MeasuredOffsetNative.png')

	return calibResult
