#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#

import pickle
import csv
import numpy as np
import sys

import scipy.io

# 
# Converts a gain and offset pkl file to csv
# 

def const_hex(val):
	val = np.uint16(val)
	return '0x' + '0'*(max(0,4-len(str(hex(val))[2:]))) + str(hex(val))[2:]

def csv_regwrite(obj, addr, val):
	obj.writerow([const_hex(val)])

def csv_regwriteburst(obj, addr, val):
	for i in range(len(val)):
		obj.writerow([const_hex(addr), const_hex(val[i]), 'Burst'])


fname = str(sys.argv[1])
print(fname)

f = open(fname, 'rb')
cal = pickle.load(f)
f.close()

print('Calibration file with', cal.keys())

c = csv.writer(open(fname[0:-3] + 'csv', 'w', newline=''))

c.writerow([const_hex(cal['ADC']['updnoffset'])])
c.writerow([const_hex(cal['ADC']['iramp'])])

c.writerow([const_hex(cal['gainComparator']['Vref1DAC'])])
c.writerow([const_hex(cal['gainComparator']['Vref1Set'])])
c.writerow([const_hex(cal['gainComparator']['Vref2DAC'])])
c.writerow([const_hex(cal['gainComparator']['Vref2Set'])])
c.writerow([const_hex(cal['gainComparator']['Vref3DAC'])])
c.writerow([const_hex(cal['gainComparator']['Vref3Set'])])

if 'VCMDAC' in list(cal['gainComparator'].keys()):
	c.writerow([const_hex(cal['gainComparator']['VCMDAC'])])
else:
	c.writerow([const_hex(0x0000)])

c.writerow([const_hex(cal['gainComparator']['VCMSet'])])

c.writerow([const_hex(cal['inverseGlobalADCGain'][0])])
c.writerow([const_hex(cal['inverseGlobalADCGain'][1])])
c.writerow([const_hex(cal['inverseGlobalADCGain'][2])])
c.writerow([const_hex(cal['inverseGlobalADCGain'][3])])

tmp = cal['ramDataGain']
for i in range(len(tmp)):
	c.writerow([const_hex(tmp[i])])

tmp = cal['ramDataOffset']
for i in range(len(tmp)):
	c.writerow([const_hex(tmp[i])])
