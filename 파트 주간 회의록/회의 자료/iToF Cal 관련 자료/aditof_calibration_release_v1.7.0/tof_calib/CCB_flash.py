#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#

import numpy as np
import os
import sys
import yaml


aditofdevicepython_path = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'aditofdevicepython'))
aditofpython_path = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'aditofpython'))

if aditofdevicepython_path not in sys.path:
	sys.path.insert(1, aditofdevicepython_path)

if aditofpython_path not in sys.path:
	sys.path.insert(1, aditofpython_path)
# os.chdir(os.path.dirname(dname))



# from . import CalibAPI as tof

import shutil

def ReadBarcode(json_file):#CalibfolderName=None):

	import aditofpython as tofflash

	eModeIndex = 10
	
	system = tofflash.System()
	status = system.initialize()
	print("system.initialize()", status)
	devices = []
	cameras = []
	status = system.getCameraList(cameras)
	print("system.getCameraList()", status)

	print("Cameras: ", cameras)
	camera1 = cameras[0]

	status = camera1.initialize(json_file)
	print("camera1.initialize()", status)
	assert (status == tofflash.Status.Ok)

	status = camera1.powerUp()
	print("camera1.powerUp()", status)
	assert (status == tofflash.Status.Ok)

	CCB = []
	status = camera1.readModuleCCBEeprom(CCB);

	barcode_raw = CCB[0][176 : 208]

	barcode = ""

	for b in barcode_raw:
		barcode = barcode + chr(b)

	print("BARCODE READ: ",barcode)

	return barcode


def WriteBarcode(cfg_file, ccb_file, json_file):

	import aditofpython as tofflash

	eModeIndex = 10
	
	system = tofflash.System()
	status = system.initialize()
	print("system.initialize()", status)
	devices = []
	cameras = []
	status = system.getCameraList(cameras)
	print("system.getCameraList()", status)

	print("Cameras: ", cameras)
	camera1 = cameras[0]

	status = camera1.initialize(json_file)
	print("camera1.initialize()", status)
	assert (status == tofflash.Status.Ok)

	status = camera1.powerUp()
	print("camera1.powerUp()", status)
	assert (status == tofflash.Status.Ok)

	# camera1.writeModuleEeprom(ccb_file,cfg_file)
	status = camera1.writeModuleEeprom(ccb_file, cfg_file)

	if (status == tofflash.Status.Ok):
		success = True
	else:
		success = False

	return success
