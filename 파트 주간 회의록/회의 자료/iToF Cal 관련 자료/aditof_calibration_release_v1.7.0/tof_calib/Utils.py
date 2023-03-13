#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#

import numpy as np
import pickle
import yaml
from . import P0Util
from . import StreamPCMUtil


def util_main(config_file, CalibfolderName):
	"""
	Utils menu
	"""
	while(True):
		print("\n=============================")
		print("    ToF Calibration v0.5.0")
		print("          UTILITIES       ")
		print("=============================\n")
		print("Calibration file directory created->", CalibfolderName)
		print("Configuration file used ->", config_file, '\n')
		print("OPTIONS:")
		print("1 -> COMPRESS P0 TABLE")
		print("2 -> UNCOMPRESS P0 TABLE")
		print("3 -> PCM STREAM")
		print("4 -> FRAMES CAPTURE")
		print("Q -> MAIN MENU")
		a = str(input("\nSelect option:"))

		if a == 'q' or a == 'Q':
			break
		elif a == '1':
			P0Util.CompressP0(config_file, CalibfolderName)
		else:
			print("Invalid option")
