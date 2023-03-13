#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#

import csv
import numpy as np
from datetime import datetime
import os
import yaml
import sys

def generate_header(config_file="", CalibfolderName=""):
	# Format = YYMM-AAABBBB
	# YY -> Year
	# MM -> Month
	# AAA -> module_class
	#        legacy -> 000
	#        walden_R1 -> 100
	# BBBB -> Random/serial number

	with open(os.path.join(CalibfolderName,'header.csv'), 'w', newline='') as csvfile:
		headerwriter = csv.writer(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
		headerwriter.writerow(['Module Serial Number', str(CalibfolderName)])