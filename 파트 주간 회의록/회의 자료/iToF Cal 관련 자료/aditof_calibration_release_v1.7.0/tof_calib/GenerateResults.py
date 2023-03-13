#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#

import numpy as np
import pickle
import os
import matplotlib.pyplot as plt
import yaml
import os
from . import GenerateReport as rep
# except:
#     import GenerateReport as rep
from scipy.constants import c, pi
from scipy import signal as sig
from datetime import datetime

from . import P0_functions as P0

from .result_generation import AVRL_results as AVRL_res
from .result_generation import Geometric_results as Geo_res
from .result_generation import GO_results as GO_res
from .result_generation import ICQC_results as ICQC_res
from .result_generation import LSDAC_results as LSDAC_res
from .result_generation import P0_results as P0_res
from .result_generation import Verify_results as ver_res
from .result_generation import Header_results as header_res
from .result_generation import Blemish_results as blem_res
from .result_generation import Focus_results as focus_res



def generate_results(config_file=None, CalibfolderName=None):
	'''
		Generates calibration results file and plots

		Inputs:
		  config_file           - name of configuration (.cfg) file
		  CalibfolderName       - name of calbration folder
		  
		Outputs:
		  None
		
	'''

	calibResult = dict()

	if not os.path.exists(os.path.join(CalibfolderName, 'Results')):
		os.mkdir(os.path.join(CalibfolderName, 'Results'))
	else:
		now = datetime.now()
		print('\nALERT: ' + os.path.join(CalibfolderName, 'Results') + ' already exists')
		old_folderName = os.path.join(CalibfolderName, 'Results') + str(now.strftime('_old_%Y%m%d_%H%M%S'))
		os.rename(os.path.join(CalibfolderName, 'Results'), old_folderName)
		print('ALERT: RENAMING OLD FOLDER: ' + os.path.join(CalibfolderName, 'Results') + ' as '+ old_folderName +'\n')
		os.mkdir(os.path.join(CalibfolderName, 'Results'))


	result_dir = os.path.join(CalibfolderName, 'Results')

	with open(config_file) as y:
		file = yaml.load(y, Loader=yaml.SafeLoader)


	calibResult['module_class'] = file['module_class']
	calibResult['module_name'] = os.path.basename(CalibfolderName)
	# calibResult['config_file'] = 'Refer each section'
	now = datetime.now()
	calibResult['date'] = str(now.strftime('%m/%d/%Y'))
	calibResult['time'] = str(now.strftime('%H:%M:%S'))

	calibResult['gain_offset_exists'] = False
	calibResult['P0_exists'] = False
	calibResult['LSDAC_exists'] = False
	calibResult['Geometric_exists'] = False
	calibResult['AVRL_exists'] = False
	calibResult['ICQC_exists'] = False
	calibResult['Cal_verify_exists'] = False
	calibResult['Header_exists'] = False
	calibResult['Focus_exists'] = False
	calibResult['Blemish_exists'] = False

	calibResult['gain_offset_pass'] = False
	calibResult['P0_pass'] = False
	calibResult['LSDAC_pass'] = False
	calibResult['Geometric_pass'] = False
	calibResult['AVRL_pass'] = False
	calibResult['ICQC_pass'] = False
	calibResult['Header_pass'] = False
	calibResult['Focus_pass'] = False
	calibResult['Blemish_pass'] = False

	calibResult = ICQC_res.add_results_ICQC(CalibfolderName, calibResult, file, result_dir)
	calibResult = GO_res.add_results_GO(CalibfolderName, calibResult, file, result_dir)
	calibResult = AVRL_res.add_results_AVRL(CalibfolderName, calibResult, file, result_dir)
	calibResult = blem_res.add_results_blemish(CalibfolderName, calibResult, file, result_dir)
	calibResult = focus_res.add_results_focus(CalibfolderName, calibResult, file, result_dir)
	calibResult = Geo_res.add_results_geometric(CalibfolderName, calibResult, file, result_dir)
	calibResult = P0_res.add_results_P0(CalibfolderName, calibResult, file, result_dir, config_file, file['reducedP0Report'])
	calibResult = LSDAC_res.add_results_LSDAC(CalibfolderName, calibResult, file, result_dir)
	calibResult = ver_res.add_results_verify(CalibfolderName, calibResult, file, result_dir)

	try:
		calibResult = header_res.add_results_header(CalibfolderName, calibResult, file, result_dir)
	except:
		pass

	f = open(os.path.join(result_dir, 'results.pkl'), 'wb')
	pickle.dump(calibResult, f)
	f.close()

	rep.generate_report(config_file, CalibfolderName)
