#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#

import numpy as np
# try:
from . import GO_functions as GO
# except:
#     import GO_functions as GO
import matplotlib.pyplot as plt
import yaml
import os
import pickle
import pandas as pd

def gain_offset_calibrate(config_file=None,CalibfolderName=""):
    '''
        Top level function of gain and offset calibration

        Inputs:
          config_file           - name of configuration (.yaml) file
          CalibfolderName       - name of calbration folder
          
        Outputs:
          None
        
    '''

    with open(config_file) as y:
        file = yaml.load(y, Loader=yaml.SafeLoader)

    GO_config = file['GainOffsetCalibration']['configuration']

    if '_tempCalFiles' in os.listdir():
        pass
    else:
        os.mkdir('_tempCalFiles')
    
    GO_settings = dict(
    					NFRAMES = GO_config['NFRAMES'],
    					doplot = GO_config['doplot'],
    					ADC_FS = GO_config['ADC_FS'],
    					gainCalSatLimit = GO_config['gainCalSatLimit'],
    					offsetCalSatLimit = GO_config['offsetCalSatLimit'],
    					compCalThres = GO_config['compCalThres'],
    					compCalSatLimit = GO_config['compCalSatLimit'],
    					compCalMargin = GO_config['compCalMargin']
    					)
    
    # Create a GainOffset object with these settings
    GainOffset = GO.GainOffset(GO_settings, file, CalibfolderName)
    
    success = GainOffset.ConfigureChip()
    cal, info = GainOffset.PerColGOCalib()
    y.close()

    GO_dir = file['GainOffsetCalibration']['output_directory']
    GO_dir = os.path.join(CalibfolderName, GO_dir)
    GO_file = file['GainOffsetCalibration']['output_file_pkl'] + '.pkl'
    GO_file = os.path.join(GO_dir, GO_file)
    output_file_csv_report = file['GainOffsetCalibration']['output_file_csv_report']

    f = open(GO_file, 'rb')
    calib = pickle.load(f)
    f.close()
    if calib['info']['Pass'] == True:
        device_passed = "PASSED"
        print("\n=======================")
        print('GAIN OFFSET CAL PASSED')
        print("=======================\n")
    else:
        device_passed = "FAILED"
        print("\n=======================")
        print('GAIN OFFSET CAL FAILED')
        print("=======================\n")

    # Save results to CSV file 
    GO_status = dict()
    GO_status['Pass/Fail'] = [calib['info']['Pass'], device_passed]
    GO_status['ADC_linear_delay'] = calib['ADC']['linear_delay']
    GO_status['ADC_iramp'] = calib['ADC']['iramp']
    GO_status['ADC_up_dn_delay'] = calib['ADC']['updnoffset']
    GO_status['Vref1DAC'] = calib['gainComparator']['Vref1DAC']
    GO_status['Vref2DAC'] = calib['gainComparator']['Vref2DAC']
    GO_status['Vref3DAC'] = calib['gainComparator']['Vref3DAC']
    for key, value in calib.items():
        if isinstance(calib[key], (list,tuple)):
            if len(calib[key]) <= 4:
                GO_status[key] = value
    GO_status['inverseGlobalADCGain_0'] = calib['inverseGlobalADCGain'][0]
    GO_status['inverseGlobalADCGain_1'] = calib['inverseGlobalADCGain'][1]
    GO_status['inverseGlobalADCGain_2'] = calib['inverseGlobalADCGain'][2]
    GO_status['inverseGlobalADCGain_3'] = calib['inverseGlobalADCGain'][3]
    GO_status['Vref3DAC'] = calib['gainComparator']['Vref3DAC']
    df = pd.DataFrame(dict([ (k,pd.Series(v)) for k,v in GO_status.items() ]))
    df = df.T
    df.columns = ['Value', 'Result', 'Min Limit', 'Max Limit']
    df.to_csv(os.path.join(GO_dir, output_file_csv_report + '.csv'))

if __name__ == "__main__":
    gain_offset_calibrate("./config/CalibrationConfig.yaml", CalibfolderName = "")
# plt.show()

