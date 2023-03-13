#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.
# This software is proprietary to Analog Devices, Inc. and its licensors.
#

import numpy as np
import matplotlib.pyplot as plt
import os
import jinja2 as j2
import pickle
from bs4 import BeautifulSoup
import pandas as pd


def generate_report(config_file=None, CalibfolderName=None):
    '''
		Generates calibration report. Supports modes 0,1,4,5,7 only

		Inputs:
		  config_file           - Name of configuration .yaml file
		  CalibfolderName       - Name of calibration folder
		  
		Outputs:
		  None
		
	'''

    result_dir = os.path.join(CalibfolderName, 'Results')
    template_folder = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'templates'))
    template_file = 'CalibrationReport.j2'

    file_loader = j2.FileSystemLoader(template_folder)
    j2_env = j2.Environment(loader=file_loader)
    template = j2_env.get_template(template_file)

    f = open(os.path.join(result_dir, 'results.pkl'), 'rb')
    results = pickle.load(f)
    f.close()

    result_keys = list(results.keys())
    print('KEYS FOUND:', result_keys)

    if results['Header_exists']:
        print('\n---Header Info---')
        for key, val in results['HeaderInfo'].items():
            print(key, ':', val)

    if results['ICQC_exists']:
        print('\n---ICQ and Connectivity---')
        for key, val in results['ICQC'].items():
            print(key, ':', val)

    if results['gain_offset_exists']:
        print('\n---Gain & Offset Calibration---')
        for key, val in results['gain_offset'].items():
            print(key, ':', val)

    if results['AVRL_exists']:
        print('\n---Address Value Register List---')
        for key, val in results['AVRL'].items():
            print(key, ':', val)

    if results['Blemish_exists']:
        print('\n---Blemish Test---')
        for key, val in results['Blemish'].items():
            print(key, ':', val)

    if results['Focus_exists']:
        print('\n---Focus Verification---')
        for key, val in results['Focus'].items():
            print(key, ':', val)

    if results['LSDAC_exists']:
        print('\n---LSDAC Settings---')
        print(results['LSDACSetting'])

    if results['Geometric_exists']:
        print('\n---Geometric Calibration---')
        for key, val in results['Geometric'].items():
            print(key, ':', val)

    if results['P0_exists']:
        print('\n---P0 Calibration---')
        for key, val in results['P0'].items():
            print(key, ':', val)

    if results['Cal_verify_exists']:
        print('\n---Calibration Verification---')
        for key, val in results['Verify'] .items():
            print(key, ':', val)


    report_html = template.render(results)

    html_fname = os.path.join(CalibfolderName, 'report.html')
    csv_fname = os.path.join(CalibfolderName, 'report.csv')

    with open(html_fname, "w") as f:
        f.write(report_html)

    
    soup = BeautifulSoup(open(html_fname), "html.parser")
    tables = soup.find_all("table")
    data = []

    for table in tables:
        rows = table.find_all('tr')

        for element in rows:
            sub_data = []
            for sub_element in element:
                try:
                    sub_data.append(sub_element.get_text())
                except:
                    continue

            data.append(sub_data)

        data.append(" ") # Adding a space before the next table

    dataFrame = pd.DataFrame(data = data)
    dataFrame.to_csv(csv_fname, index=False)


if __name__ == "__main__":
    generate_report("./config/CalibrationConfig.yaml",
                    CalibfolderName="./CalibFiles")
