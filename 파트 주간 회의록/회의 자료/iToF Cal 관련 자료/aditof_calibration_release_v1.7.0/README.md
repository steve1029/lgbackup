# Calibration Software readme

This repository contains the source code for calibration of ADI ToF cameras. The repository has the following directories:

```
├── tof_calib: scripts for calibration
├── aditofdevicepython: SDK python bindings
├── config: configuration files
├── templates: calibration report templates
├── html: function documentation
├── lamp_control: ADI specific IR lamp control
├── software license: license documentation
├── verify: depth compute functions
├── environment.yml: yml file to create an anaconda environment
├── run_calibration.py: script to start a calibration session
├── run_calibration_no_imatest.py: script to start a calibration session if Imatest is not installed
└── run_XXX.py: scripts to run individual calibration routines
```

### Dependencies
* Install latest [Miniconda](https://conda.io/miniconda.html)
* Install Imatest

### Installation instructions
* UnZip calibration source code files downloaded from Analog Devices
* Open an Anaconda Command Prompt or Windows Command Prompt
* Navigate to source code root directory (e.g. C:\Calibration)
* Create Anaconda/Miniconda environment from file and activate environment
```
cd Calibration
conda env create -f environment.yml
conda activate tofCalib
```

### Imatest installation instructions
* Imatest download and installation (https://www.imatest.com/docs/installation/)
* Imatest IT instructions (https://www.imatest.com/docs/imatest-it-instructions/#Python)
* To install Imatest in the anaconda environment:
    * Run setup.py with a prefix
    * ``` 
        cd %PROGRAMFILES%\Imatest\v2020.1\IT\libs\library\python
        python setup.py install --force --prefix=%VENV_PATH%
        ```
    * Where the environment variable VENV_PATH is the path to the 'root' directory for the virtual environment

### Run/ start a calibration session
```
python run_calibration.py <path-to-yaml_config_file>
Example: python run_calibration.py ./config/CalibrationConfig_Walden.yaml
```

# Getting Started with Calibration Software

### Starting a calibration session
A calibration session can be started with either of the following commands from ./calibration (home) directory:
* To create a session with a unique timestamp use the command mentioned below. This command creates a new calibration session directory in the format CalibFiles_<ToF module class>_yyyymmdd_HHMMSS 
```
python run_calibration.py <path-to-yaml_config_file>
```

* To create a session with a custom name or to access an existing session directory, use the command mentioned below.
```
python run_calibration.py <path-to-yaml_config_file> <session_directory_name>
```

* To run calibration software if Imatest is not installed, replace **run_calibration.py** with **run_calibration_no_imatest.py** in the above commands.

### Calibration Menu
After running one of the two commands mentioned previously, a calibration menu appears on the console.
```
=============================
    ToF Calibration v1.1
=============================

Calibration file directory created-> <session_directory_name>
Configuration file used -> <path-to-yaml_config_file>

OPTIONS:
1 -> CONNECTIVITY TEST and ICQ     :NOT EXEC
2 -> GAIN AND OFFSET CALIBRATION   :NOT EXEC
3 -> VLOW AND DAC TRIM             :NOT EXEC
4 -> BLEMISH TEST                  :NOT EXEC
5 -> FOCUS ADJUSTMENT              :NOT EXEC
6 -> FOCUS VERIFICATION            :NOT EXEC
7 -> GEOMETRIC CALIBRATION         :NOT EXEC
8 -> LSDAC SETTING                 :NOT EXEC
9 -> P0 CALIBRATION                :NOT EXEC
C -> GENERATE CCB
M -> FLASH CCB TO MODULE
V -> VERIFY CALIBRATION
R -> GENERATE REPORT
Q -> QUIT


Select option:
```
The menu displays the calibration options available in the recommended order. Most of the directory/file names of the calibration input/output can be changed in the yaml config file. The status of the calibration is displayed on the right.
```
1 -> Incoming Quality Check and connectivity test to make sure that the camera is operable.
2 -> Gain and Offset calibration is a prerequisite for many of the following calibrations.
3 -> This option performs Vlow and DAC trim.
4 -> Requires blemish station with light panel. Blemish test is not available if run_calibration_no_imatest.py is used. Gain and offset calibration is required.
5 -> Requires focus station. Focus adjustment is not available if run_calibration_no_imatest.py is used. Gain and offset calibration is required.
6 -> Requires focus station. Focus verification is not available if run_calibration_no_imatest.py is used. Gain and offset calibration is required.
7 -> Requires geometric station. Gain and offset calibration is required.
8 -> LSDAC values are added based on the csv file name mentioned in the yaml configuration file: LSDACSetting.input_file_csv
9 -> Requires P0 station. Gain and offset calibration is required.
C -> This option generates a ccb file (compressed and uncompressed versions) from the calibration output files present in the session. If there are no calibration output files, an empty .ccb file is created.
M -> This option writes the cfg file and compressed CCB file to NVM on the module
V -> Requires flat wall target. This option checks flatness of point cloud and verifies CCB generation and write to NVM were successful
R -> This option generates a html report based on the calibration outputs in the session
Q -> Exits the session
```

Note:
* Performing a calibration multiple times will not replace the existing calibration output file. A timestamp will be added to the existing directory.
* The latest calibration output (the one matching the file names in the yaml config file) is used for ccb and report generation.
* If you encounter an error or if the program exits the session directory, you can reenter the session using the command **python run_calibration.py <path-to-yaml_config_file> <session_directory_name>**

# Tips and tricks

### Generating .ccb without a camera
1. Start a calibration session or open an existing session directory.

2. Create directories based on the file names mentioned in yaml configuration file used. And copy the calibration output files into them.
For example:
 ```
 Walden_001 (session directory)
    ├── AddressValueRegisterList
    |   ├── AddressValueRegisterList.csv
    |   ├── AddressValueRegisterList.pkl
    |   └── AVRL_dict.pkl
    ├── GO_output
    |   ├── ADC_Native.pkl
    |   ├── DacCalibration.pkl
    |   :
    |   └── MinMaxADCValueperCol.pkl
    :
    :
    └── P0_output
        ├── mean_frames_Mode_1.pkl
        ├── P0Table_mode_1.pkl
        :
        └── test_mode_1_freqDist_312mm.pkl
```
3. Enter 'C' or 'c' to generate the ccb file.

### Replacing a block/multiple blocks in .ccb file
1. For this, you require the calibration output files of both the calibration being replaced and updated calibration.
2. Start a calibration session or open an existing session directory.
3. If you want to replace, say, the P0 block, replace the files in the directory P0_output (refer yaml configuration file for the directory/file name) and enter 'C' in the options to generate the ccb file.

### Run and individual calibration script
```
python run_ICQ.py <path-to-yaml_config_file> <optional_calibration_folder_name>
Example: python run_ICQ.py ./config/CalibrationConfig_Walden.yaml
```

### Miscellaneous
1. Refer config/GeometricParameterMapping.txt for help in matching the instrinsic parameter.



