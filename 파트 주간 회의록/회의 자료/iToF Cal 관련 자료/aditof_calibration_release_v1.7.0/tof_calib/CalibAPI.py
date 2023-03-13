#!/usr/bin/env python3
#
# Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.   
# This software is proprietary to Analog Devices, Inc. and its licensors.  
#

import cv2
import numpy as np
import csv
import time
import sys
import matplotlib.pyplot as plt
from datetime import datetime
import pickle
import os
import yaml

import logging
from datetime import datetime
from scipy import ndimage

#DEBUG - add delay after reading frame
debug_delay = 0.01

# sys.path.append("./ADI_FG/TOFI-93/ardacho_fpga/test/device_python_wrapper/src/")
# sys.path.append("./aditofdevicepython/") 

aditofdevicepython_path = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'aditofdevicepython'))

if aditofdevicepython_path not in sys.path:
    sys.path.insert(1, aditofdevicepython_path)


try:
    from . import GenerateLSDACBlock as LSDAC
except:
    import GenerateLSDACBlock as LSDAC

# import ArdachoIOLib.ArdachoIOLib as ArdachoIOLib
import aditofdevicepython as tofdevice
from struct import *

# GPIO definitions
CAM_RESET_N = 79
MPSOC_SYNC_DIR = 77
MPSOC_SYNC_DIR1 = 169
GPIO_CAM_FSYNC = 170
MPSOC_SYNC_SEL0 = 75
MPSOC_SYNC_SEL1 = 76

LMZ_EN = 83
PS_V5VVPS3 = 69
PP_DEPTH = 85
PS_VAUXVPS1 = 67
PS_VAUXVPS2 = 68
DEPTH_VAUX_RUN = 84
CAM_SHUTDOWN_N =64

I2C_MALKA = 0       
I2C_MODULE = 1      # Mainly used for the temp sensor and Archer driver

#AD5593R ID and DAC Selection

AD5593R_ID = 0x11
AD5593R_DAC_SET = 0x10
VAUX_ADJ = 0x01


# Power Monitor Ids
VSYS_DEV_ID = 0x44
VMAIN_DEV_ID = 0x42
VAUX_DEV_ID = 0x41
VDEPTH_DEV_ID = 0x40
TEMP_SENSE_ID = 0x48
TEMP_REG = 0

s1 = np.array([ 0x0100,
                0x0102,
                0x0104,
                0x0106,
                0x0108,
                0x010a,
                0x010c,
                0x010e,
                0x0110,
                0x0116,
                0x011a,
                0x011c,
                0x0122,
                0x012a,
                0x012c,
                0x012e,
                0x0130,
                0x0134,
                0x0136,
                0x0138,
                0x014c,
                0x014e,
                0x0154,
                0x0156,
                0x0158,
                0x015a,
                0x015c,
                0x015e,
                0x0160,
                0x0184,
                0x0192], dtype=np.uint16)

s2 = np.array([ 0x0162,
                0x016a,
                0x016c,
                0x016e,
                0x0170,
                0x0172], dtype=np.uint16)

# ADSD3000 - Laser Driver Definitions
ADSD3000_ID = 0x64

adsd3000_config = np.array([[0x0404, 0x0001],
                            [0x0002, 0x0025],
                            [0x0404, 0x0001],
                            [0x0003, 0x2478],
                            [0x0002, 0x0022],
                            [0x0404, 0x0001],
                            [0x000D, 0x00E0],
                            [0x000E, 0x07C0],
                            [0x0004, 0x9701],
                            [0x0060, 0xA808],
                            [0x0005, 0x00F4],
                            [0x0006, 0x0000],
                            [0x0013, 0x0008],
                            [0x0002, 0x002C]],dtype=np.uint16)

# Addr      Data
# 0404       0001
# 0002       0025
# 0404       0001
# 0003       2400
# 0002       0022
# 0404       0001
# 000E       07CB
# 0004       972F
# 0060       A808 (readback value is 8808)
# 0002       002C (enable internal SYNC, external SYNC use 0024)

def getABdata(Test_imageData):
    imageDataComplex_no_P0 = np.zeros([Test_imageData.shape[0], Test_imageData.shape[1], 3], dtype = complex)
    nFreqs = 3
    nPhases = 3
    capturePhaseStep = 2*np.pi*np.array([0, 0.33333, 0.66667, 0, 0.33333, 0.66667, 0, 0.33333, 0.66667])
    # capturePhaseStep = np.zeros(nCaptures).astype('float')
    frequencyMatrix = np.zeros([nPhases, 2])

    for f in range(nFreqs):
        for j in range(nPhases):
            pidx = f * nPhases + j
            frequencyMatrix[j, 0] = np.cos(-1 * capturePhaseStep[pidx])
            frequencyMatrix[j, 1] = np.sin(-1 * capturePhaseStep[pidx])

        pseudoinverseMatrix = np.array(np.round(np.linalg.pinv(frequencyMatrix), 8), dtype=complex)

        for j in range(nPhases):
            imageDataComplex_no_P0[:, :, f] =  (imageDataComplex_no_P0[:, :, f] + Test_imageData[:, :, f * nPhases + j] * (pseudoinverseMatrix[0,j] + 1j* pseudoinverseMatrix[1,j]))


    clean_IR = np.abs(imageDataComplex_no_P0)

    return clean_IR


def ICQtest(measurement, test_min, test_max):
    """
    Computes whether a test has passed or failed
    Inputs:
        measurement: measured value  
        test_min: minimum limit  
        test_max: maximum limit  

    Outputs:
        ret: status of test in string  
        tests_failed: status of test in boolean  

    """
    if(measurement>test_min and measurement<test_max):
        ret = "PASSED"
        tests_failed = 0
    else:
        ret = "FAILED"
        tests_failed = 1
    return ret, tests_failed


class CalibAPI:

    def __init__( self, module_class = "Legacy", n_modes_cfg = 10, debug = False):
        """Class constructor
        
        :returns: None  
        :rtype: None  
        """
        print("module class = " + str(module_class))
        self.bProgrammed = False
        self.Mode = 3
        self.width = 1024
        self.height =1024
        self.subFrames = 1
        self._debug = debug
        self.fps = 30
        self.module_class = module_class
        self.rawImage = np.zeros( (self.height, self.width, self.subFrames), dtype = np.int16)
        self.pRAW = np.zeros( (self.height, self.width * self.subFrames), dtype = np.int16)
        self.rawN = np.zeros( [100, (self.height* self.width * self.subFrames)], dtype = np.int16)
        self.unwrappedDepth = np.zeros( (self.height, self.width), dtype = np.double)
        self.nPhases = 0
        self.nFreq = 0
        self.freqs = []
        # self.dev = ArdachoIOLib.ArdachoIOLib() #IOLib.tof_device() $$
        self.dev = tofdevice.tof_device()
        self.dac_amptest0 = 255
        self.dac_amptest1 = 182
        self.dac_cmref = 255
        self.firstStart = True
        self.flag = 0
        self.prevVoltage = 0
        self.origSetting = dict()
        self.origSetting['reg'] = dict()
        self.origSetting['reg']['addr'] = np.array([],dtype=np.uint16)
        self.origSetting['reg']['val'] = np.array([],dtype=np.uint16)
        self.n_modes_cfg = n_modes_cfg


    def __del__(self):
        """Class destructor, deinitializes the camera.
        
        :returns: None  
        :rtype: None  
        """
        self.Stop()

        ret = self.sensorPowerDown()
        if( ret < 0):
            print("Error during sensor power down. Please restart your system")       

    def error_assert(self, var, message):
        """
        Raises error if the test has failed with a message
        Inputs:
            var: status of test  
            message: error message to display  
        Outputs:
            none  

        """
        if var == False:
            raise Exception(message)
            try:
                logging.error(message)
            except:
                pass


    def Start( self):
        """
        Command to start the camera. First checks if the microsequencer
        has started.
        Inputs:
            none  

        Outputs:
            none  

        """
        self.dev.writeSPISingle( 0x000C, 0x00C5)
        time.sleep(0.1) #Delay required for useq
        
        # check if useq works
        errorCode = 0
        for i in range(10):
            data = self.dev.readSPISingle( 0x0256,1 )
            data = data[0]
            print("data = " + hex(data))
            time.sleep(0.1)
            if( data == 0x02):
                print("uSeq running")
                break
            else:
                time.sleep(0.5)
                data = self.dev.readSPISingle( 0x0256,1 )
                data = data[0]
                print("data retry = " + hex(data))
                if( data == 0x02):
                    print("uSeq running")
                    break
                else:
                    errorCode = self.dev.readSPISingle( 0x0032, 1)[0]
                    print( "Error = " + hex(errorCode))
                
        if( errorCode != 0x0 or data == 0x00):
            sys.exit()
        
        self.dev.startSensor()
      
        # Kick-off camera stream
        self.bProgrammed = True
        print("camera programmed")


    def Stop(self):
        """
        Command to stop the camera
        Inputs:
            none 

        Outputs:
            none 

        """
        i = 0
        
        # self.dev.setGpio( MPSOC_SYNC_SEL0, 0)
        # self.dev.setGpio( MPSOC_SYNC_SEL1, 0)
        # self.dev.setGpio( MPSOC_SYNC_DIR, 0)
        # self.dev.setGpio( 171, 0) # external (0) / internal (1) sync source

        # self.dev.setGpio( MPSOC_SYNC_DIR1, 0) # Gpio manual control
        # self.dev.setGpio( GPIO_CAM_FSYNC, 0) # Gpio manual control 
        

        if( self.bProgrammed == True):
            # safely stop the camera
            self.dev.writeSPISingle( 0x000C, 0x0002)
            regData = self.dev.readSPISingle( 0x000C, 1)
            
            while( regData[0] != 0x0 and i<20):
                time.sleep(0.1)
                regData = self.dev.readSPISingle( 0x000C, 1)
                i = i + 1
        if( i == 20):
            print("Error stopping camera")

        self.dev.stopSensor()
        print("CAMERA STOPPED")




    def Init( self, Mode=3, fps=30, cfg = ""):
        """
        Initializes the image sensor.
        
        Inputs:
            Mode: sensor mode (0-10) 
            fps: maximum fps of the image stream 
            cfg: path to .cfg file 

        Outputs:
            0 if successful
        """
        ret = self.dev.openDevice( Mode, -1)
        if( ret < 0):
            print("Error opening the camera")
            return -2
        
        ''
        # 
        # self.dev.setGpio( MPSOC_SYNC_SEL0, 1)
        # self.dev.setGpio( MPSOC_SYNC_SEL1, 1)
        
        # SyncMaster = 1
        # SyncSlave = 0
        # syncSel = SyncMaster
        # self.dev.setGpio( 171, syncSel) # mux2 sel signal : external (0) / internal (1) sync source
        # self.dev.setGpio( MPSOC_SYNC_DIR, syncSel) # snl74 direction signal: external (0) / internal (1) sync source

        # #self.dev.setGpio( MPSOC_SYNC_DIR1, 0) # timer (0) / GPIO (1) internal sync
        # #self.dev.setGpio( GPIO_CAM_FSYNC, 0) # Gpio manual control 
        # 
        self.dev.setSyncMode( 0, 0) # external , external sync 1v8

        #self.dev.setSyncMode( 0, 0)

        ret = self.sensorPowerUp()
        if( ret < 0):
            print("Error during sensor power up. Please restart your system.")
            sys.exit()


        self.Mode = Mode
        self._setInternalClassMode()

        self.fps = fps
        self.dev.setCameraFrameRate( self.fps)

        print(cfg)
        time.sleep(1)
        self.dev.programCfg(cfg)

        if( Mode != 3):
            #self.changeMode( self.Mode)
            self.dev.writeSPISingle( 0x0200, self.Mode)

        self.Start()

        return 0


    def ConnectSensor(self, Mode=3, fps=10, cfg = ""):
        """
        Command to connect to sensor - powerup the device, set mode, set fps,
        and program cfg file
        Inputs:
            mode: camera mode 
            fps: frames per second 
            cfg: location of .cfg file 

        Outputs:
            none

        """
        ret = self.dev.openDevice( Mode, -1)
        if( ret < 0):
            print("Error opening the camera")
            return -2
        
        ret = self.sensorPowerUp()
        if( ret < 0):
            print("Error during sensor power up. Please restart your system.")
            sys.exit()


        self.Mode = Mode
        self._setInternalClassMode()

        self.fps = fps
        self.dev.setCameraFrameRate( self.fps)

        # print("Config_1000.00_3p6A_66p_Ofilm.cfg")
        # time.sleep(3)

        
        print(cfg)
        logging.info('Loading cfg file: ' + cfg)
        time.sleep(0.5)
        self.dev.programCfg(cfg)
        # self.programCSV("process_data_Legacy_p.csv")

        if (self.module_class == 'Walden_R1' or 'Crosby'):         # MZ: If Walden module then invert the LSMOD control register in ADSD3100
            self.InvertLSMOD()

        if( Mode != 3):
            #self.changeMode( self.Mode)
            self.dev.writeSPISingle( 0x0200, self.Mode)


    def testtestwrite(self,addr,val):
        """
        Test a register write
        Inputs:
            addr: address in hex or decimal 
            val: value in hex or decimal 

        Outputs:
            none

        """
        self.dev.writeSPISingle(addr, val)


    def DisconnectSensor(self):
        """
        Command to disconnect sensor. Stop camera and powerdown.
        Inputs:
            none  

        Outputs:
            none  

        """
        i = 0
        if( self.bProgrammed == True):
            # safely stop the camera
            self.dev.writeSPISingle( 0x000C, 0x0002)
            regData = self.dev.readSPISingle( 0x000C, 1)
            
            while( regData[0] != 0x0 and i<20):
                time.sleep(0.1)
                regData = self.dev.readSPISingle( 0x000C, 1)
                i = i + 1
        if( i == 20):
            print("Error stopping camera")
        print("CAMERA STOPPED")
        ret = self.sensorPowerDown()
        if( ret < 0):
            print("Error during sensor power down. Please restart your system")



    def StartSensor(self):
        """
        Command to start sensor. Needs ConnectSensor() to be run before.
        Starts capturing of raw data.
        Inputs:
            none 

        Outputs:
            none 

        """

        # Send start command to camera
        self.dev.writeSPISingle( 0x000C, 0x00C5)

        # check if useq works
        errorCode = 0
        for i in range(10):
            data = self.dev.readSPISingle( 0x0256,1 )
            data = data[0]
            print("data = " + hex(data))
            time.sleep(0.1)
            if( data == 0x02):
                print("uSeq running")
                break
            else:
                errorCode = self.dev.readSPISingle( 0x0032, 1)[0]
                print( "Error = " + hex(errorCode))

        if( errorCode != 0x0 or data == 0x00):
            sys.exit()

        # Kick-off camera stream
        self.bProgrammed = True
        print("camera programmed")
        self.dev.startSensor()

        return 0

    def QuickStartSensor(self):
        """
        Command to start sensor without checks
        Inputs:
            none 
        Outputs:
            none 

        """
        self.dev.startSensor()
        if self.firstStart == True:
            self.dev.writeSPISingle( 0x000C, 0x00C5)
            self.firstStart = False
        else:
            self.dev.writeSPISingle( 0x000C, 0x0001)
        val = 1
        i = 0
        while (val != 0 and i < 10):
            time.sleep(0.01)
            val = self.dev.readSPISingle(0x000C, 1)
            i = i + 1
        
        self.bProgrammed = True



    def StopSensor(self):
        """
        Command to stop sensor.
        Inputs:
            none

        Outputs:
            none

        """
        i = 0
        if( self.bProgrammed == True):
            # safely stop the camera
            self.dev.writeSPISingle( 0x000C, 0x0002)
            regData = self.dev.readSPISingle( 0x000C, 1)
            
            while( regData[0] != 0x0 and i<20):
                time.sleep(0.01)
                regData = self.dev.readSPISingle( 0x000C, 1)
                i = i + 1
        if( i == 20):
            print("Error stopping camera")

    def QuickStopSensor(self):
        """
        Command to stop sensor without checks 
        Inputs:
            none 

        Outputs:
            none 

        """
        # self.dev.stopSensor()
        self.dev.writeSPISingle( 0x000C, 0x0002)
        regData = self.dev.readSPISingle( 0x000C, 1)
        i=0
            
        while( regData[0] != 0x0 and i<20):
            time.sleep(0.01)
            regData = self.dev.readSPISingle( 0x000C, 1)
            i = i + 1
        if( i == 20):
            print("Error stopping camera")

        self.dev.stopSensor()



    def regread(self, addr, length=1):
        """
        Read a register. If address lies between 0x0E00 and 0x0EFE,
        writes 0x0000 to 0x0014 beforehand.
        Inputs:
            addr: register address in hex or decimal 
            length: number of registers to read 

        Outputs:
            register value 

        """
        if addr >= 0x0E00 and addr <= 0x0EFE:
            self.dev.writeSPISingle(0x0014, 0x0000)
            # print('did 0014<-0000')

        if length == 1 or length == '':
            return self.dev.readSPISingle(addr, 1)[0]
        else:
            return self.dev.readSPISingle(addr, length)

    def regwrite(self, addr, val, mode = ''):
        """
        Write to a register. If address lies between 0x0E00 and 0x0EFE,
        writes 0x0000 to 0x0014 beforehand.
        Inputs:
            addr: register address in hex or decimal 
            val: register value in hex or decimal 

        Outputs:
            status of write 

        """
        #print(val , 'writing to', addr)
        if mode == '':
            if isinstance(val, np.uint16) or isinstance(val, int):
                #print('stage1')
                if (addr in s1) or (addr in s2): # Analog registers
                    status = 0xFFFF
                    i = 10
                    while (((status >> 3) & 1 == 1) or ((status >> 11) & 1 == 1))and i > 0:
                        status = self.dev.readSPISingle(0x0150, 1) # Check if regif is busy
                        #print('status1-->',status)
                        i = i - 1
                    self.dev.writeSPISingle(addr, val)
                    if addr in s1:
                        self.dev.writeSPISingle(0x0150, 0x0105)
                        #print('sdsdsdf')
                    else:
                        self.dev.writeSPISingle(0x0150, 0x0501)
                    status = 0xFFFF
                    i = 10
                    while (((status >> 3) & 1 == 1) or ((status >> 11) & 1 == 1))and i > 0:
                        status = self.dev.readSPISingle(0x0150, 1) # Check if regif is busy
                        #print('status2-->',status)
                        i = i - 1
                
                elif addr >= 0x0E00 and addr <= 0x0EFE:
                    self.dev.writeSPISingle(0x0014, 0x0000)
                    #print('did 0014<-0000')
                    self.dev.writeSPISingle(addr, val)

                else:
                    self.dev.writeSPISingle(addr, val)
                # ret_val = self.dev.readSPISingle(addr, 1)[0]
                return True
            else:
                self.dev.writeSPISingle(addr, val)
                return True

        elif mode == 'ignore':
            self.dev.writeSPISingle(addr, val)
            return True

        else:
            return False

    def testBoardADC(self):
        """
        Function to check if the onboard ADC is functioning correctly
        Inputs:
            none 

        Outputs:
            status: staus of the test 

        """
        print('Testing Board ADC...')
        try:
            logging.info('Testing Board ADC...')
        except:
            pass

        success = self.dac_amptest0_set(158)
        self.error_assert(success, "Cannot set dac_amptest0")
        success = self.dac_amptest1_set(205)
        self.error_assert(success, "Cannot set dac_amptest1")
        success = self.dac_cmref_set(255)
        self.error_assert(success, "Cannot set dac_cmref")
        loop = 1
        status = False
        while(not(status) and loop <= 10):
            b1 = self.CamMeasureAnaTestVoltage('DACwithSF_AMPTEST1-AMPTEST0')
            print("Attempt", loop)
            print("Set voltage: 0.48V. Measured voltage:", str(np.round(b1,4)) + 'V')
            try:
                logging.info("Attempt " + str(loop))
                logginh.info("Set voltage: 0.48V. Measured voltage:", str(np.round(b1,4)) + 'V')
            except:
                pass

            if b1 > 0.35 and b1 < 0.6:
                status = True
            loop = loop + 1

        if status:
            print('Board ADC working correctly')
            try:
                logging.info('Board ADC working correctly')
            except:
                pass

        if status == False:
            raise Exception("ERROR: Either DACwithSF_AMPTEST1-AMPTEST0 or onboard ADC is incorrect")

        return status


    def ICQSensorPowerUp(self, tests_failed, limits = None, ICQ_dict=None):
        """
        Implements ICQ + sensor initialization sequence enabling the power regulators 
            and making functional checks.
        Inputs:
            tests_failed: number of tests failed before the startup 
            limits: minimum and maximum thresholds 
            ICQ_dist: results of the tests done prior 

        Outputs:
            0
            ICQ_dict: result of the powerup 
            tests_failed: number of tests failed 

        """
        global Vmain_pre_test
        global Vmain_static_test
        global Vmain_on_test
        global Vsys_pre_test
        global Vsys_static_test
        global Vsys_on_test
        global Vdepth_pre_test
        global Vdepth_static_test
        global Vdepth_on_test
        global Vaux_pre_test
        global Vaux_static_test
        global Vaux_on_test
        global Imain_pre_test
        global Imain_static_test
        global Imain_on_test
        global Isys_pre_test
        global Isys_static_test
        global Isys_on_test
        global Idepth_pre_test
        global Idepth_static_test
        global Idepth_on_test
        global Iaux_pre_test
        global Iaux_static_test
        global Iaux_on_test

        Vmain_pre_min = limits['Vmain_pre'][0] #Vmain can't be disabled currently
        Vmain_pre_max = limits['Vmain_pre'][1]
        Imain_pre_min = limits['Imain_pre'][0]
        Imain_pre_max = limits['Imain_pre'][1]
        Vsys_pre_min = limits['Vsys_pre'][0]
        Vsys_pre_max = limits['Vsys_pre'][1]
        Isys_pre_min = limits['Isys_pre'][0]
        Isys_pre_max = limits['Isys_pre'][1]
        Vdepth_pre_min = limits['Vdepth_pre'][0]
        Vdepth_pre_max = limits['Vdepth_pre'][1]
        Idepth_pre_min = limits['Idepth_pre'][0]
        Idepth_pre_max = limits['Idepth_pre'][1]
        Vaux_pre_min = limits['Vaux_pre'][0]
        Vaux_pre_max = limits['Vaux_pre'][1]
        Iaux_pre_min = limits['Iaux_pre'][0]
        Iaux_pre_max = limits['Iaux_pre'][1]

        Vmain_static_min = limits['Vmain_static'][0]
        Vmain_static_max = limits['Vmain_static'][1]
        Imain_static_min = limits['Imain_static'][0]
        Imain_static_max = limits['Imain_static'][1]
        Vsys_static_min = limits['Vsys_static'][0]
        Vsys_static_max = limits['Vsys_static'][1]
        Isys_static_min = limits['Isys_static'][0]
        Isys_static_max = limits['Isys_static'][1]
        Vdepth_static_min = limits['Vdepth_static'][0]
        Vdepth_static_max = limits['Vdepth_static'][1]
        Idepth_static_min = limits['Idepth_static'][0]
        Idepth_static_max = limits['Idepth_static'][1]
        Vaux_static_min = limits['Vaux_static'][0]
        Vaux_static_max = limits['Vaux_static'][1]
        Iaux_static_min = limits['Iaux_static'][0]
        Iaux_static_max = limits['Iaux_static'][1]

        #self.dev.writeI2C(I2C_MALKA, AD5593R_ID,[AD5593R_DAC_SET|VAUX_ADJ,0x9C, 0x17]) # MZ1: Added VAUX DAC set for 18V  
        self.dev.writeI2C(I2C_MALKA, AD5593R_ID,[AD5593R_DAC_SET|VAUX_ADJ,0x9A, 0x2C]) # MZ1: Added VAUX DAC set for 22V 

        self.dev.setGpio( CAM_RESET_N, 0) # reset camera
        self.dev.setGpio( LMZ_EN, 0) # disable Vmain
        time.sleep(0.01) # wait 10 ms
        [ Vmain_pre, Imain_pre, Pmain_pre] = self.PowerMonitor( VMAIN_DEV_ID) # Check Vmain before enabled
        Imain_pre = Imain_pre * 1000 #Convert to mA

        Vmain_pre_pass, fail = ICQtest(Vmain_pre, Vmain_pre_min, Vmain_pre_max)
        tests_failed = tests_failed + fail
        Imain_pre_pass, fail = ICQtest(Imain_pre, Imain_pre_min, Imain_pre_max)
        tests_failed = tests_failed + fail

        print(" Vmain_pre = " + str(Vmain_pre), Vmain_pre_pass)
        print(" Imain_pre = " + str(Imain_pre), Imain_pre_pass)

        self.dev.setGpio( CAM_RESET_N, 0) # reset camera
        self.dev.setGpio( LMZ_EN, 1) # enable Vmain
        time.sleep(0.01) # wait 10 ms
        [ Vmain_static, Imain_static, Pmain_static] = self.PowerMonitor( VMAIN_DEV_ID) # Check Vmain
        Imain_static = Imain_static * 1000 #Convert to mA

        Vmain_static_pass, fail = ICQtest(Vmain_static, Vmain_static_min, Vmain_static_max)
        tests_failed = tests_failed + fail
        Imain_static_pass, fail = ICQtest(Imain_static, Imain_static_min, Imain_static_max)
        tests_failed = tests_failed + fail

        print(" Vmain_static = " + str(Vmain_static), Vmain_static_pass)
        print(" Imain_static = " + str(Imain_static), Imain_static_pass)
        #print(" Pmain_static = " + str(Pmain_static))

        time.sleep(0.2) # wait 100 ms
        [ Vdepth_pre, Idepth_pre, Pdepth_pre] = self.PowerMonitor( VDEPTH_DEV_ID) # Check Vdepth before enabled
        Idepth_pre = Idepth_pre * 1000 #Convert to mA

        Vdepth_pre_pass, fail = ICQtest(Vdepth_pre, Vdepth_pre_min, Vdepth_pre_max)
        tests_failed = tests_failed + fail
        Idepth_pre_pass, fail = ICQtest(Idepth_pre, Idepth_pre_min, Idepth_pre_max)
        tests_failed = tests_failed + fail

        print(" Vdepth_pre = " + str(Vdepth_pre), Vdepth_pre_pass)
        print(" Idepth_pre = " + str(Idepth_pre), Idepth_pre_pass)

        self.dev.setGpio( PS_V5VVPS3, 1) # enable TPS61230A producint 5V2 to drive Vdepth
        time.sleep(0.1) # wait 100 ms
        [ Vdepth_static, Idepth_static, Pdepth_static] = self.PowerMonitor( VDEPTH_DEV_ID) # Check Vdepth
        Idepth_static = Idepth_static * 1000 #Convert to mA

        Vdepth_static_pass, fail = ICQtest(Vdepth_static, Vdepth_static_min, Vdepth_static_max)
        tests_failed = tests_failed + fail
        Idepth_static_pass, fail = ICQtest(Idepth_static, Idepth_static_min, Idepth_static_max)
        tests_failed = tests_failed + fail

        print(" Vdepth_static = " + str(Vdepth_static), Vdepth_static_pass)
        print(" Idepth_static = " + str(Idepth_static), Idepth_static_pass)
        #print(" Pdepth_static = " + str(Pdepth_static))

        self.dev.setGpio( PP_DEPTH, 1) # enable XC6222 and Vdepth

        time.sleep(0.2) # wait 100 ms
        [ Vsys_pre, Isys_pre, Psys_pre] = self.PowerMonitor( VSYS_DEV_ID)
        Isys_pre = Isys_pre * 1000 #Convert to mA

        Vsys_pre_pass, fail = ICQtest(Vsys_pre, Vsys_pre_min, Vsys_pre_max)
        tests_failed = tests_failed + fail
        Isys_pre_pass, fail = ICQtest(Isys_pre, Isys_pre_min, Isys_pre_max)
        tests_failed = tests_failed + fail

        print(" Vsys_pre = " + str(Vsys_pre), Vsys_pre_pass)
        print(" Isys_pre = " + str(Isys_pre), Isys_pre_pass)

        self.dev.setGpio( PS_VAUXVPS1, 1) # enable ADP196
        self.dev.setGpio( PS_VAUXVPS2, 1) # enable Vsys
        time.sleep( 0.1) # wait 4 ms
        [ Vsys_static, Isys_static, Psys_static] = self.PowerMonitor( VSYS_DEV_ID)
        Isys_static = Isys_static * 1000 #Convert to mA

        Vsys_static_pass, fail = ICQtest(Vsys_static, Vsys_static_min, Vsys_static_max)
        tests_failed = tests_failed + fail
        Isys_static_pass, fail = ICQtest(Isys_static, Isys_static_min, Isys_static_max)
        tests_failed = tests_failed + fail

        print(" Vsys_static = " + str(Vsys_static), Vsys_static_pass)
        print(" Isys_static = " + str(Isys_static), Isys_static_pass)
        #print(" Psys_static = " + str(Psys_static))
        time.sleep(0.01) # wait 10 ms

        self.dev.setGpio( DEPTH_VAUX_RUN, 0)
        self.dev.setGpio( PS_VAUXVPS1, 0)
        time.sleep(0.1) # wait 100 ms
        [ Vaux_pre, Iaux_pre, Paux_pre] = self.PowerMonitor( VAUX_DEV_ID)
        Iaux_pre = Iaux_pre * 1000 #Convert to mA

        Vaux_pre_pass, fail = ICQtest(Vaux_pre, Vaux_pre_min, Vaux_pre_max)
        tests_failed = tests_failed + fail
        Iaux_pre_pass, fail = ICQtest(Iaux_pre, Iaux_pre_min, Iaux_pre_max)
        tests_failed = tests_failed + fail
        
        print(" Vaux_pre = " + str( Vaux_pre), Vaux_pre_pass)
        print(" Iaux_pre = " + str( Iaux_pre), Iaux_pre_pass)

        self.dev.setGpio( PS_VAUXVPS1, 1)
        self.dev.setGpio( DEPTH_VAUX_RUN, 1)
        time.sleep(0.1) # wait 10 ms
        [ Vaux_static, Iaux_static, Paux_static] = self.PowerMonitor( VAUX_DEV_ID)
        Iaux_static = Iaux_static * 1000 #Convert to mA

        Vaux_static_pass, fail = ICQtest(Vaux_static, Vaux_static_min, Vaux_static_max)
        tests_failed = tests_failed + fail
        Iaux_static_pass, fail = ICQtest(Iaux_static, Iaux_static_min, Iaux_static_max)  
        tests_failed = tests_failed + fail

        print(" Vaux_static = " + str( Vaux_static), Vaux_static_pass)
        print(" Iaux_static = " + str( Iaux_static), Iaux_static_pass)
        #print(" Paux = " + str( Paux))      

        Vmain_pre_test = ['Vmain_pre_V', Vmain_pre, str(Vmain_pre_pass), Vmain_pre_min, Vmain_pre_max]
        ICQ_dict['Vmain_pre_V'] = Vmain_pre_test[1:]
        Imain_pre_test = ['Imain_pre_mA', Imain_pre, str(Imain_pre_pass), Imain_pre_min, Imain_pre_max]
        ICQ_dict['Imain_pre_mA'] = Imain_pre_test[1:]
        Vsys_pre_test = ['Vsys_pre_V', Vsys_pre, str(Vsys_pre_pass), Vsys_pre_min, Vsys_pre_max]
        ICQ_dict['Vsys_pre_V'] = Vsys_pre_test[1:]
        Isys_pre_test = ['Isys_pre_mA', Isys_pre, str(Isys_pre_pass), Isys_pre_min, Isys_pre_max]
        ICQ_dict['Isys_pre_mA'] = Isys_pre_test[1:]
        Vdepth_pre_test = ['Vdepth_pre_V', Vdepth_pre, str(Vdepth_pre_pass), Vdepth_pre_min, Vdepth_pre_max]
        ICQ_dict['Vdepth_pre_V'] = Vdepth_pre_test[1:]
        Idepth_pre_test = ['Idepth_pre_mA', Idepth_pre, str(Idepth_pre_pass), Idepth_pre_min, Idepth_pre_max]
        ICQ_dict['Idepth_pre_mA'] = Idepth_pre_test[1:]
        Vaux_pre_test = ['Vaux_pre_V', Vaux_pre, str(Vaux_pre_pass), Vaux_pre_min, Vaux_pre_max]
        ICQ_dict['Vaux_pre_V'] = Vaux_pre_test[1:]
        Iaux_pre_test = ['Iaux_pre_mA', Iaux_pre, str(Iaux_pre_pass), Iaux_pre_min, Iaux_pre_max]
        ICQ_dict['Iaux_pre_mA'] = Iaux_pre_test[1:]
        
        Vmain_static_test = ['Vmain_static_V', Vmain_static, str(Vmain_static_pass), Vmain_static_min, Vmain_static_max]
        ICQ_dict['Vmain_static_V'] = Vmain_static_test[1:]
        Imain_static_test = ['Imain_static_mA', Imain_static, str(Imain_static_pass), Imain_static_min, Imain_static_max]
        ICQ_dict['Imain_static_mA'] = Imain_static_test[1:]
        Vsys_static_test = ['Vsys_static_V', Vsys_static, str(Vsys_static_pass), Vsys_static_min, Vsys_static_max]
        ICQ_dict['Vsys_static_V'] = Vsys_static_test[1:]
        Isys_static_test = ['Isys_static_mA', Isys_static, str(Isys_static_pass), Isys_static_min, Isys_static_max]
        ICQ_dict['Isys_static_mA'] = Isys_static_test[1:]
        Vdepth_static_test = ['Vdepth_static_V', Vdepth_static, str(Vdepth_static_pass), Vdepth_static_min, Vdepth_static_max]
        ICQ_dict['Vdepth_static_V'] = Vdepth_static_test[1:]
        Idepth_static_test = ['Idepth_static_mA', Idepth_static, str(Idepth_static_pass), Idepth_static_min, Idepth_static_max]
        ICQ_dict['Idepth_static_mA'] = Idepth_static_test[1:]
        Vaux_static_test = ['Vaux_static_V', Vaux_static, str(Vaux_static_pass), Vaux_static_min, Vaux_static_max]
        ICQ_dict['Vaux_static_V'] = Vaux_static_test[1:]
        Iaux_static_test = ['Iaux_static_mA', Iaux_static, str(Iaux_static_pass), Iaux_static_min, Iaux_static_max]
        ICQ_dict['Iaux_static_mA'] = Iaux_static_test[1:]

        if(tests_failed == 0):
            print("Static Malka Supplies PASSED")
            ICQ_dict['info']['Malka_pass'] = 1 
        else:
            print("Static Malka Supplies FAILED")
            ICQ_dict['info']['Malka_pass'] = 0 

        # Wait for CAM_SHUTDOWN_N to go high indicating that everything is ok
        retries = 10
        while( retries > 0):
            if (self.module_class == 'Walden_R1' or 'Crosby'):
                cam_shutdown = 1 # MZ: Walden does not have shutdown output to controller (connector pin left floating in rev.1)
            else:
                cam_shutdown = self.dev.getGpio( CAM_SHUTDOWN_N)
            if( cam_shutdown == 1):
                break
            else:
                retries = retries - 1

        # if( retries == 0):
        #     print( "Error during camera initialization")
        #     self.dev.setGpio( DEPTH_VAUX_RUN, 0)
        #     self.dev.setGpio( PS_VAUXVPS2, 0)
        #     self.dev.setGpio( PS_VAUXVPS1, 0)
        #     self.dev.setGpio( PS_V5VVPS3, 0)
        #     self.dev.setGpio( LMZ_EN, 0)
        #     return -1

        # wait 2 ms
        time.sleep( 0.002)
        self.dev.setGpio( CAM_RESET_N, 1)       

        return [0, ICQ_dict, tests_failed]

    def ICQSensorStart(self, Mode=3, fps=10, cfg = "", file_name = "", 
                        limits = None, ICQ_dict=None, ICQ_mode = 'no_laser'):
        """
        Starts the sensor, performs ICQ and saves it to csv file
        Inputs:
            mode: camera mode 
            fps: frequency per second 
            cfg: name of cfg file 
            file_name: name of csv file to store the results 
            limits: minimum and maximum thresholds 
            ICQ_dict: dictionary containing the results of prior tests 
            ICQ_mode: whether to enable laser or not 

        Outputs:
            0 
            ICQ_dict: result in a dictionary 

        """

        tests_failed = 0
        global Vmain_on_test
        global Vsys_on_test
        global Vdepth_on_test
        global Vaux_on_test
        global Imain_on_test
        global Isys_on_test
        global Idepth_on_test
        global Iaux_on_test

        ret = self.dev.openDevice( Mode, -1)
        if( ret < 0):
            print("Error opening the camera")
            return -2
        #self.dev.writeI2C(AD5593R_ID,[AD5593R_DAC_SET|VAUX_ADJ,0x98, 0xBB]) # MZ1: Added VAUX DAC set for 25V
        # self.dev.writeI2C(AD5593R_ID,[AD5593R_DAC_SET|VAUX_ADJ,0x97, 0xCE]) # DAC set for 27V
        #self.dev.writeI2C(AD5593R_ID,[AD5593R_DAC_SET|VAUX_ADJ,0x97, 0x4B]) # DAC set for 28V
        #self.dev.writeI2C(AD5593R_ID,[AD5593R_DAC_SET|VAUX_ADJ,0x97, 0x09]) # DAC set for 28.5V
        #self.dev.writeI2C(AD5593R_ID,[AD5593R_DAC_SET|VAUX_ADJ,0x9E, 0x13]) # DAC set for ~13.8V

        [ret, ICQ_dict, tests_failed] = self.ICQSensorPowerUp(tests_failed, limits = limits, ICQ_dict=ICQ_dict)
        if( ret < 0):
            print("Error during sensor power up. Please restart your system.")
            sys.exit()

        self.Mode = Mode
        self._setInternalClassMode()

        self.fps = fps
        self.dev.setCameraFrameRate(fps)

        self.dev.programCfg(cfg)
        #self.programCSV("process_data_Legacy.csv")

        
        if( Mode != 3):
            #self.changeMode( self.Mode)
            self.dev.writeSPISingle( 0x0200, self.Mode)
        
        self.dev.startSensor()
        print(" Camera running... ")

        # Send start command to camera
        self.dev.writeSPISingle( 0x000C, 0x00C5)

        Vmain_on_min = limits['Vmain_on'][0]
        Vmain_on_max = limits['Vmain_on'][1]
        Imain_on_min = limits['Imain_on'][0]
        Imain_on_max = limits['Imain_on'][1]
        Vsys_on_min = limits['Vsys_on'][0]
        Vsys_on_max = limits['Vsys_on'][1]
        Isys_on_min = limits['Isys_on'][0]
        Isys_on_max = limits['Isys_on'][1]
        Vdepth_on_min = limits['Vdepth_on'][0]
        Vdepth_on_max = limits['Vdepth_on'][1]
        Idepth_on_min = limits['Idepth_on'][0]
        Idepth_on_max = limits['Idepth_on'][1]
        Vaux_on_min = limits['Vaux_on'][0]
        Vaux_on_max = limits['Vaux_on'][1]
        Iaux_on_min = limits['Iaux_on'][0]
        Iaux_on_max = limits['Iaux_on'][1]

        #Measure supply rails after enabling camera
        time.sleep(0.01) # wait 10 ms
        [ Vmain_on, Imain_on, Pmain_on] = self.PowerMonitor( VMAIN_DEV_ID) # Check Vmain
        Imain_on = Imain_on * 1000 #Convert to mA
        
        Vmain_on_pass, fail = ICQtest(Vmain_on, Vmain_on_min, Vmain_on_max)
        tests_failed = tests_failed + fail
        Imain_on_pass, fail = ICQtest(Imain_on, Imain_on_min, Imain_on_max)
        tests_failed = tests_failed + fail

        print(" Vmain-on = " + str(Vmain_on), Vmain_on_pass)
        print(" Imain-on = " + str(Imain_on), Imain_on_pass)
        #print(" Pmain_on = " + str(Pmain_on))

        [ Vdepth_on, Idepth_on, Pdepth_on] = self.PowerMonitor( VDEPTH_DEV_ID) # Check Vdepth
        Idepth_on = Idepth_on * 1000 #Convert to mA

        Vdepth_on_pass, fail = ICQtest(Vdepth_on, Vdepth_on_min, Vdepth_on_max)
        tests_failed = tests_failed + fail
        Idepth_on_pass, fail = ICQtest(Idepth_on, Idepth_on_min, Idepth_on_max)
        tests_failed = tests_failed + fail

        print(" Vdepth-on = " + str(Vdepth_on), Vdepth_on_pass)
        print(" Idepth-on = " + str(Idepth_on), Idepth_on_pass)
        #print(" Pdepth_on = " + str(Pdepth_on))

        [ Vsys_on, Isys_on, Psys_on] = self.PowerMonitor( VSYS_DEV_ID)
        Isys_on = Isys_on * 1000 #Convert to mA

        Vsys_on_pass, fail = ICQtest(Vsys_on, Vsys_on_min, Vsys_on_max)
        tests_failed = tests_failed + fail
        Isys_on_pass, fail = ICQtest(Isys_on, Isys_on_min, Isys_on_max)
        tests_failed = tests_failed + fail

        print(" Vsys-on = " + str(Vsys_on), Vsys_on_pass)
        print(" Isys-on = " + str(Isys_on), Isys_on_pass)
        #print(" Psys_on = " + str(Psys_on))
        time.sleep( 0.01) # wait 10 ms

        [ Vaux_on, Iaux_on, Paux_on] = self.PowerMonitor( VAUX_DEV_ID)
        Iaux_on = Iaux_on * 1000 #Convert to mA

        Vaux_on_pass, fail = ICQtest(Vaux_on, Vaux_on_min, Vaux_on_max)
        tests_failed = tests_failed + fail
        Iaux_on_pass, fail = ICQtest(Iaux_on, Iaux_on_min, Iaux_on_max)
        tests_failed = tests_failed + fail

        print(" Vaux-on = " + str( Vaux_on), Vaux_on_pass)
        print(" Iaux-on = " + str( Iaux_on), Iaux_on_pass)
        #print(" Paux_on = " + str( Paux_on))
   
        
        Vmain_on_test = ['Vmain-on_V', Vmain_on, str(Vmain_on_pass), Vmain_on_min, Vmain_on_max]
        ICQ_dict['Vmain-on_V'] = Vmain_on_test[1:]
        Imain_on_test = ['Imain-on_mA', Imain_on, str(Imain_on_pass), Imain_on_min, Imain_on_max]
        ICQ_dict['Imain-on_mA'] = Imain_on_test[1:]
        Vsys_on_test = ['Vsys-on_V', Vsys_on, str(Vsys_on_pass), Vsys_on_min, Vsys_on_max]
        ICQ_dict['Vsys-on_V'] = Vsys_on_test[1:]
        Isys_on_test = ['Isys-on_mA', Isys_on, str(Isys_on_pass), Isys_on_min, Isys_on_max]
        ICQ_dict['Isys-on_mA'] = Isys_on_test[1:]
        Vdepth_on_test = ['Vdepth-on_V', Vdepth_on, str(Vdepth_on_pass), Vdepth_on_min, Vdepth_on_max]
        ICQ_dict['Vdepth-on_V'] = Vdepth_on_test[1:]
        Idepth_on_test = ['Idepth-on_mA', Idepth_on, str(Idepth_on_pass), Idepth_on_min, Idepth_on_max]
        ICQ_dict['Idepth-on_mA'] = Idepth_on_test[1:]
        Vaux_on_test = ['Vaux-on_V', Vaux_on, str(Vaux_on_pass), Vaux_on_min, Vaux_on_max]
        ICQ_dict['Vaux-on_V'] = Vaux_on_test[1:]
        Iaux_on_test = ['Iaux-on_mA', Iaux_on, str(Iaux_on_pass), Iaux_on_min, Iaux_on_max]
        ICQ_dict['Iaux-on_mA'] = Iaux_on_test[1:]

        
        # check if useq works
        errorCode = 0
        time.sleep(0.1) 
        for i in range(10):
            data = self.dev.readSPISingle( 0x0256,1 )
            data = data[0]
            print("data = " + hex(data))
            if data == 0:
                errorCode = self.dev.readSPISingle( 0x0032, 1)[0]
                print( "Error = " + hex(errorCode))

            ADC_LSB = 0.000125
            ADC_REF = 2.5 #Vref
            ADC_BITS = 2**12 #12 bit ADC

            VDDA1_min = limits['VDDA1'][0]
            VDDA1_max = limits['VDDA1'][1]
            VHIGH_min = limits['VHIGH'][0]
            VHIGH_max = limits['VHIGH'][1]
            VLOW_min = limits['VLOW'][0]
            VLOW_max = limits['VLOW'][1]
            VLD_min = limits['VLD'][0]
            VLD_max = limits['VLD'][1]

            #Dummy ADC read to fix bug
            ADC_code = self.dev.getADCvalue(0)

            time.sleep(0.1)
            self.dev.writeSPISingle( 0x0160, 0x0007) #Diff V1p2
            time.sleep(0.1)
            ADC_code = self.dev.getADCvalue(0)
            VDDA1 = ADC_code * ADC_LSB
            VDDA1_pass, fail = ICQtest(VDDA1, VDDA1_min, VDDA1_max)
            tests_failed = tests_failed + fail
            print(" VDDA1 = " + str(VDDA1), VDDA1_pass)

            self.dev.writeSPISingle( 0x0160, 0x0008) #SE Vhigh
            time.sleep(0.1)
            ADC_code = self.dev.getADCvalue(4)
            VHIGH = ADC_code * ADC_LSB
            VHIGH_pass, fail = ICQtest(VHIGH, VHIGH_min, VHIGH_max)
            tests_failed = tests_failed + fail
            print(" VHIGH = " + str(VHIGH), VHIGH_pass)

            self.dev.writeSPISingle( 0x0160, 0x0008) #SE Vlow
            time.sleep(0.1)
            ADC_code = self.dev.getADCvalue(5)
            VLOW = ADC_code * ADC_LSB
            VLOW_pass, fail = ICQtest(VLOW, VLOW_min, VLOW_max)
            tests_failed = tests_failed + fail
            print(" VLOW = " + str(VLOW), VLOW_pass)

            VDDA1_test = ['VDDA1_V', VDDA1, str(VDDA1_pass), VDDA1_min, VDDA1_max]
            ICQ_dict['VDDA1_V'] = VDDA1_test[1:]
            VHIGH_test = ['VHIGH_V', VHIGH, str(VHIGH_pass), VHIGH_min, VHIGH_max]
            ICQ_dict['VHIGH_V'] = VHIGH_test[1:]
            VLOW_test = ['VLOW_V', VLOW, str(VLOW_pass), VLOW_min, VLOW_max]
            ICQ_dict['VLOW_V'] = VLOW_test[1:]

            if ICQ_mode == 'laser':
                self.dev.writeI2C(I2C_MALKA, AD5593R_ID, [0x04, 0x00, 0x10])
                self.dev.writeI2C(I2C_MALKA, AD5593R_ID, [0x02, 0x00, 0x10])
                self.dev.writeI2C(I2C_MALKA, AD5593R_ID, [0x40])
                VLD = self.dev.readI2C(I2C_MALKA, AD5593R_ID, 2)
                VLD[0] = VLD[0] & 0x0F #ADC conversion is 12 LSBs (bits 13-15 = ADC channel info) 
                VLD = VLD[0] *256 + VLD[1] #unpack bytes
                VLD = VLD * (ADC_REF / ADC_BITS) #convert ADC code to voltage
                VLD = VLD * 5 #Resistor divider schematic - needs to be corrected
                VLD_pass, fail = ICQtest(VLD, VLD_min, VLD_max)
                tests_failed = tests_failed + fail
                print(" VLD = " + str(VLD), VLD_pass)
                VLD_test = ['VLD_V', VLD, str(VLD_pass), VLD_min, VLD_max]
                ICQ_dict['VLD_V'] = VLD_test[1:]

            elif ICQ_mode == 'no_laser':
                VLD_test = ['VLD_V', 0, 'NA', -0.1, 0.1]
                ICQ_dict['VLD_V'] = VLD_test[1:]

            else:
                raise Exception("\n\nERROR: Invalid ICQ mode\n\n")
                  

            if(tests_failed == 0):
                print("Device PASSED")
                device_passed = "PASSED"
                ICQ_dict['info']['Pass'] = 1
            else:
                print("Device FAILED")
                device_passed = "FAILED"
                print("Number of tests failed = ", tests_failed)
                ICQ_dict['info']['Pass'] = 0


            Device_test = ['Pass/Fail', tests_failed, str(device_passed), 0, 0]

            # #data directory path
            # DATA_PATH = './datalogs'
            # #data file name
            # DATA_NAME = 'IQC_log'
            # #todays date
            

            time.sleep(0.1)
            if( data == 0x02):
                print("uSeq running")
                break
            else:
                errorCode = self.dev.readSPISingle( 0x0032, 1)[0]
                print( "Error = " + hex(errorCode))
                
        if( errorCode != 0x0 or data == 0x00):
            sys.exit()
            
        # Kick-off camera stream
        self.bProgrammed = True
        print("camera programmed")

        self.QuickStopSensor()

        now = datetime.now()
        date_string = now.date().strftime("%Y_%m_%d")
        time_string = now.time().strftime("_%H_%M_%S")

        return [tests_failed, ICQ_dict]


    def CamMeasureAnaTestVoltage(self, SOURCE, N_AVERAGES = 1):
        """
        Measures a voltage from ADSD3100 Analog Test mux using on board ADC.  
        Inputs:
            - source: Internal ADSD3100 voltage to measure.  Valid options are: 
               'DAC_TX1' 
               'DAC_TX2' 
               'DAC_AMPTEST1' 
               'DAC_BIAS1' 
               'DAC_BIAS2' 
               'DAC_SATREF' 
               'DAC_VREST' 
               'DAC_AMPTEST0' 
               'DAC_CMREF' 
               'DACwithSF_X-Y',                       
                 X,Y valid options are {AMPTEST0,AMPTEST1,CMREF,GND} 
                 where X = adc_muxp_in/amp_testseln/ATP 
                   and Y = adc_muxn_in/amp_testselp/ATN 
               'comp_vref0' 
               'comp_vref1' 
               'comp_vref2' 
               'amp_vcm'                              
               'AnaTestMux_0' ... 'AnaTestMux_31' 
        
            - num_averages (optional). How many times to read the voltage and average 
        
        Outputs:
            - voltage (optional). Measured voltage 
        """

        # voltage = CamMeasureAnaTestVoltage(source)
        # voltage = CamMeasureAnaTestVoltage(source, num_averages)
        
        # Measures a voltage from ADSD3100 Analog Test mux using on board ADC.  On ADSD3100
        # BUB this requires jumpers on J29 pins 2-3 (ATP_ADC) and J31 pins 2-3 (ATN_ADC)
        

        val = self.regread(0x0026)
        time.sleep(0.1)
        success = True
        val = int(val) & 0x0800 #check bit 11 to see if useq is still running

        if(val!=0):
            print("Retry")
            time.sleep(0.1)
            val = self.regread(0x0026)
            time.sleep(0.1)
            print("Val =", val)
            val = int(val) & 0x0800 #check bit 11 to see if useq is still running
 
        #self.error_assert(success, 'Cannot communicate with chip')
        self.error_assert(not(val), 'Must stop uSeq before calling')
        ADC_config = dict()

        ADC_config['mux'] = 'AIN0-AIN1'

        val = self.regread(0x017C)

        origSettingADC = dict()

        origSettingADC['xosc_ctrl'] = int(val)
        origSettingADC['reg'] = dict()
        origSettingADC['reg']['addr'] = []
        origSettingADC['reg']['val'] = []

        val = int(val) | 0x0020
        s_ = self.regwrite(0x017C, val)
        # self.error_assert(success, 'Unable to write register 0x017C')

        val = self.regread(0x0146)
        origSettingADC['reg']['val'] = np.append(origSettingADC['reg']['val'], int(val))
        origSettingADC['reg']['addr'] = np.append(origSettingADC['reg']['addr'], 0x0146)
        val = int(val) & 0xFF7F #power up bandgap
        success = success and self.regwrite(0x0146, val)
        self.error_assert(success, 'Unable to power up bandgap')

        val = self.regread(0x0160) # original state of ana_test_mux_s1
        #self.error_assert(success, 'Unable to read ana_test_mux_s1')
        origSettingADC['reg']['addr'] = np.append(origSettingADC['reg']['addr'], 0x0160)
        origSettingADC['reg']['val'] = np.append(origSettingADC['reg']['val'], val)

        val = self.regread(0x0128) # original state of dac_ctrl2
        #self.error_assert(success, 'Unable to read dac_ctrl2')
        origSettingADC['reg']['addr'] = np.append(origSettingADC['reg']['addr'], 0x0128)
        origSettingADC['reg']['val'] = np.append(origSettingADC['reg']['val'], val)

        val = self.regread(0x0014) # original state of digPwrDown
        #self.error_assert(success, 'Unable to read digPwrDown')
        origSettingADC['reg']['addr'] = np.append(origSettingADC['reg']['addr'], 0x0014)
        origSettingADC['reg']['val'] = np.append(origSettingADC['reg']['val'], val)

        if SOURCE.startswith('DAC_'):
            dacNames = ['DAC_TX1','DAC_TX2','DAC_AMPTEST1','DAC_BIAS1','DAC_BIAS2','DAC_SATREF','DAC_VREST','DAC_AMPTEST0','DAC_CMREF']
            try:
                dacIdx = dacNames.index(SOURCE)
            except:
                raise Exception('Cannot measure voltage. Unknown DAC value')

            success = self.regwrite(0x0128, 0x0000)
            self.error_assert(success, 'Unable to power up DACs')

            val = self.regread(0x012A)
            origSettingADC['reg']['addr'] = np.append(origSettingADC['reg']['addr'], 0x012A)
            origSettingADC['reg']['val'] = np.append(origSettingADC['reg']['val'], val)
            val = int(val) & 0xFF00
            val = val | int(dacIdx + 1) # =1 due to difference in indexing
            success = success and self.regwrite(0x012A, val)
            success = success and self.regwrite(0x0160, 0x0013)
            self.error_assert(success, 'Unable to configuire test mux')

        elif SOURCE.startswith('amp_vcm'):
            val = self.regread(0x0146)
            val = val & 0xFFCF
            s = self.regwrite(0x0146, val)
            success = self.regwrite(0x0160, 0x001B)

        elif SOURCE.startswith('DACwithSF_'):
            val = self.regread(0x0146)
            val = int(val) & 0xFFFD
            success = success and self.regwrite(0x0146, val)
            success = success and self.regwrite(0x0128, 0x0000)
            self.error_assert(success, 'Unable to power up DACs and column currents')

            tmp = SOURCE.split('_')
            DAC1 = tmp[1].split('-')[0]
            try: # if there is no DAC2
                DAC2 = tmp[1].split('-')[1]
            except:
                ADC_config['mux'] = 'AIN0-GND'
                DAC2 = DAC1

            if (DAC2 == 'GND' and DAC1 != 'GND'):
                ADC_config['mux'] = 'AIN0-GND'
                DAC2 = DAC1
            elif (DAC1 == 'GND'):
                ADC_config['mux'] = 'AIN1-GND'
                DAC1 = DAC2

            if DAC1.upper() == 'AMPTEST0':
                amptestselOverrideVal = int('0x2000', 0)
            elif DAC1.upper() == 'AMPTEST1':
                amptestselOverrideVal = int('0x3000', 0)
            elif DAC1.upper() == 'CMREF':
                amptestselOverrideVal = int('0x4000', 0)
            else:
                raise Exception('Unknown source for DAC selection [0]')

            if DAC2.upper() == 'AMPTEST0':
                amptestselOverrideVal = amptestselOverrideVal | int('0x0400', 0)
            elif DAC2.upper() == 'AMPTEST1':
                amptestselOverrideVal = amptestselOverrideVal | int('0x0420', 0)
            elif DAC2.upper() == 'CMREF':
                amptestselOverrideVal = amptestselOverrideVal | int('0x0800', 0)
            else:
                raise Exception('Unknown source for DAC selection [1]')

            # val = self.regread(0x0E10)
            # print('regread 0x0E10', hex(val))
            # origSettingADC['reg']['addr'] = np.append(origSettingADC['reg']['addr'], int('0x0E10', 0))
            # origSettingADC['reg']['val'] = np.append(origSettingADC['reg']['val'], val)

            # val = self.regread(0x0E18)
            # print('regread 0x0E18', hex(val))
            # origSettingADC['reg']['addr'] = np.append(origSettingADC['reg']['addr'], int('0x0E18', 0))
            # origSettingADC['reg']['val'] = np.append(origSettingADC['reg']['val'], val)

            # val = self.regread(0x0E00)
            # origSettingADC['reg']['addr'] = np.append(origSettingADC['reg']['addr'], int('0x0E00', 0))
            # origSettingADC['reg']['val'] = np.append(origSettingADC['reg']['val'], val)


            # s = self.regrmw(0x0014,0x0000,0x0001)
            # s = self.regrmw(0x0E00,0x0002,0x0003)
            # s = self.regrmw(0x0E18,0x7C20,0x7C20)
            # s = self.regrmw(0x0E10,amptestselOverrideVal,0x7C20)
            
            val = self.regread(0x0E10)
            origSettingADC['reg']['addr'] = np.append(origSettingADC['reg']['addr'], int('0x0E10', 0))
            origSettingADC['reg']['val'] = np.append(origSettingADC['reg']['val'], val)
            val = int(val) & int('0x83DF' ,0)
            val = val | amptestselOverrideVal
            s_ = self.regwrite(0x0E10, val) # >>>>> override
            #self.error_assert(success, 'Unable to overrideamp_testselp')

            val = self.regread(0x0E18)
            origSettingADC['reg']['addr'] = np.append(origSettingADC['reg']['addr'], int('0x0E18', 0))
            origSettingADC['reg']['val'] = np.append(origSettingADC['reg']['val'], val)
            val = int(val) | int('0x7C20' ,0)
            s_ = self.regwrite(0x0E18, val) # >>>>> override
            #self.error_assert(success, 'Unable to overrideamp_testselp')

            success = success and self.regwrite(0x0014, 0x0000)
            s, origSettingADC['ampMuxOrig'] = self.regreadburst(0x0E1C, 10, 'increment')
            ampMuxVal = [0x0200, 0x0008, 0x0200, 0x0008, 0x0200, 0x0008, 0x0200, 0x0008, 0x0200, 0x0008]
            s = self.regwriteburst(0x0E1C, ampMuxVal, 'increment')

            # val = self.regread(0x0160)
            # origSettingADC['reg']['addr'] = np.append(origSettingADC['reg']['addr'], int('0x0160', 0))
            # origSettingADC['reg']['val'] = np.append(origSettingADC['reg']['val'], val)

            s = self.regwrite(0x0160, 0x0012)
            self.error_assert(success, 'Unable to configuire test mux')
            # s = self.regwrite(0x0150, 0x0105)

        elif SOURCE.startswith('comp_v'):
            val = self.regread(0x0146)
            val = int(val) & int('0xFFEF',0)
            success = success and self.regwrite(0x0146, val)
            self.error_assert(success, 'Unable to power up col_comparator')

            if SOURCE.lower() == 'comp_vref0':
                success = success and self.regwrite(0x0160, 0x0014)
            elif SOURCE.lower() == 'comp_vref1':
                success = success and self.regwrite(0x0160, 0x0015)
            elif SOURCE.lower() == 'comp_vref2':
                success = success and self.regwrite(0x0160, 0x0016)
            else:
                raise Exception('Unknown source')

        elif SOURCE == 'amp_vcm':
            val = self.regread(0x0146)
            val = int(val) & int('0xFFCF',0)
            success = success and self.regwrite(0x0146, val)
            self.error_assert(success, 'Unable to power up col_comparator and AMP')

            success = success and self.regwrite(0x0160, 0x001B)

        elif SOURCE.startswith('AnaTestMux_'):
            ana_test_mux = int(SOURCE.split('_')[1])

            if np.isnan(ana_test_mux) or ana_test_mux > 31 or ana_test_mux < 0:
                print('Unknown source. The AnaTestMux_ option must include a mux value between 0-31\n')

            success = self.regwrite(0x0160, ana_test_mux)

        else:
            raise Exception("Unknown source")

        # adding pause to avoid bogus readings given the sampling freq increased in the newer FW version
        # val = self.regread(0x0e16)
        # origSettingADC['reg']['addr'] = np.append(origSettingADC['reg']['addr'], int('0x0e16', 0))
        # origSettingADC['reg']['val'] = np.append(origSettingADC['reg']['val'], val)

        # val = self.regread(0x0e14)
        # origSettingADC['reg']['addr'] = np.append(origSettingADC['reg']['addr'], int('0x0e14', 0))
        # origSettingADC['reg']['val'] = np.append(origSettingADC['reg']['val'], val)

        # s = self.regwrite(0x0e16, 0x4000)
        # s = self.regwrite(0x0e14, 0x0100)
        muxOptions = np.array(['ain0-ain1','ain0-ain3','ain1-ain3','ain2-ain3','ain0-gnd','ain1-gnd','ain2-gnd','ain3-gnd'])
        tmp, = np.where(muxOptions == ADC_config['mux'].lower())
        muxVal = tmp[0]

        # for i in range(N_AVERAGES-1, -1,-1):
        #   time.sleep(0.01)
        #   val = self.ReadBoardADC(muxVal)
        #   voltage = np.double(np.int16(val) / (2**15) * 4.096 * 1000000)

        # voltage = voltage / 1000000 # in V

        vRepeat = True
        nIter = 3

        while(vRepeat and (nIter >= 1)):
            # print(vRepeat,nIter)
            for i in range(N_AVERAGES-1, -1,-1):
                time.sleep(0.01)
                val = self.ReadBoardADC(muxVal)
                voltage = np.double(np.int16(val) / (2**15) * 4.096 * 1000000)

            voltage = voltage / 1000000 # in V

            if (self.prevVoltage != voltage) and (abs(voltage) > 0.03) and (abs(voltage) < 3.8):
                vRepeat = False
                self.prevVoltage = voltage
            else:
                time.sleep(0.1)
                nIter = nIter - 1

            # if nIter < 1:
            #     print('\nONBOARD ADC ERROR\n')
            #     try:
            #         logging.info('\nONBOARD ADC ERROR\n')
            #     except:
            #         pass

        # clean up
        success = True
        keys = list(origSettingADC.keys())

        if 'ampMuxOrig' in keys:
            success = self.regwriteburst(0x0E1C, np.array(origSettingADC['ampMuxOrig'],dtype=np.uint16), 'increment')

        origADCAddr = np.array(origSettingADC['reg']['addr'],dtype=np.uint16)
        origADCVal = np.array(origSettingADC['reg']['val'],dtype=np.uint16)
        s = self.regwrite(origADCAddr, origADCVal)

        xosc_ctrl = origSettingADC['xosc_ctrl']

        if ((xosc_ctrl >> 6) & 1) == 1: # if XIDLE_HOST originally was 1
            xosc_ctrl = xosc_ctrl | int('0x0010',0) # set XIDLE_HOST_set
            xosc_ctrl = xosc_ctrl & int('0xFFDF',0) # dont set XIDLE_HOST_clear
            s = self.regwrite(0x017C, xosc_ctrl)
            # self.error_assert(success, 'Unable to restore register 0x017C')

        self.error_assert(success, 'Unable to restore original register settings')
        print('Analog test mux source:', SOURCE , 'measured voltage:', np.round(voltage, 4))
        try:
            logging.info('Analog test mux source: ' + str(SOURCE) +' measured voltage: ' + str(np.round(voltage, 4)))
        except:
            pass

        return voltage
    

    def regwritemultiple(self, addr, val):
        """
        Write multiple registers 
        Inputs:
            addr: address array 
            val: value array 

        Outputs:
            status 

        """
        self.dev.writeSPISingle(addr, val)


    def regrmw(self, addr, val, mask=np.uint16(0x0000)):
        """
        Register read-modify-write 
        Inputs:
            addr: address in hex or decimal 
            val: value in hex or decimal 
            mask: mask in hex or decimal 

        Outputs:
            success: status of rmw 

        """
        addr = np.uint16(addr)
        val = np.uint16(val)
        mask = np.uint16(mask)
        prevVal = np.uint16(self.regread(addr))
        newVal = (prevVal & ~mask) | (val & mask)
        success = self.regwrite(addr, newVal)
        return success


    def dac_amptest0_get(self):
        """
        Get amptest0 value 
        Inputs:
            none

        Outputs:
            value

        """
        return self.dac_amptest0


    def dac_amptest0_set(self, val):
        """
        Set amptest0 value 
        Inputs:
            val: amptest0 value 

        Outputs:
            s: status

        """
        s = self.regwrite(0x0132, np.uint16(val))
        s_ = self.regwrite(0x0126, 0x0080) # Latch DAC_AMPTEST0
        # s = s and self.regrmw(0x0146, 0x0000, 0x0080) # Power up analog bandgap reference
        # s = s and self.regrmw(0x0128, 0x0000, 0x8184) # Power up DAC_AMPTEST0, DAC_AMPTEST1 and DAC_CMREF
        if s:
            self.dac_amptest0 = val
        return s


    def dac_amptest1_get(self):
        """
        Get amptest1 value
        Inputs:
            none

        Outputs:
            value

        """
        return self.dac_amptest1


    def dac_amptest1_set(self, val):
        """
        Set amptest1 value
        Inputs:
            val: amptest1 value

        Outputs:
            s: status

        """
        s = self.regwrite(0x0132, np.uint16(val))
        s_ = self.regwrite(0x0126, 0x0004) # Latch DAC_AMPTEST0
        # s = s and self.regrmw(0x0146, 0x0000, 0x0080) # Power up analog bandgap reference
        # s = s and self.regrmw(0x0128, 0x0000, 0x8184) # Power up DAC_AMPTEST0, DAC_AMPTEST1 and DAC_CMREF
        if s:
            self.dac_amptest1 = val
        return s


    def dac_cmref_get(self):
        """
        Get cmref value
        Inputs:
            none

        Outputs:
            value

        """
        return self.dac_cmref


    def dac_cmref_set(self, val):
        """
        Set cmref value
        Inputs:
            val: cmref value 

        Outputs:
            s: status

        """
        s = self.regwrite(0x0132, np.uint16(val))
        s_ = self.regwrite(0x0126, 0x0100) # Latch DAC_AMPTEST0
        # s = s and self.regrmw(0x0146, 0x0000, 0x0080) # Power up analog bandgap reference
        # s = s and self.regrmw(0x0128, 0x0000, 0x8184) # Power up DAC_AMPTEST0, DAC_AMPTEST1 and DAC_CMREF
        if s:
            self.dac_cmref = val
        return s
        

    def regreadburst(self, addr, length, mode = ''):
        """
        Register read burst (from RAM or multiple registers)
        Inputs:
            addr: Address 
            length: length to read 
            mode: RAM read or 'increament' to read multiple registers 

        Outputs:
            True 
            val: read value in array 

        """
        if mode == '':
            s, tmp =  self.dev.readSPIBurst(addr, length*2+4) # Temporory workaround
            val = np.zeros(length)
            val = 256*tmp[::2] + tmp[1::2]
            return np.all(s), val[0:-2]
        elif mode == 'increment':
            val = np.zeros(length,dtype=np.uint16)
            for i in range(length):
                val[i] = self.regread(addr + 2*i)
            return True, val

    def print_regreadburst(self, addr, length):
        """
        Print register read burst 
        Inputs: 
            addr: Address 
            length: length to read 

        Outputs:
            None 

        """
        for i in range(length):
            val = self.regread(addr + 2*i)
            print(hex(addr + 2*i),'-->',hex(val))

    def regwriteburst(self, addr, data, mode = ''):
        """
        Register write burst (to RAM or multiple registers)
        Inputs:
            addr: Address array 
            data: Data array 
            mode: RAM read or 'increament' to read multiple registers 

        Outputs:
            status 

        """
        addr = np.uint16(addr)
        data = np.uint16(data)
        if mode == '':
            RAMData = np.zeros( (len(data)*2), dtype = np.uint8)
            for i in range( len(data)):
                RAMData[2*i] = data[i] >> 8
                RAMData[2*i+1] = data[i] & 0xFF
            return self.dev.writeSPIBurst(addr, RAMData)

        elif mode == 'increment':
            if isinstance(addr, np.uint16):
                for i in range(len(data)):
                    success = self.regwrite(addr + i*2, data[i])
            else:
                for i in range(len(data)):
                    success = self.regwrite(addr[i], data[i])

        return success

        
    def GetIndirectRegister(self, name):
        """
        Read indirect registers
        Inputs:
            name: register name 

        Outputs:
            status
            value

        """
        if name == 'N_OVRD_ANAMODE':
            addr = 0x1fc1
            s = self.regwrite(0x0004, addr)
            return s, self.regread(0x0006)

        elif name == 'WAVE_FUNCTION_0':
            addr = 0x1fc5
            s = self.regwrite(0x0004, addr)
            return s, self.regread(0x0006)

        elif name == 'WAVE_FUNCTION_1':
            addr = 0x1fc9
            s = self.regwrite(0x0004, addr)
            return s, self.regread(0x0006)

        elif name == 'WAVE_FUNCTION_2':
            addr = 0x1fcd
            s = self.regwrite(0x0004, addr)
            return s, self.regread(0x0006)

        elif name == 'LASER_CONTROL':
            addr = 0x1fd1
            s = self.regwrite(0x0004, addr)
            return s, self.regread(0x0006)

        elif name == 'B_USE_LASER' or name == 'N_MSI_ID':
            addr = 0x1fb1
            s = self.regwrite(0x0004, addr)
            val = self.regread(0x0006)
            if name == 'B_USE_LASER':
                return val & 0xFF
            else:
                return val >> 8

        elif name == 'B_OVRD_ANAMODE' or name == 'B_NO_FSYNCVEC':
            addr = 0x1fb5
            s = self.regwrite(0x0004, addr)
            val = self.regread(0x0006)
            if name == 'B_OVRD_ANAMODE':
                return s, val & 0xFF
            else:
                return s, val >> 8

        elif name == 'B_LVDS_PD':
            addr = 0x1fb9
            s = self.regwrite(0x0004, addr)
            return self.regread(0x0006)

        elif name == 'B_EN_PHASESWEEP' or name == 'B_LFSR':
            addr = 0x1fbd
            s = self.regwrite(0x0004, addr)
            val = self.regread(0x0006)
            if name == 'B_EN_PHASESWEEP':
                return s, val & 0xFF
            else:
                return s, val >> 8

        else:
            raise Exception("Incorrect indirect register name: " + str(name))
            return  0, -1


    def SetIndirectRegister(self, name, val):
        """
        Set indirect registers
        Inputs:
            name: register name 
            val: register value 

        Outputs:
            status

        """
        if name == 'N_OVRD_ANAMODE':
            addr = 0x1fc1
            s = self.regwrite(0x0000, addr)
            s_ = self.regwrite(0x0002, np.uint16(val))

        elif name == 'WAVE_FUNCTION_0':
            addr = 0x1fc5
            s = self.regwrite(0x0000, addr)
            s_ = self.regwrite(0x0002, np.uint16(val))

        elif name == 'WAVE_FUNCTION_1':
            addr = 0x1fc9
            s = self.regwrite(0x0000, addr)
            s_ = self.regwrite(0x0002, np.uint16(val))

        elif name == 'WAVE_FUNCTION_2':
            addr = 0x1fcd
            s = self.regwrite(0x0000, addr)
            s_ = self.regwrite(0x0002, np.uint16(val))

        elif name == 'LASER_CONTROL':
            addr = 0x1fd1
            s = self.regwrite(0x0000, addr)
            s_ = self.regwrite(0x0002, np.uint16(val))

        elif name == 'B_USE_LASER' or name == 'N_MSI_ID':
            addr = 0x1fb1
            s = self.regwrite(0x0004, addr)
            prevVal = self.regread(0x0006)
            s = self.regwrite(0x0000, addr)
            if name == 'B_USE_LASER':
                s_  = self.regwrite(0x0002, np.uint16((prevVal & 0xFF00) | (val & 0x00FF)))
            else:
                s_ = self.regwrite(0x0002, np.uint16((prevVal & 0x00FF) | ((val << 8) & 0xFF00)))

        elif name == 'B_OVRD_ANAMODE' or name == 'B_NO_FSYNCVEC':
            addr = 0x1fb5
            s = self.regwrite(0x0004, addr)
            prevVal = self.regread(0x0006)
            s = self.regwrite(0x0000, addr)
            if name == 'B_OVRD_ANAMODE':
                s_ = self.regwrite(0x0002, np.uint16((prevVal & 0xFF00) | (val & 0x00FF)))
            else:
                s_ = self.regwrite(0x0002, np.uint16((prevVal & 0x00FF) | ((val << 8) & 0xFF00)))

        elif name == 'B_LVDS_PD':
            addr = 0x1fb9
            s = self.regwrite(0x0000, addr)
            s_ = self.regwrite(0x0002, np.uint16(val))

        elif name == 'B_EN_PHASESWEEP' or name == 'B_LFSR':
            addr = 0x1fbd
            s = self.regwrite(0x0004, addr)
            prevVal = self.regread(0x0006)
            s = self.regwrite(0x0000, addr)
            if name == 'B_EN_PHASESWEEP':
                s_ = self.regwrite(0x0002, np.uint16((prevVal & 0xFF00) | (val & 0x00FF)))
            else:
                s_ = self.regwrite(0x0002, np.uint16((prevVal & 0x00FF) | ((val << 8) & 0xFF00)))

        else:
            raise Exception("Incorrect indirect register name: " + str(name))

        s = self.regwrite(0x000C, 0x00C4)
        val = self.regread(0x0012)
        return s and self.regwrite(0x0012, np.uint16(val + 1)) # update sequence


    def LoadGainOffsetCalib(self, fname):
        """
        Loads gain and offset to memory
        Inputs:
            fname: location of csv file 

        Outputs:
            none

        """
        fin = open(fname, 'r')
        csv_reader = csv.reader( fin, delimiter=",")
        arr = np.array([],dtype=np.uint16)
        for row in csv_reader:
            arr = np.append(arr, np.uint16(int(str(row[0]),0)))
        
        ADC_updnoffset = arr[0]
        ADC_iramp = arr[1]

        Vref1DAC = arr[2]
        Vref1Set = arr[3]
        Vref2DAC = arr[4]
        Vref2Set = arr[5]
        Vref3DAC = arr[6]
        Vref3Set = arr[7]
        VCMDAC = arr[8]
        VCMSet = arr[9]
        inverseGlobalADCGain = np.zeros(4,dtype=np.uint16)
        inverseGlobalADCGain[0] = arr[10]
        inverseGlobalADCGain[1] = arr[11]
        inverseGlobalADCGain[2] = arr[12]
        inverseGlobalADCGain[3] = arr[13]
        perColGain = arr[14:12814]
        perColOffset = arr[12814:25614]

        dac_ctrl2_s1 = Vref1DAC | (Vref2DAC << 8)
        dac_ctrl3_s1 = Vref3DAC

        if 'VCMDAC' != 0:
            dac_ctrl3_s1 = dac_ctrl3_s1| (VCMDAC << 8)

        dac_ctrl2_s1_mask = 0x003F * (Vref1Set != 0) + 0x3F00 * (Vref2Set != 0)
        dac_ctrl3_s1_mask = 0x003F * (Vref3Set != 0)

        if VCMSet != 0 and VCMDAC != 0:
            dac_ctrl3_s1_mask = dac_ctrl3_s1_mask + 0x3F00 * (VCMSet != 0)
    
        val = self.regread(0x0112)
        if val != 22833:
            raise Exception('Cannot Communicate with chip / incorrect chipId')

        digPwrDown = self.regread(0x0014)
        s = self.regwrite(0x0014, 0x0000)

        s = self.regwrite(0x0102, ADC_updnoffset)
        s = s and self.regwrite(0x0104, ADC_iramp)
        s = s and self.regwriteburst(0x0520, inverseGlobalADCGain, 'increment')
        s = s and self.regrmw(0x012E, dac_ctrl2_s1, dac_ctrl2_s1_mask)
        s = s and self.regrmw(0x0130, dac_ctrl3_s1, dac_ctrl3_s1_mask)

        # perColGain
        s = s and self.regwrite(0x0500, 0x0008)
        s = s and self.regwrite(0x0502, 0x0000)
        s = s and self.regwriteburst(0x0504, perColGain)
        s = s and self.regrmw(0x0528, 0x0000, 0x0004)

        # perColOffset
        s = s and self.regwrite(0x0500, 0x0004)
        s = s and self.regwrite(0x0502, 0x0000)
        s = s and self.regwriteburst(0x0504, perColOffset)
        s = s and self.regrmw(0x0528, 0x0000, 0x0002)

        s = self.regwrite(0x0014, digPwrDown)
        if s:
            print('Gain and Offset calibration data loaded!')


    def SetLSDACValue(self, LSDAC_VALUE):
        """
        Sets LSDAC value. This function encrypts the LSDAC values
        before writing to memory
        Inputs:
            LSDAC_VALUE: 2D array of LSDAC value (unencrypted) 

        Outputs:
            none

        """

        #Stop execution before configuring LSDAC
        self.dev.writeSPISingle( 0x000C, 0x0002)
        regData = self.dev.readSPISingle( 0x000C, 1)
        i = 0
        while( regData[0] != 0x0 and i<20):
            time.sleep(0.1)
            regData = self.dev.readSPISingle( 0x000C, 1)
            i = i + 1
        if( i == 20):
            print("Error - camera not stopping")
            sys.exit()


        address, value = LSDAC.GenerateLSDACBlock(LSDAC_VALUE, self.n_modes_cfg)
        numel = len(address)
        newAddresses = np.array([])
        newValues = np.zeros(numel*2,dtype=object)
        for i in range(numel):
            newAddresses = np.append(newAddresses, np.array([0x0000,0x0002]))
        newValues[::2] = address
        newValues[1::2] = value
        print('Addr', newAddresses)
        print('Val', newValues)
        self.regwrite( np.asarray(newAddresses, dtype=np.uint16), np.asarray(newValues, dtype=np.uint16))

        #Start execution again after configuring LSDAC
        self.dev.writeSPISingle( 0x000C, 0x0001)

        print('LSDAC value set to', LSDAC_VALUE)
        try:
            logging.info('LSDAC value set to' + str(LSDAC_VALUE))
        except:
            pass



    def ReadBoardADC(self, val):
        """
        Read onboard ADC value
        Inputs:
            val: value 

        Outputs:
            ADC value 

        """
        return self.dev.getADCvalue(val)


    def _setInternalClassMode( self):
        """Configures internal variables depending in the selected mode.

        Inputs:
            none

        Outputs:
            none
        """
        mode_info_file_path = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'config/CameraModeInfo.yaml'))
        with open(mode_info_file_path) as y:
            modeInfo = yaml.load(y, Loader=yaml.SafeLoader)

        try:
            self.width = modeInfo['mode'][int(self.Mode)]['width']
            self.height = modeInfo['mode'][int(self.Mode)]['height']
            self.subFrames = modeInfo['mode'][int(self.Mode)]['nCaptures']
            self.nFreq = len(modeInfo['mode'][int(self.Mode)]['freqsMhz'])
            self.nPhases = modeInfo['mode'][int(self.Mode)]['nPhases']
            self.freqs = modeInfo['mode'][int(self.Mode)]['freqsMhz']
            self.freqPerPhase = modeInfo['mode'][int(self.Mode)]['freqPerPhase']
        except:
            raise Exception("Invalid mode selected (valid modes = 0-10)")
                
        self.rawImage = np.zeros( (self.height, self.width, self.subFrames), dtype = np.int16)
        self.pRAW = np.zeros( (self.height, self.width * self.subFrames), dtype = np.int16)
        self.unwrappedDepth = np.zeros( (self.height, self.width, self.nFreq), dtype = np.double)
        return 0

    def setFPS( self, fps):
        """Changes camera framerate
        
        :param fps: Camera framerate 
        :type fps: int 
        :returns: None  
        :rtype: None 
        """
        self.dev.setCAMFps(fps)


    def GetFramesADCCalib(self, nFrames=1):
        """
        Captures frames for ADC calibration
        Inputs:
            nFrames: number of frames to capture 

        Outputs:
            success: status of capture 
            frames: frames in a 3D array 

        """
        self.QuickStartSensor()

        time.sleep(0.5)

        frames = np.zeros([self.height, self.width, nFrames],dtype=np.int16)
        for frm in range(nFrames):
            tmp = self.dev.getFloatImage()
            time.sleep(debug_delay)
            RawImage = tmp[0]

            if self.dev.validFrame !=1:
                raise Exception('Error capturing frames! - invalid frame')
                return -1
            else:
                subImage = RawImage[0 : self.width*self.height]
                image16 = np.resize(subImage, (self.height, self.width))
                frames[:,:,frm] = image16.copy()
                success = True

        print('Frame captured - ADC Calib')
        logging.info('Frame captured - ADC Calib')
        # self.QuickStopSensor()
        #self.dev.stopSensor()
        self.QuickStopSensor()
        

        return success, frames


    def GetFramesPixVoltage(self, nFrames=1):
        """
        Captures frames for pixel voltage test
        Inputs:
            nFrames: number of frames to capture 

        Outputs:
            success: status of capture  
            frames: frames in a 3D array  

        """
        self.QuickStartSensor()

        time.sleep(0.5)

        frames = np.zeros([self.height, self.width, nFrames],dtype=np.int16)
        for frm in range(nFrames):
            tmp = self.dev.getFloatImage()
            time.sleep(debug_delay)
            RawImage = tmp[0]

            if self.dev.validFrame !=1:
                raise Exception('Error capturing frames! - invalid frame')
                return -1
            else:
                subImage = RawImage[0 : self.width*self.height]
                image16 =  self.dev.ConvertFloat2LinearVal(subImage)
                image16 = np.resize(image16, (self.height, self.width))
                frames[:,:,frm] = image16.copy()
                success = True

        print('Frame captured - ADC Calib')
        logging.info('Frame captured - ADC Calib')
        # self.QuickStopSensor()
        #self.dev.stopSensor()
        self.QuickStopSensor()
        
        return success, frames


    def GetFramesCompDACalib(self, nFrames=1):
        """
        Captures frames for comparator DAC calibratiom
        Inputs:
            nFrames: number of frames to capture 

        Outputs:
            GT: gain tag values 
            ADC: ADC values 
            Sat: Saturation status values 

        """
        # self.QuickStartSensor()
        self.Start()

        time.sleep(0.5)
        
        frames = np.zeros([self.height, self.width, 1, nFrames],dtype=np.int16)
        for frm in range(nFrames):
            #RawImage = self.dev.getFloatImage()
            tmp = tmp = self.dev.getFloatImage()
            time.sleep(debug_delay)
            RawImage = tmp[0]
            subImage = RawImage[0 : self.width*self.height]
            image16 = np.resize(subImage, (self.height, self.width))
            frames[:,:,0,frm] = image16.copy()

            if self.dev.validFrame !=1:
                raise Exception('Error capturing frames! - invalid frame')
                return -1
            else:
                success = True

        print('Frame captured - CompDAC Calib')
        logging.info('Frame captured - CompDAC Calib')

        self.Stop()
        
        imgRaw = frames[2:,:,:,:]
        GT = ((imgRaw.astype(np.uint16) & 3072) >> 10).astype(np.uint8)
        ADC = imgRaw.astype(np.uint16) & 1023
        SAT = ADC < 3

        if np.any(SAT == True):
            print('Saturation detected in comparator calibration frame!')
            logging.warning('Saturation detected in comparator calibration frame!')

        print(np.nanmean(ADC[:,:,0,0]),'ADC mean')
        logging.info(str(np.nanmean(ADC[:,:,0,0])) + ' ADC mean')
        print(np.nanmin(ADC[:,:,0,0]),'ADC min')
        logging.info(str(np.nanmin(ADC[:,:,0,0])) + ' ADC min')
        print(np.nanmax(ADC[:,:,0,0]),'ADC max')
        logging.info(str(np.nanmax(ADC[:,:,0,0])) + ' ADC max')


        return GT,ADC,SAT

    def GetFramesPCGO(self, nFrames=1):
        """
        Captures frames for per column gain and offset
        Inputs:
            nFrames: number of frames to capture 

        Outputs:
            GT: gain tag values 
            ADC: ADC values 
            Sat: Saturation status values 

        """
        self.QuickStartSensor()

        time.sleep(0.5)
        
        frames = np.zeros([self.height, self.width, 1, nFrames],dtype=np.int16)
        for frm in range(nFrames):
            #RawImage = self.dev.getFloatImage()
            tmp = self.dev.getFloatImage()
            time.sleep(debug_delay)
            RawImage = tmp[0]

            if self.dev.validFrame !=1:
                raise Exception('Error capturing frames! - invalid frame')
                return -1
            else:
                subImage = RawImage[0 : self.width*self.height]
                image16 = np.resize(subImage, (self.height, self.width))
                frames[:,:,0,frm] = image16.copy()
                success = True

        print('Frame captured - perColumn Calib')
        logging.info('Frame captured - perColumn Calib')
        # self.QuickStopSensor()
        #self.dev.stopSensor()
        self.QuickStopSensor()
        # time.sleep(0.1)
        
        imgRaw = frames[20:,:,:,:]
        GT = ((imgRaw.astype(np.uint16) & 3072) >> 10).astype(np.uint8)
        ADC = imgRaw.astype(np.uint16) & 1023
        SAT = ADC < 3

        if np.any(SAT == True):
            print('Saturation detected in Per Col GO calibration frame!')

        print(np.nanmean(ADC[:,:,0,0]),'ADC mean')
        logging.info(str(np.nanmean(ADC[:,:,0,0])) + ' ADC mean')
        print(np.nanmin(ADC[:,:,0,0]),'ADC min')
        logging.info(str(np.nanmin(ADC[:,:,0,0])) + ' ADC min')
        print(np.nanmax(ADC[:,:,0,0]),'ADC max')
        logging.info(str(np.nanmax(ADC[:,:,0,0])) + ' ADC max')


        return GT,ADC,SAT

    
    def procFrame( self):
        """Retrieves a frame from the camera and process the frame data filling the internal buffers and processing the
        raw data from the camera.
        
        :returns: None 
        :rtype: None 
        """

        RawImage = self.dev.getFloatImage()
        time.sleep(debug_delay)
        #RawImage = self.dev.getImage8U()
        if( self.dev.validFrame != 1):
        #if( self.dev.IO.validFrame != 1):
            return -1
        else:
            RawImage = RawImage[0]
            #RAW = self.dev.ConvertFloat2LinearVal( RawImage)
            RAW = RawImage
            RAW = np.resize( RAW, (self.height  * self.subFrames, self.width))
            self.pRAW = RAW.copy()

            for i in range( self.subFrames):
                subImage = RawImage[ i*self.width*self.height: (i+1)*self.width*self.height]
                # convert fractional raw image from camera to 16bit signed image
                image16 =  self.dev.ConvertFloat2LinearVal( subImage)
                image16 = np.resize( image16, (self.height, self.width))
                
                self.rawImage[:,:,i] = image16.copy()

            for i in range( len(self.freqs)):                
                f = self.freqs[i]
                depth = self.visualize_map( self.rawImage[:,:,i*self.nPhases:(i+1)*self.nPhases], f)[0]

                self.unwrappedDepth[:,:,i] = depth.copy()

        return 0



    def block_max(self, img):
        """
        Computes x and y coordinates of the brightest spot
        in the image
        Inputs:
            img: image array 

        Outputs:
            max_x: x coordinate of maximum brightness 
            max_y: y coordinate of maximum brightness 
        """
        res = ndimage.interpolation.zoom(img,0.125)
        max_blk = np.where(res == np.amax(res))
        max_x = np.clip(max_blk[1][0] * 8 + 4, 16, 1007)
        max_y = np.clip(max_blk[0][0] * 8 + 4, 16, 1007)
        return max_x, max_y



    def StreamAndCapturePCM(self, paramDict = dict()):
        """
        Streams and captures PassiveIR image
        Inputs:
            paramDict: dictionary containing frame parameters 

        Outputs:
            True 
            frames_dict: dictionary containing frames and header 
        """
         # Header Info

        keys = list(paramDict.keys())
        
        mode = 3

        if 'nFrames' in keys:
            nFrames = paramDict['nFrames']
        else:
            nFrames = 5

        if 'fileName' in keys:
            filename = paramDict['fileName']
        else:
            now = datetime.now()
            filename = 'mode_' + str(mode) + '_' + now.strftime("%Y%m%d_%H%M%S")

        if 'outputFormat' in keys:
            outputFormat = paramDict['outputFormat']
        else:
            outputFormat = 'pkl'


        self.StartSensor()
        Temp_sensor_offset = 1493.55 # From calibration
        Temp_sensor_slope = 5.45 # From calibration
        self.changeMode(mode)


        while(True):

            tmp = self.dev.getFloatImage()
            time.sleep(debug_delay)
            #RawImage = self.dev.getImage8U()
            if( self.dev.validFrame != 1):
            #if( self.dev.IO.validFrame != 1):
                print('Invalid frame')
                tmp = self.dev.getFloatImage()
                time.sleep(debug_delay)
                if( self.dev.validFrame != 1):
                    print('Retry: Invalid frame - Exiting')
                    sys.exit()

            else:
                RawImage = tmp[0]
                #RAW = self.dev.ConvertFloat2LinearVal( RawImage)
                RAW = RawImage
                RAW = np.resize( RAW, (self.height  * self.subFrames, self.width))
                subImage = RawImage[0:self.width*self.height]
                # convert fractional raw image from camera to 16bit signed image
                image16 =  self.dev.ConvertFloat2LinearVal( subImage)
                image16 = np.resize( image16, (self.height, self.width))
                orig_image = np.copy(image16)
                image16 = image16 + np.amin( image16)*-1
                image16 = np.array( image16, dtype=np.uint16)

                # process image for visualization        
                mImg = np.uint8( (image16/np.amax(image16)*(255)))
                max_x, max_y = self.block_max(mImg)
                mImg = cv2.applyColorMap( mImg, cv2.COLORMAP_BONE)
                Imgstr = "Passive IR Image"
                text1 = "Press 'd' to save and stop steaming"
                text2 = "Brightest 32x32 ROI mean = " + str(np.round(np.mean(orig_image[max_y-16:max_y+16,max_x-16:max_x+16]),3))

                font = cv2.FONT_HERSHEY_SIMPLEX
                fontScale = 1
                fontColor = (255,0,255) #BGR
                thickness = 2
                org1 = (50, 50)
                org2 = (50, 1000)

                mImg = cv2.putText(mImg, text1, org1, font, fontScale, fontColor, thickness, cv2.LINE_AA)
                mImg = cv2.putText(mImg, text2, org2, font, fontScale, fontColor, thickness, cv2.LINE_AA)

                mImg = cv2.rectangle(mImg, (max_x-16, max_y-16), (max_x+16, max_y+16), (0,0,255), 2)

                #cv2.putText( mImg, str(j), ( 200, 200), cv2.FONT_HERSHEY_SIMPLEX, .5, ( 0, 0, 0), 2, cv2.LINE_AA)
                cv2.imshow( Imgstr, mImg)

            c = cv2.waitKey(50) & 0xFF
            if (c == ord('d')) :
                cv2.destroyAllWindows()
                break
                


        captures = self.subFrames
        print(captures)
        # if self.freqPerPhase[-1] == 134: # Ignore passiveIR capture
        #     captures = captures - 1

        Img = np.zeros([self.height, self.width, captures, nFrames], dtype=np.int16)
        Header_tmp = np.zeros(nFrames, dtype=object)
        RawImage = np.zeros(nFrames,dtype=object)
        Header = np.zeros([captures, nFrames],dtype=object)

        frm = 0
        count = 0
        while frm < nFrames:
            print('Frame #', frm)
            tmp = self.dev.getFloatImage()
            time.sleep(debug_delay)
            if( self.dev.validFrame != 1):
                print('skipped frame')
                pass
            else:
                RawImage[frm] = tmp[0]
                Header_tmp[frm] = tmp[1]
                frm = frm + 1
            print('count:', count)
            count = count + 1

        self.QuickStopSensor()
                
        Header_tmp_per_capture_size = 128

        for frm in range(nFrames):
            print('Restructuring frm #:', frm)
            for i in range(captures):
                Header_tmp_per_capture = Header_tmp[frm][i*Header_tmp_per_capture_size : (i+1)*Header_tmp_per_capture_size]
                subImage = RawImage[frm][i*self.width*self.height: (i+1)*self.width*self.height]
                # convert fractional raw image from camera to 16bit signed image
                image16 =  self.dev.ConvertFloat2LinearVal(subImage)
                image16 = np.resize( image16, (self.height, self.width))
                Img[:,:,i,frm] = image16.copy()
                Header[i][frm] = dict()
                Header[i][frm]['mode'] = Header_tmp_per_capture[48] & 0x7
                Header[i][frm]['Frame_number'] = Header_tmp_per_capture[4]
                Header[i][frm]['height'] = Header_tmp_per_capture[7]
                Header[i][frm]['width'] = Header_tmp_per_capture[6]
                Header[i][frm]['nPhases'] = (Header_tmp_per_capture[50] & 0xF0) >> 4 #self.nPhases
                Header[i][frm]['phaseNo'] = Header_tmp_per_capture[50] & 0xF
                Header[i][frm]['nCaptures'] = (Header_tmp_per_capture[49] & 0xFC0) >> 6
                Header[i][frm]['captureNo'] = Header_tmp_per_capture[49] & 0x3F
                Header[i][frm]['nFreqs'] = (Header_tmp_per_capture[49] & 0xF000) >> 12
                Header[i][frm]['Temp_sensor_DegC'] = ((Header_tmp_per_capture[28] & 0xFFF) - Temp_sensor_offset) / Temp_sensor_slope
                Header[i][frm]['freqVal'] = Header_tmp_per_capture[53]
                Header[i][frm]['clkgenDenom'] = (Header_tmp_per_capture[52] & 0xFE00) >> 9
                Header[i][frm]['clkgenNumerator'] = Header_tmp_per_capture[52] & 0x1FF
                Header[i][frm]['LSDAC_port'] = Header_tmp_per_capture[57] >> 8
                Header[i][frm]['LSDAC_starboard'] = Header_tmp_per_capture[57] & 0x00FF

        if outputFormat == 'pkl':
            output = dict()
            output['Image'] = Img
            output['Header'] = Header
            f = open(filename, 'wb')
            pickle.dump(output, f)
            f.close()
        elif outputFormat == 'bin':
            for frm in range(nFrames):
                write_file = open(filename[:-4] + str(frm) + '.bin', "wb")
                tmp = np.array(Img[:,:,:,frm],dtype=np.int16)
                # tmp_flattened = np.zeros([],dtype=np.int16)
                for i in range(captures):
                    if i == 0:
                        tmp_flattened = tmp[:,:,i].flatten()
                    if i > 0 :
                        tmp_flattened = np.append(tmp_flattened, tmp[:,:,i].flatten())
                tmp_flattened.tofile(write_file, format='%d')
        else:
            raise Exception('Invalid output format for storing frames.')

        frames_dict = dict()
        frames_dict['Image'] = Img
        frames_dict['Header'] = Header

        return True, frames_dict


    def CaptureFramesAB(self, paramDict=dict()):
        """
        Captures AB frames 
        Inputs:
            paramDict: dictionary containing frame parameters

        Outputs:
            True
            output: dictionary containing frames and header
        """

        keys = list(paramDict.keys())

        if 'mode' in keys:
            mode = paramDict['mode']
        else:
            mode = 3

        if 'nFrames' in keys:
            nFrames = paramDict['nFrames']
        else:
            nFrames = 5

        if 'nWarmupFrames' in keys:
            nWarmupFrames = paramDict['nWarmupFrames']
        else:
            nWarmupFrames = 5

        if 'OVERRIDE_LSDACS' in keys:
            OVERRIDE_LSDACS = paramDict['OVERRIDE_LSDACS']
        else:
            OVERRIDE_LSDACS = False

        if 'LSDACValue' in keys:
            LSDACValue = paramDict['LSDACValue']
        else:
            LSDACValue = -1  # default value

        if 'feedbackCap' in keys:
            feedbackCap = paramDict['feedbackCap']
        else:
            feedbackCap = 0x8B42

        if 'samplingCap' in keys:
            samplingCap = paramDict['samplingCap']
        else:
            samplingCap = 0x0333

        if 'fileName' in keys:
            filename = paramDict['fileName']
        else:
            now = datetime.now()
            filename = 'mode_' + str(mode) + '_' + now.strftime("%Y%m%d_%H%M%S")

        if 'enableSweep' in keys:
            enableSweep = paramDict['enableSweep']
        else:
            enableSweep = False

        if 'outputFormat' in keys:
            outputFormat = paramDict['outputFormat']
        else:
            outputFormat = 'pkl'

        if 'also_save_image_as_bin' in keys:
            also_save_image_as_bin = paramDict['also_save_image_as_bin']
        else:
            also_save_image_as_bin = False

        if 'displayFrame' in keys:
            displayFrame = paramDict['displayFrame']
        else:
            displayFrame = False

        if 'displayDuration' in keys:
            displayDuration = paramDict['displayDuration']
        else:
            displayDuration = 3

        if 'folderName' in keys:
            calibFolderName = paramDict['folderName']
        else:
            print("No calib folder name to save file")
            sys.exit()

        if 'feedbackCap' in keys:
            success = self.regwrite(0x10C, feedbackCap)
        if 'samplingCap' in keys:
            success = self.regwrite(0x10E, samplingCap)

        if enableSweep:
            self.SetIndirectRegister('B_EN_PHASESWEEP', 1)

        self.QuickStartSensor()
        Temp_sensor_offset = 1493.55  # From calibration
        Temp_sensor_slope = 5.45  # From calibration

        captures = 10
        Img = np.zeros([self.height, self.width, captures], dtype=np.int16)

        while (True):

            time.sleep(0.5)
            tmp = self.dev.getFloatImage()
            time.sleep(debug_delay)
            # RawImage = self.dev.getImage8U()
            if (self.dev.validFrame != 1):
                # if( self.dev.IO.validFrame != 1):
                print('Invalid frame')
                pass

            else:

                RawImage = tmp[0]
                # RAW = self.dev.ConvertFloat2LinearVal( RawImage)
                #RAW = RawImage
                #RAW = np.resize(RAW, (self.height * self.subFrames, self.width))
                #subImage = RawImage[0:self.width * self.height]

                print('Restructuring frm:')
                for i in range(captures):
                    subImage = RawImage[i*self.width*self.height: (i+1)*self.width*self.height]

                    # convert fractional raw image from camera to 16bit signed image
                    image16 =  self.dev.ConvertFloat2LinearVal(subImage)
                    image16 = np.resize( image16, (self.height, self.width))
                    Img[:,:,i] = image16.copy()

                clean_IR = getABdata(Img)
                AB_img = clean_IR[:,:,0]

                image16 = AB_img + np.amin( AB_img)*-1
                image16 = np.array( image16, dtype=np.uint16)

                orig_image = np.copy(image16)
                # image16 = image16 + np.amin(image16) * -1
                # image16 = np.array(image16, dtype=np.uint16)
                # image16.astype(np.uint16)
                # image16.tofile('roi-u16.raw')

                # process image for visualization
                # mImg = np.uint8((image16 / np.amax(image16) * (255)))
                # mImg.astype(np.uint8)
                # mImg.tofile('roi-u8.raw')

                # process image for visualization        
                scaledImg = np.uint8( (image16/np.amax(image16)*(255)))
                max_x, max_y = self.block_max(scaledImg)
                scaledImg = cv2.applyColorMap( scaledImg, cv2.COLORMAP_BONE)
                mImg = np.copy(scaledImg)

                Imgstr = "AB Image"
                text1 = "Press 'd' to save and stop streaming"
                text2 = "Brightest 32x32 ROI mean = " + str(
                    np.round(np.mean(orig_image[max_y - 16:max_y + 16, max_x - 16:max_x + 16]), 3))

                font = cv2.FONT_HERSHEY_SIMPLEX
                fontScale = 1
                fontColor = (255, 0, 255)  # BGR
                thickness = 2
                org1 = (50, 50)
                org2 = (50, 450)

                mImg = cv2.putText(mImg, text1, org1, font, fontScale, fontColor, thickness, cv2.LINE_AA)
                #mImg = cv2.putText(mImg, text2, org2, font, fontScale, fontColor, thickness, cv2.LINE_AA)

                mImg = cv2.rectangle(mImg, (max_x - 16, max_y - 16), (max_x + 16, max_y + 16), (0, 0, 255), 2)

                # cv2.putText( mImg, str(j), ( 200, 200), cv2.FONT_HERSHEY_SIMPLEX, .5, ( 0, 0, 0), 2, cv2.LINE_AA)
                cv2.imshow(Imgstr, mImg)

            c = cv2.waitKey(50) & 0xFF
            if (c == ord('d')):
                plt.imsave(os.path.join(calibFolderName, 'AB image_Linear.png'), orig_image, cmap='gray')
                orig_image[orig_image <= 1] = 1 #Remove values less than 1 before taking log of values
                logdata = np.log(orig_image)
                plt.imsave(os.path.join(calibFolderName, 'AB image_Log.png'), logdata, cmap='gray')
                cv2.destroyAllWindows()
                break

        self.QuickStopSensor()
        return True, clean_IR

    def CaptureFramesP0(self, paramDict = dict()):
        """
        Captures frames for P0 calibration
        Inputs:
            paramDict: dictionary containing frame parameters 

        Outputs: 
            True 
            output: dictionary containing frames and header 
        """

        keys = list(paramDict.keys())
        
        if 'mode' in keys:
            mode = paramDict['mode']
        else:
            mode = 3

        if 'nFrames' in keys:
            nFrames = paramDict['nFrames']
        else:
            nFrames = 5

        if 'nWarmupFrames' in keys:   
            nWarmupFrames = paramDict['nWarmupFrames']
        else:
            nWarmupFrames = 5

        if 'OVERRIDE_LSDACS' in keys:
            OVERRIDE_LSDACS = paramDict['OVERRIDE_LSDACS']
        else:
            OVERRIDE_LSDACS = False

        if 'LSDACValue' in keys:
            LSDACValue = paramDict['LSDACValue']
        else:
            LSDACValue = -1 # default value

        if 'feedbackCap' in keys:
            feedbackCap = paramDict['feedbackCap']
        else:
            feedbackCap = 0x8B42

        if 'samplingCap' in keys:
            samplingCap = paramDict['samplingCap']
        else:
            samplingCap = 0x0333

        if 'fileName' in keys:
            filename = paramDict['fileName']
        else:
            now = datetime.now()
            filename = 'mode_' + str(mode) + '_' + now.strftime("%Y%m%d_%H%M%S")

        if 'enableSweep' in keys:
            enableSweep = paramDict['enableSweep']
        else:
            enableSweep = False

        if 'outputFormat' in keys:
            outputFormat = paramDict['outputFormat']
        else:
            outputFormat = 'pkl'

        if 'also_save_image_as_bin' in keys:
            also_save_image_as_bin = paramDict['also_save_image_as_bin']
        else:
            also_save_image_as_bin = False

        if 'displayFrame' in keys:
            displayFrame = paramDict['displayFrame']
        else:
            displayFrame = False

        if 'displayDuration' in keys:
            displayDuration = paramDict['displayDuration']
        else:
            displayDuration = 3


        if 'feedbackCap' in keys:
            success = self.regwrite(0x10C, feedbackCap)
        if 'samplingCap' in keys:
            success = self.regwrite(0x10E, samplingCap)

        # Disabling gain correction
        # success = self.regwrite(0x027C, 0x0000)

        if enableSweep:
            self.SetIndirectRegister('B_EN_PHASESWEEP', 1)

        Temp_sensor_offset = 1493.55 # From calibration
        Temp_sensor_slope = 5.45 # From calibration

        self.StartSensor()

        self.changeMode(mode)

        if OVERRIDE_LSDACS:
            success = self.SetLSDACValue(LSDACValue)


        # Warmup frames
        for i in range(nWarmupFrames):
            print('Warmup Frame #', i)
            tmp = self.dev.getFloatImage()
            time.sleep(debug_delay)
            if( self.dev.validFrame != 1):
                tries = 2
                while((self.dev.validFrame != 1) and (tries > 0)):
                    print('Frame retry #', tries)
                    time.sleep(0.5)
                    tmp = self.dev.getFloatImage()
                    time.sleep(0.5)
                    tries = tries - 1
                    if((self.dev.validFrame != 1) and (tries == 0)):
                        sys.exit()

        hdr_tmp = tmp[1]
        print(((hdr_tmp[28] & 0xFFF) - Temp_sensor_offset) / Temp_sensor_slope)

        captures = self.subFrames
        # if self.freqPerPhase[-1] == 134: # Ignore passiveIR capture
        #     captures = captures - 1

        Img = np.zeros([self.height, self.width, captures, nFrames], dtype=np.int16)
        Img_tmp = np.zeros([self.height, self.width, captures], dtype=np.int16)
        Header_tmp = np.zeros(nFrames, dtype=object)
        RawImage = np.zeros(nFrames,dtype=object)
        Header = np.zeros([captures, nFrames],dtype=object)


        frm = 0
        count = 0
        while frm < nFrames:
            print('Frame #', frm)
            tmp = self.dev.getFloatImage()
            time.sleep(debug_delay)
            if( self.dev.validFrame != 1):
                print('skipped frame')
                pass
            else:
                RawImage[frm] = tmp[0]
                Header_tmp[frm] = tmp[1]
                frm = frm + 1
            print('count:', count)
            count = count + 1

        self.QuickStopSensor()
                
        Header_tmp_per_capture_size = 128

        for frm in range(nFrames):
            print('Restructuring frm #:', frm)
            for i in range(captures):
                Header_tmp_per_capture = Header_tmp[frm][i*Header_tmp_per_capture_size : (i+1)*Header_tmp_per_capture_size]
                subImage = RawImage[frm][i*self.width*self.height: (i+1)*self.width*self.height]
                # convert fractional raw image from camera to 16bit signed image
                image16 =  self.dev.ConvertFloat2LinearVal(subImage)
                image16 = np.resize(image16, (self.height, self.width))
                Img_tmp[:,:,i] = image16.copy()
                Header[i][frm] = dict()
                Header[i][frm]['mode'] = Header_tmp_per_capture[48] & 0x7
                Header[i][frm]['Frame_number'] = Header_tmp_per_capture[4]
                Header[i][frm]['height'] = Header_tmp_per_capture[7]
                Header[i][frm]['width'] = Header_tmp_per_capture[6]
                Header[i][frm]['nPhases'] = (Header_tmp_per_capture[50] & 0xF0) >> 4 #self.nPhases
                Header[i][frm]['phaseNo'] = Header_tmp_per_capture[50] & 0xF
                Header[i][frm]['nCaptures'] = (Header_tmp_per_capture[49] & 0xFC0) >> 6
                Header[i][frm]['captureNo'] = Header_tmp_per_capture[49] & 0x3F
                Header[i][frm]['nFreqs'] = (Header_tmp_per_capture[49] & 0xF000) >> 12
                Header[i][frm]['Temp_sensor_DegC'] = ((Header_tmp_per_capture[28] & 0xFFF) - Temp_sensor_offset) / Temp_sensor_slope
                Header[i][frm]['freqVal'] = Header_tmp_per_capture[53]
                Header[i][frm]['clkgenDenom'] = (Header_tmp_per_capture[52] & 0xFE00) >> 9
                Header[i][frm]['clkgenNumerator'] = Header_tmp_per_capture[52] & 0x1FF
                Header[i][frm]['LSDAC_port'] = Header_tmp_per_capture[57] >> 8
                Header[i][frm]['LSDAC_starboard'] = Header_tmp_per_capture[57] & 0x00FF
            Img[:,:,:,frm] = Img_tmp

        output = dict()
        output['Image'] = Img
        output['Header'] = Header

        if outputFormat == 'pkl':
            f = open(filename, 'wb')
            pickle.dump(output, f)
            f.close()
        elif outputFormat == 'bin':
            if not os.path.exists(filename[:-4]+'_bin'):
                os.mkdir(filename[:-4]+'_bin')

            frm_name = os.path.join(filename[:-4]+'_bin', os.path.basename(filename[:-4]))

            for frm in range(nFrames):
                write_file = open(frm_name + str(frm) + '.bin', "wb")
                tmp = np.array(Img[:,:,:,frm],dtype=np.int16)
                # tmp_flattened = np.zeros([],dtype=np.int16)
                for i in range(captures):
                    if i == 0:
                        tmp_flattened = tmp[:,:,i].flatten()
                    if i > 0 :
                        tmp_flattened = np.append(tmp_flattened, tmp[:,:,i].flatten())
                tmp_flattened.tofile(write_file, format='%d')

        elif outputFormat == 'none':
            pass # do not save frames
            
        else:
            raise Exception('Invalid output format for storing frames.')

        if also_save_image_as_bin and outputFormat != 'bin':
            if not os.path.exists(filename[:-4]+'_bin'):
                os.mkdir(filename[:-4]+'_bin')

            frm_name = os.path.join(filename[:-4]+'_bin', os.path.basename(filename[:-4]))

            for frm in range(nFrames):
                write_file = open(frm_name + str(frm) + '.bin', "wb")
                tmp = np.array(Img[:,:,:,frm],dtype=np.int16)
                # tmp_flattened = np.zeros([],dtype=np.int16)
                for i in range(captures):
                    if i == 0:
                        tmp_flattened = tmp[:,:,i].flatten()
                    if i > 0 :
                        tmp_flattened = np.append(tmp_flattened, tmp[:,:,i].flatten())
                tmp_flattened.tofile(write_file, format='%d')

        if displayFrame:
            plt.figure()
            plt.imshow(np.mean(Img, axis=3)[:,:,0])
            plt.show(block=False)
            plt.pause(displayDuration)
            plt.close()

        return True, output


    def changeMode( self, Mode):
        """Changes image sensor mode. This function will not change the camera resolution.
        
        :param Mode: image sensor mode. 
        :type Mode: int 
        :returns: None 
        :rtype: None 
        """
        self.Mode = Mode
        self._setInternalClassMode()

        self.dev.writeSPISingle( 0x000C, 0x0002)
        regData = self.dev.readSPISingle( 0x000C, 1)
        i = 0
        while( regData[0] != 0x0 and i<20):
            time.sleep(0.1)
            regData = self.dev.readSPISingle( 0x000C, 1)
            i = i + 1
        if( i == 20):
            print("Error - camera not stopping")
            sys.exit()

        self.dev.writeSPISingle( 0x0200, self.Mode)
        self.dev.writeSPISingle( 0x000C, 0x0001)


    def visualize_map(self, pixout, f_mhz):
        """
        Visualizes map
        Inputs:
            pixout: raw pixel values 
            f_mhz: frequency in mhz 

        Outputs:
            dist: distance map in meter 
            ampl: amplitude map 
        """          
        nPhase = self.nPhases #len(pixout[0,0,:])
        V1_real = np.sum(np.array([np.cos(2*np.pi*n/nPhase)
                        for n in range(0, nPhase)]) * pixout, axis=2)
        V1_imag = np.sum(-1 * np.array([np.sin(2*np.pi*n/nPhase)
                        for n in range(0, nPhase)]) * pixout, axis=2)
        dist = (299792458/(4*np.pi*f_mhz*1000000)*np.arctan2(V1_imag, V1_real))
        dist = dist % (299792458 * (1/f_mhz/1000000) / 2) # Rolling phases
        ampl = np.abs(V1_real, V1_imag)
        
        
        return dist, ampl # in meters

    def programCSV( self, filename):
        """
        Programs the sensor with csv
        Inputs:
            filename: csv filename 

        Outputs:
            none 
        """ 
        fin = open(filename)
        csv_reader = csv.reader( fin, delimiter=",")

        if( self._debug):
            fname = "firmware0.bin"
            fwrite = open( fname, "w+b")
            fdebug = open( "debug.txt", "w")

        regAddr = 0x0000
        regData = 0x0000
        RAMAddr = 0x0000
        Data = []
        rData = []
        rAddr = []
        i = 0
        j = 0
        k = 0
        z = 0
        TxLen = 0
        prevTxLen = 0
        bProgram = 0
        bProgramRAM = 0

        for row in csv_reader:
            bProgram = 0
            bProgramRAM = 0

            try:
                TxLen = int( row[4])
            except:
                print( "error parsing len" + str(row))
                sys.exit()
            try:
                regAddr = int( row[0], 16)
            except:
                #print( "error parsing reg addr" + str(row))
                regAddr = -1

            try:
                regData = int( row[1], 16)
            except:
                print( "error parsing reg data" + str(row))
                regData = -1

            if( TxLen > 2):
                if( regAddr != -1):
                    Data.append( regAddr)                    
                if( regData != -1):
                    Data.append( regData)  
                if( len(rAddr) > 0):
                    bProgram = 1
            else:
                if( self._debug):
                    fwrite.write( regAddr.to_bytes( 2, byteorder="big", signed=False))
                    fwrite.write( regData.to_bytes( 2, byteorder="big", signed=False))
                
                i = i + 1
                rAddr.append( regAddr)
                rData.append( regData)

                if( len( Data) > 0):
                    bProgramRAM = 1
    
            if( bProgram):
                print( "programming " + str( len(rAddr)) + " single registers")
                i = 0            
                j = j + 1
                
                if( self.dev != None):
                    self.dev.writeSPISingle( np.asarray( rAddr, dtype=np.uint16), np.asarray( rData, dtype=np.uint16))
                    #time.sleep(2)
                    pass

                if( self._debug):
                    fwrite.close()
                    fname = "firmware"+str(j)+".bin"
                    fwrite = open( fname, "w+b")
                    for i in range( len( rAddr)):
                        fdebug.write( hex(rAddr[i]) + ", " + hex( rData[i]) + "\n")

                rData = []
                rAddr = []

            if( bProgramRAM):
                print( "programming RAM " + str( len(Data)*2-2) + " registers")
                i = 0
                RAMAddr = []
                RAMAddr = int(Data[0])
                #Data.remove(0)
                #Data = Data[1:-1]
                del Data[0]
                if( self._debug):                
                    fwrite.write( RAMAddr.to_bytes( 2, byteorder="big", signed=False))
                    for dat in Data:
                        fwrite.write( dat.to_bytes( 2, byteorder="big", signed=False))

                    fwrite.close()
                    j = j + 1
                    fname = "firmware"+str(j)+".bin"
                    fwrite = open( fname, "w+b")

                if( self.dev != None):
                    RAMData = np.zeros( (len( Data)*2), dtype = np.uint8)                
                    for i in range( len(Data)):
                        RAMData[2*i] = Data[i] >> 8
                        RAMData[2*i+1] = Data[i] & 0xFF
                    self.dev.writeSPIBurst( RAMAddr, RAMData)

                if( self._debug):
                    k = 0
                    for dat in Data:
                        if k == 0:
                            fdebug.write( hex(RAMAddr) + ", " + hex(dat) + "\n")
                        else:
                            if( k%2 == 0):
                                fdebug.write( ", " + hex(dat) + "\n")
                            else:
                                fdebug.write( hex(dat))
                        k = k + 1

                    if( k%2 ==0):
                        fdebug.write("\n")
                Data = []
            k = k + 1

        if( len( rAddr) > 0):
            print( "programming " + str( len( rAddr)) + " final registers")
            i = 0
            j = j + 1

            if( self._debug):
                fwrite.close()
                fname = "firmware_final.bin"
                fwrite = open( fname, "w+b")
                for i in range( len( rAddr)):
                    fwrite.write( rAddr[i].to_bytes( 2, byteorder="big", signed=False))
                    fwrite.write( rData[i].to_bytes( 2, byteorder="big", signed=False))

            if( self.dev != None):
                self.dev.writeSPISingle( np.asarray( rAddr, dtype=np.uint16), np.asarray( rData, dtype=np.uint16))
                pass

            if( self._debug):
                for i in range( len( rAddr)):
                    fdebug.write( hex(rAddr[i]) + ", " + hex( rData[i]) + "\n")
        if( self._debug):
            fdebug.close()

    def PowerMonitor( self,PMId):
        """This function reads the voltage, current and power measured by a given
        power monitor in the Malka board.

        :param dev: Ardacho camera pointer. 
        :type dev: Ardacho IOLib device class. 
        :param PMId: Power Monitor id. Use any of the ids defined at the top of the application script. 
        :type PMId: int      
        :returns: returns a list with the measured parameters [ Voltage, Current, Power] 
        :rtype: [ float, float, float] 
        """

        CONF_REG = 0x00
        VBUS_REG = 0x02
        POWER_REG = 0x03
        SHUNT_REG = 0x01
        CURR_REG = 0x04
        CAL_REG = 0x05
        
        read_delay = 0.02
        full_scale = 32767 #7FFF

        time.sleep(read_delay)
        self.dev.writePowerMonReg( PMId, CONF_REG, 0x4007 | 1 << 9 | 4 << 6 | 4 << 3 )
        time.sleep(read_delay)
        Vbus =  self.dev.readPowerMonReg( PMId, VBUS_REG)
        time.sleep(read_delay)
        Cal = self.dev.readPowerMonReg( PMId, CAL_REG)
        time.sleep(read_delay)
        CUR = self.dev.readPowerMonReg( PMId, CURR_REG)
        if CUR > full_scale:
            CUR = 0
        time.sleep(read_delay)
        SHUNT = self.dev.readPowerMonReg( PMId, SHUNT_REG)
        if SHUNT > full_scale:
            SHUNT = 0
        time.sleep(read_delay)
        PWR = self.dev.readPowerMonReg( PMId, POWER_REG)
        if PWR > full_scale:
            PWR = 0
        time.sleep(read_delay)


        Volt = round(Vbus * 1.25/1000, 3)
        if( PMId == VMAIN_DEV_ID):
            Imax = (0.00512/0.01)*((32768)/Cal) # Vmain uses 0.01 shunt resistor
        else:
            Imax = (0.00512/0.02)*((32768)/Cal) # Vaux, Vdepth and Vsys uses 0.02 shunt resistor

        ILSB = (Imax/(32768))
        Curr =  round(CUR* ILSB,5)
        PwrLSB = 25*ILSB
        Power = round(PWR * PwrLSB, 5)
        return [ Volt, Curr, Power ]


    def sensorPowerDown( self):
        """
        De-initialize camera
        Inputs:
            none 

        Outputs:
            0
        """ 
        # De-Initialize the camera
        self.dev.setGpio( CAM_RESET_N, 0)
        self.dev.setGpio( DEPTH_VAUX_RUN, 0)
        self.dev.setGpio( PS_VAUXVPS1, 0)
        self.dev.setGpio( PS_VAUXVPS2, 0)
        self.dev.setGpio( PS_V5VVPS3, 0)
        self.dev.setGpio( LMZ_EN, 0)
        self.dev.setGpio( CAM_RESET_N, 0)

        # Wait for CAM_SHUTDOWN_N to go low indicating that everything is ok
        retries = 10
        while( retries > 0):
            if (self.module_class == 'Walden_R1' or 'Crosby'):
                cam_shutdown = 0    # MZ: Walden does not have shutdown output to controller (connector pin left floating in rev.1)
            else:
                cam_shutdown = self.dev.getGpio( CAM_SHUTDOWN_N)
            time.sleep(0.1)
            if( cam_shutdown == 0):
                break
            else:
                retries = retries - 1

        if( retries == 0):
            print( "Error during camera de-initialization")
            self.dev.setGpio( DEPTH_VAUX_RUN, 0)
            self.dev.setGpio( PS_VAUXVPS2, 0)
            self.dev.setGpio( PS_VAUXVPS1, 0)
            self.dev.setGpio( PS_V5VVPS3, 0)
            self.dev.setGpio( LMZ_EN, 0)
            return -1

        return 0

          
    def sensorPowerUp( self):
        """
        Implements sensor initialization sequence enabling the power regulators and making functional checks.
        Inputs:
            none 

        Outputs:
            0 
        """ 
        #self.dev.writeI2C(I2C_MALKA, AD5593R_ID,[AD5593R_DAC_SET|VAUX_ADJ,0x9E, 0xF8]) # MZ1: Added VAUX DAC set for 12V 
        #self.dev.writeI2C(I2C_MALKA, AD5593R_ID,[AD5593R_DAC_SET|VAUX_ADJ,0x9C, 0x17]) # MZ1: Added VAUX DAC set for 18V  
        self.dev.writeI2C(I2C_MALKA, AD5593R_ID,[AD5593R_DAC_SET|VAUX_ADJ,0x9A, 0x2C]) # MZ1: Added VAUX DAC set for 22V       
        #self.dev.writeI2C(I2C_MALKA, AD5593R_ID,[AD5593R_DAC_SET|VAUX_ADJ,0x97, 0xDE]) # MZ1: Added VAUX DAC set for 27V 

        self.dev.setGpio( CAM_RESET_N, 0) # reset camera
        self.dev.setGpio( LMZ_EN, 1) # enable Vmain
        time.sleep(0.01) # wait 10 ms
        [ Vmain, Imain, Pmain] = self.PowerMonitor( VMAIN_DEV_ID) # Check Vmain
        # MZ: Added current and voltage measurements
        print('Vmain: ' + str(Vmain) + ', Imain: ' + str(Imain) + ', Pmain: ' + str(Pmain))

        self.dev.setGpio( PS_V5VVPS3, 1) # enable TPS61230A producint 5V2 to drive Vdepth
        time.sleep(0.01) # wait 10 ms
        [ Vdepth, Idepth, Pdepth] = self.PowerMonitor( VDEPTH_DEV_ID) # Check Vmain
        # MZ: Added current and voltage measurements
        print('Vdepth: ' + str(Vdepth) + ', Idepth: ' + str(Idepth) + ', Pdepth: ' + str(Pdepth))
        
        self.dev.setGpio( PP_DEPTH, 1) # enable XC6222 and Vdepth
        time.sleep( 0.004) # wait 4 ms

        self.dev.setGpio( PS_VAUXVPS1, 1) # enable ADP196
        self.dev.setGpio( PS_VAUXVPS2, 1) # enable Vsys
        time.sleep(0.01) # wait 10 ms
        [ Vsys, Isys, Psys] = self.PowerMonitor( VSYS_DEV_ID)
        # MZ: Added current and voltage measurements
        print("Vsys: " + str(Vsys) + ", Isys: " + str(Isys)+", Psys: " + str(Psys))
        
        time.sleep( 0.004) # wait 4 ms

        self.dev.setGpio( DEPTH_VAUX_RUN, 1)
        time.sleep(0.01) # wait 10 ms
        [ Vaux, Iaux, Paux] = self.PowerMonitor( VAUX_DEV_ID)
        # MZ: Added current and voltage measurements
        print('Vaux: ' + str(Vaux) +', Iaux: ' + str(Iaux) +', Paux: ' + str(Paux))

        # Wait for CAM_SHUTDOWN_N to go high indicating that everything is ok
        retries = 10
        while( retries > 0):
            if (self.module_class == 'Walden_R1' or 'Crosby'):
                cam_shutdown = 1 # MZ: Walden does not have shutdown output to controller (connector pin left floating in rev.1)
            else:
                cam_shutdown = self.dev.getGpio( CAM_SHUTDOWN_N)
            if( cam_shutdown == 1):
                break
            else:
                retries = retries - 1


        # wait 2 ms
        time.sleep( 0.002)
        self.dev.setGpio( CAM_RESET_N, 1)        

        return 0

    def bitget(self, var, pos): # !Python indexing!
        """
        Returns bit in a position
        Inputs:
            var: hex or decimal 
            pos: position 

        Outputs:
            bit in the position 
        """ 
        return(var >> pos) & 1

    def repmatdec2Hex(self, dec, rep):
        """
        Repeats decimal as a hex
        Inputs:
            dec: decimal input 
            rep: repeat count 

        Outputs:
            hex output 
        """ 
        hexval = hex(dec).split('x')[-1]
        out = ''
        for _ in range(rep):
            out = out + hexval
        return int('0x' + out.upper(),0)

    def get_addr_hex2dec(self, addr, k):
        """
        Repeats decimal as a hex with padding
        Inputs:
            addr: address 
            k: repeat count 

        Outputs:
            hex output with padding 
        """ 
        dec = int(addr,0) + k*2
        hex_raw = hex(dec).split('x')[-1]
        pad_zero = 4 - len(hex_raw)
        return ('0x' + '0'*pad_zero + hex_raw)

    def generate_truthtable(self, n):
        """
        Generates a 'truth table' of n variables
        Inputs:
            n: variable length 

        Outputs:
            truth table 
        """
        if n < 1:
            return [[]]
        subtable = self.generate_truthtable(n - 1)
        return [ row + [v] for row in subtable for v in [0,1] ]

    def ReadAmpGainSetting(self):
        """
        Reads amplifier gain setting
        Inputs:
            none 

        Outputs:
            amp gain setting 
        """ 
        val = self.regread(0x014E) # Analog NCDS mode bit
        # self.error_assert(success, 'Cannot read Analog NCDS mode bit')
        ncds_mode = self.bitget(int(val), 3)

        if ncds_mode:
            Cin = np.array([160,80,80]) # For ADSD3100, Cin is different between analog and digital CDS modes
        else:
            Cin = np.array([320,160,80])

        Cfb = np.array([4*25.88,2*20.24,2*6.76,2*5.17,2*3.57]) # gain= (160*cin1+80*cin0+80)/(2*3.57+2*5.17*cfb0+2*6.76*cfb1+2*20.24*cfb2+4*25.88*cfb3)

        tmp1 = np.sum(np.array(self.generate_truthtable(2)) * Cin[0:2], axis=1) + Cin[2]
        tmp2 = np.sum(np.array(self.generate_truthtable(4)) * Cfb[0:4], axis=1) + Cfb[4]
        ampgainTable = np.transpose(np.reshape(tmp1,[1,len(tmp1)])) / tmp2

        # plt.plot(ampgainTable[0,:], label="Cin=0")
        # plt.plot(ampgainTable[1,:], label="Cin=1")
        # plt.plot(ampgainTable[2,:], label="Cin=2")
        # plt.plot(ampgainTable[3,:], label="Cin=3")
        # plt.legend()
        # plt.ylabel('Gain')
        # plt.xlabel('Cfb')
        # plt.title('ampgainTable')
        # plt.show()

        # Manually update table with gains measured on ADSD3100
        ampgainTable[3,1] = 16.75
        ampgainTable[3,2] = 14.7
        ampgainTable[3,4] = 6.8
        ampgainTable[3,11] = 2.35
        ampgainTable[0,8] = 0.72


        RegisterValueCin = self.regread(0x010E) # AMP Cin register
        # self.error_assert(success, 'Cannot read AMP Cin register')
        RegisterValueCfb = self.regread(0x010C) # AMP Cfb register
        # self.error_assert(success, 'Cannot read AMP Cfb register')

        ampgain = np.zeros(4)

        for i in range(4):
            Cfb = (int(RegisterValueCfb) >> i*4) & 15
            Cin = (int(RegisterValueCin) >> i*4) & 3
            ampgain[i] = ampgainTable[Cin, Cfb]

        return ampgain

    def SetDiffDacSfVoltage(self, DacCal, DacSel, TargetDiff, CMwindow, DacCal_folderName=""):
        """
        Sets differential DAC voltage
        Inputs:
            DacCal: DAC calibration values 
            DacSel: DAC selection ex: AMPTEST0 etc 
            TargetDiff: Voltage difference 
            CMWindow: CommonMode window 
            DacCal_folderName: location of file to store DacCal values 

        Outputs:
            DacCal : DAC calibration values 
            AMPTEST0, AMPTEST1, CMREF values 
            diff_meas_out: measured differential voltage 

        """ 
        debug_delay = 0.2
        time.sleep(debug_delay)
        #s, valArr = self.regreadburst(0x0E00, 80,'increment')
        self.regwrite(0x0014,0x0000)
        time.sleep(debug_delay)
        self.regwrite(0x0014,0x0000)
        time.sleep(debug_delay)
        #print(valArr)
        NUM_ITERATIONS = 1
        MAX_ERROR = [5e-3, 15e-3]

        if len(list(DacCal.keys())) == 0:
            CMREF_Samples = [150, 203, 255]
            AMPTEST_Samples = [150, 176, 203, 229, 255]
            CMREF_meas = np.zeros(len(CMREF_Samples))
            AMPTEST0_meas = np.zeros([len(CMREF_Samples), len(AMPTEST_Samples)])
            AMPTEST1_meas = np.zeros([len(CMREF_Samples), len(AMPTEST_Samples)])
            for j in range(len(CMREF_Samples)-1, -1,-1):
                # success = self.API.APIComm(1, self.API.API['main']['ioctrl'], self.API.API['IO']['dac_cmref']['set'], int(CMREF_Samples[j]))
                success = self.dac_cmref_set(np.uint8(CMREF_Samples[j]))
                self.error_assert(success, 'Unable to set DAC CMREF')
                time.sleep(debug_delay)
                CMREF_meas[j] = self.CamMeasureAnaTestVoltage('DACwithSF_CMREF')
                for k in range(len(AMPTEST_Samples)-1,-1,-1):
                    # success = self.API.APIComm(1, self.API.API['main']['ioctrl'], self.API.API['IO']['dac_amptest0']['set'], int(AMPTEST_Samples[k]))
                    success = self.dac_amptest0_set(np.uint8(AMPTEST_Samples[k]))
                    self.error_assert(success, 'Unable to set DAC AMPTEST0')
                    # success = self.API.APIComm(1, self.API.API['main']['ioctrl'], self.API.API['IO']['dac_amptest1']['set'], int(AMPTEST_Samples[k]))
                    success = self.dac_amptest1_set(np.uint8(AMPTEST_Samples[k]))
                    self.error_assert(success, 'Unable to set DAC AMPTEST1')
                    AMPTEST0_meas[j,k] = self.CamMeasureAnaTestVoltage('DACwithSF_AMPTEST0')
                    AMPTEST1_meas[j,k] = self.CamMeasureAnaTestVoltage('DACwithSF_AMPTEST1')
            y,x = np.meshgrid(AMPTEST_Samples,CMREF_Samples)
            Xmeas = [[np.ones([len(CMREF_Samples)*len(AMPTEST_Samples)])], [np.transpose(x).flatten()],
                    [np.transpose(y).flatten()], [np.transpose(y).flatten()**2]]
            Xmeas = np.squeeze(np.transpose(Xmeas))
            M = np.linalg.lstsq(Xmeas, np.transpose(AMPTEST0_meas).flatten(), rcond=None)[0] # Matrix left division

            [y,x] = np.meshgrid(range(0,256), range(0,256))
            Xfit = [[np.ones([np.size(y)])], [np.transpose(x).flatten()],
                    [np.transpose(y).flatten()], [np.transpose(y).flatten()**2]]
            Xfit = np.squeeze(np.transpose(Xfit))
            DacCal['AMPTEST0_pred'] = np.reshape(np.dot(Xfit,M), np.shape(y))

            M = np.linalg.lstsq(Xmeas, np.transpose(AMPTEST1_meas).flatten(), rcond=None)[0] # Matrix left division
            DacCal['AMPTEST1_pred'] = np.reshape(np.dot(Xfit,M), np.shape(y))

            # Dacs are non-linear at low code values, so remove these points from the predicted data
            DacCal['AMPTEST0_pred'][:,0:np.min(CMREF_Samples)] = np.nan
            DacCal['AMPTEST0_pred'][0:np.min(AMPTEST_Samples),:] = np.nan
            DacCal['AMPTEST1_pred'][:,0:np.min(CMREF_Samples)] = np.nan
            DacCal['AMPTEST1_pred'][0:np.min(AMPTEST_Samples),:] = np.nan

            DacCal['CMREF_pred'] = np.polyval(np.polyfit(CMREF_Samples, CMREF_meas, 1),range(0,256))
            DacCal['CMREF_pred'][0:np.min(CMREF_Samples)] = np.nan # Remove CMREF values below min sampled DAC code

            DacCal['RangeCMREF'] = np.array([CMREF_meas[-1], CMREF_meas[0]])
            DacCal['RangeAMPTEST0'] = np.array([np.max(AMPTEST0_meas[:,0]), np.min(AMPTEST0_meas[:,-1])])
            DacCal['RangeAMPTEST1'] = np.array([np.max(AMPTEST1_meas[:,0]), np.min(AMPTEST1_meas[:,-1])])
            if(DacCal_folderName != ""):
                f = open(os.path.join(DacCal_folderName, 'DacCal.pkl'), 'wb')
                pickle.dump(DacCal, f)
                f.close()


        if DacSel.upper() == 'CMREF-AMPTEST0':
            # Sanity check inputs
            if (np.max(CMwindow) + TargetDiff/2) < DacCal['RangeCMREF'][0]:
                raise Exception('CMWindow [%.3f,%.3f] V and TargetDiff %.3f V exceed range of CMRef DAC (%.3fV)' %(CMwindow,TargetDiff,DacCal['RangeCMREF'][0]))
                logging.error(str('CMWindow [%.3f,%.3f] V and TargetDiff %.3f V exceed range of CMRef DAC (%.3fV)' %(CMwindow,TargetDiff,DacCal['RangeCMREF'][0])))
        
            if (np.min(CMwindow) + TargetDiff/2) > DacCal['RangeAMPTEST0'][1]:
                raise Exception('CMWindow [%.3f,%.3f] V and TargetDiff %.3f V exceed range of CMRef DAC (%.3fV)' %(CMwindow,TargetDiff,DacCal['RangeAMPTEST0'][1]))
                logging.error(str('CMWindow [%.3f,%.3f] V and TargetDiff %.3f V exceed range of CMRef DAC (%.3fV)' %(CMwindow,TargetDiff,DacCal['RangeAMPTEST0'][1])))

            diff = -(-np.reshape(DacCal['CMREF_pred'],[1,len(DacCal['CMREF_pred'])]) + DacCal['AMPTEST0_pred']).astype(np.float)
            cm = 0.5 * (np.reshape(DacCal['CMREF_pred'],[1,len(DacCal['CMREF_pred'])]) + DacCal['AMPTEST0_pred'])

            diff = diff.astype(np.float)
            cond1 = ~np.isnan(cm)
            cond2 = ~np.isnan(cm)
            cond1[cond1] = cm[cond1] < CMwindow[0] # equivalent of cond1 = cm < CMwindow[0]
            cond2[cond2] = cm[cond2] > CMwindow[1] # equivalent of cond2 = cm > CMwindow[1]
            diff[cond1] = np.nan
            diff[cond2] = np.nan

            x,y = np.shape(diff)

            tmp = (abs(diff - TargetDiff)).flatten()
            error_pred = np.sort(tmp)
            idx = np.argsort(tmp)

            if error_pred[0] > MAX_ERROR[1]:
                print('Predicted error of %.1fmV exceeds DAC error threshold\n' %(error_pred[0]*1e3))
                logging.warning(str('Predicted error of %.1fmV exceeds DAC error threshold\n' %(error_pred[0]*1e3)))

            if NUM_ITERATIONS > 1:
                diff_meas = np.zeros(NUM_ITERATIONS)

            for i in range(NUM_ITERATIONS):
                CMREF = int(idx[i] % x)
                AMPTEST0 = int(np.floor(idx[i] / x))

                # success = self.API.APIComm(1, self.API.API['main']['ioctrl'], self.API.API['IO']['dac_cmref']['set'], CMREF)
                success = self.dac_cmref_set(np.uint8(CMREF))
                self.error_assert(success, 'Unable to set DAC CMREF')
                # success = self.API.APIComm(1, self.API.API['main']['ioctrl'], self.API.API['IO']['dac_amptest0']['set'], AMPTEST0)
                success = self.dac_amptest0_set(np.uint8(AMPTEST0))
                self.error_assert(success, 'Unable to set DAC AMPTEST0')

                if NUM_ITERATIONS > 1:
                    diff_meas[i] = self.CamMeasureAnaTestVoltage('DACwithSF_CMREF-AMPTEST0')

            if NUM_ITERATIONS > 1:
                idxMinErr = np.argmin(np.abs(TargetDiff - diff_meas))
                CMREF = int(idxMinErr[i] % x)
                AMPTEST0 = int(np.floor(idxMinErr[i] / x))
                # success = self.API.APIComm(1, self.API.API['main']['ioctrl'], self.API.API['IO']['dac_cmref']['set'], CMREF)
                success = self.dac_cmref_set(np.uint8(CMREF))
                self.error_assert(success, 'Unable to set DAC CMREF')
                # success = self.API.APIComm(1, self.API.API['main']['ioctrl'], self.API.API['IO']['dac_amptest0']['set'], AMPTEST0)
                success = self.dac_amptest0_set(np.uint8(AMPTEST0))
                self.error_assert(success, 'Unable to set DAC AMPTEST0')

            cmSetpoint = cm[AMPTEST0-1, CMREF-1] # get predicted CM voltage
            AMPTEST1 = np.nanargmin(np.abs(cmSetpoint - DacCal['AMPTEST1_pred'][:,CMREF-1]))
            # success = self.API.APIComm(1, self.API.API['main']['ioctrl'], self.API.API['IO']['dac_amptest1']['set'], int(AMPTEST1))
            success = self.dac_amptest1_set(np.uint8(AMPTEST1))
            self.error_assert(success, 'Unable to set DAC AMPTEST1')

            if NUM_ITERATIONS > 1:
                diff_meas_out = diff_meas[idxMinErr]
            else:
                diff_meas_out = -self.CamMeasureAnaTestVoltage('DACwithSF_CMREF-AMPTEST0')            

        elif DacSel.upper() == 'AMPTEST1-AMPTEST0':
            # Sanity check inputs
            if (np.min(CMwindow) + TargetDiff/2) > DacCal['RangeAMPTEST0'][1]:
                raise Exception('CMWindow [%.3f,%.3f] V and TargetDiff %.3f V exceed range of AMPTEST DAC (%.3fV)' %(CMwindow,TargetDiff,DacCal['RangeAMPTEST0'][1]))
                logging.error(str('CMWindow [%.3f,%.3f] V and TargetDiff %.3f V exceed range of AMPTEST DAC (%.3fV)' %(CMwindow,TargetDiff,DacCal['RangeAMPTEST0'][1])))

            if (np.max(CMwindow) - TargetDiff/2) < DacCal['RangeAMPTEST0'][0]:
                raise Exception('CMWindow [%.3f,%.3f] V and TargetDiff %.3f V exceed range of AMPTEST DAC (%.3fV)' %(CMwindow,TargetDiff,DacCal['RangeAMPTEST0'][0]))
                logging.error(str('CMWindow [%.3f,%.3f] V and TargetDiff %.3f V exceed range of AMPTEST DAC (%.3fV)' %(CMwindow,TargetDiff,DacCal['RangeAMPTEST0'][0])))


            CMREF = np.nanargmin(np.abs(np.mean(CMwindow) - DacCal['CMREF_pred']))

            # success = self.API.APIComm(1, self.API.API['main']['ioctrl'], self.API.API['IO']['dac_cmref']['set'], int(CMREF))
            success = self.dac_cmref_set(np.uint8(CMREF))
            self.error_assert(success, 'Unable to set DAC CMREF')
            time.sleep(debug_delay)

            diff = -np.reshape(DacCal['AMPTEST1_pred'][:,CMREF],[1,len(DacCal['AMPTEST1_pred'][:,CMREF])]) + np.reshape(DacCal['AMPTEST0_pred'][:,CMREF],[len(DacCal['AMPTEST0_pred'][:,CMREF]),1])
            cm = 0.5 * (np.reshape(DacCal['AMPTEST1_pred'][:,CMREF],[1,len(DacCal['AMPTEST1_pred'][:,CMREF])]) + np.reshape(DacCal['AMPTEST0_pred'][:,CMREF],[len(DacCal['AMPTEST0_pred'][:,CMREF]),1]))

            diff = -diff.astype(np.float)
            cond1 = ~np.isnan(cm)
            cond2 = ~np.isnan(cm)
            cond1[cond1] = cm[cond1] < CMwindow[0] # equivalent of cond1 = cm < CMwindow[0]
            cond2[cond2] = cm[cond2] > CMwindow[1] # equivalent of cond2 = cm > CMwindow[1]
            diff[cond1] = np.nan
            diff[cond2] = np.nan

            x,y = np.shape(diff)

            tmp = (abs(diff - TargetDiff)).flatten()
            error_pred = np.sort(tmp)
            idx = np.argsort(tmp)

            if error_pred[0] > MAX_ERROR[1]:
                print('Predicted error of %.1fmV exceeds DAC error threshold\n' %(error_pred[0]*1e3))
                logging.warning(str('Predicted error of %.1fmV exceeds DAC error threshold\n' %(error_pred[0]*1e3)))

            AMPTEST1 = int(idx[0] % x)
            AMPTEST0 = int(np.floor(idx[0] / x))

            # success = self.API.APIComm(1, self.API.API['main']['ioctrl'], self.API.API['IO']['dac_amptest0']['set'], AMPTEST0)
            success = self.dac_amptest0_set(np.uint8(AMPTEST0))
            self.error_assert(success, 'Unable to set DAC AMPTEST0')
            time.sleep(debug_delay)
            # success = self.API.APIComm(1, self.API.API['main']['ioctrl'], self.API.API['IO']['dac_amptest1']['set'], AMPTEST1)
            success = self.dac_amptest1_set(np.uint8(AMPTEST1))
            self.error_assert(success, 'Unable to set DAC AMPTEST1')
            time.sleep(debug_delay)

            diff_meas_out = self.CamMeasureAnaTestVoltage('DACwithSF_AMPTEST1-AMPTEST0')

        # print(self.regreadburst(0x0E00, 80,'increment'))
        # s = self.regwriteburst(0x0E00, valArr, 'increment')
        self.regwrite(0x0014,0x0000)

        return DacCal, {'CMREF': CMREF, 'AMPTEST0': AMPTEST0, 'AMPTEST1': AMPTEST1}, diff_meas_out

    def UpdateUseCaseFrameSetting(self, usecaseId, fieldname, fieldVal):
        """
        Updates use case frame setting
        Inputs:
            usecaseId 
            fieldname 
            fieldVal 

        Outputs:
            success: status 
        """ 

        status = 0
        prevVal = 0

        if (usecaseId < 0 or usecaseId > 15):
            return False, False

        if fieldname == 'DRAINSTYLE':
            fieldIndex = 7
        else:
            return False, False

        # Offset 7 is where the use case entries start in
        # DMEM1, there are Frame Details and 3 ROIs per use case (4)
        ADDR = 7 + usecaseId * 4

        # Using Bank Addressing Scheme instead of auto bank addressing using LSBs
        # of the address due to bug in addressing the entire address space
        #
        # Separate out Bank ID and Bank Address
        # (2 LSBs indicate Bank ID)

        BANK_ID = int(ADDR) & 3
        BANK_ADDR = ADDR >> 2
        IA_SELECT_VAL = (BANK_ID << 8) + 2

        status = self.regwrite(0x0500, IA_SELECT_VAL)
        status = status and (self.regwrite(0x0502, BANK_ADDR))

        if status:
            status, usecaseVals = self.regreadburst(0x0506, 32)
            if status:
                usecaseVals = np.squeeze(usecaseVals)
                prevVal = usecaseVals[fieldIndex - 1]

                usecaseVals[fieldIndex - 1] = fieldVal

                status = self.regwrite(0x502, BANK_ADDR)
                status = status and self.regwriteburst(0x0504, usecaseVals)

        return status, prevVal

    def tregwrite(self, regaddr, regval, regmask = 0):
        """
        Write to register while saving original value
        Inputs:
            regaddr: register address 
            regval: register value 
            regmask: register mask 

        Outputs:
            success: status of write 
        """ 
        if regmask == 0:
            origVal = self.regread(regaddr)
            success = self.regwrite(regaddr, regval)
        else:
            # if regmask == 'increment':
            #   origVal, success = self.regreadburst(regaddr, len(regval), 'increment')
            #   success = success and self.regwriteburst(regaddr, regval, 'increment')
            # else:
            origVal = self.regread(regaddr)
            success = self.regrmw(regaddr, regval, regmask)

        if (regaddr in self.origSetting['reg']['addr']) == False:
            self.origSetting['reg']['addr'] = np.append(self.origSetting['reg']['addr'], regaddr)
            self.origSetting['reg']['val'] = np.append(self.origSetting['reg']['val'], origVal)

        return success

    def WriteOrigSettings(self):
        """
        Writes back original settings to the registers based on tregwrite
        Inputs:
            none 

        Outputs:
            none 
        """ 
        rs = True
        for ns in range(len(self.origSetting['reg']['addr'])-1,-1,-1):
            # if len(self.origSetting['reg']['val'][ns]) > 1:
            #   rs = rs and self.regwriteburst(1, 
            #                                       str(self.origSetting['reg']['addr'][ns]),
            #                                       self.convert2matlabArray(self.origSetting['reg']['val'][ns]),
            #                                       'increment')
            # else:
            # rs = rs and self.regwrite(1, str(self.origSetting['reg']['addr'][ns]), int(float(self.origSetting['reg']['val'][ns])))
            rs = rs and self.regwrite(self.origSetting['reg']['addr'][ns], self.origSetting['reg']['val'][ns])

            # self.error_assert(rs, 'Cannot restore original register settings')

    def InitModes(self, clksValue_0, LockScale_0):
        """
        Initialize sensor mode
        Inputs:
            clksValue_0 
            LockScale_0 

        Outputs:
            mode: returns information of the mode
        """ 
        name = ['native_resolution', 'sub_sampled_2x', 'sub_sampled_4x', 'binned', 'dark', 'preamble', 'postamble']
        tmp = int('0x0E30', 0) + 6* np.arange(0,len(name))
        regAddrStart = tmp
        regAddrEnd = tmp + 2
        regAddrRepeat = tmp + 4

        name = np.append(name, 'binned_1x2')
        regAddrStart = np.append(regAddrStart, int('0x0E12', 0))
        regAddrEnd = np.append(regAddrEnd, int('0x0E1A', 0))
        regAddrRepeat = np.append(regAddrRepeat, int('0x0E02', 0))

        repeatCount = np.zeros(len(name))
        clksValue = np.zeros(len(name), dtype=object)
        LockScale = np.zeros(len(name), dtype=object)
        for i in range(len(name)):
            clksValue[i] = clksValue_0 
            LockScale[i] = LockScale

        mode = dict(
                    name = name,
                    repeatCount = repeatCount,
                    regAddrStart = regAddrStart,
                    regAddrEnd = regAddrEnd,
                    regAddrRepeat = regAddrRepeat,
                    clksValue = clksValue,
                    LockScale = LockScale
                    )
        return mode

    def DumpTimingTool(self, var1, var2 = 0, handles = dict()):
        """
        Read / write dump engine timing
        Inputs:
            var1: read or write to chip 
            var2: 0-> initialize Dump Engine Structure 
            handles: contains handles to add data to 

        Outputs:
            handles: Updates handles 
        """ 
        if var1 == 'ReadfromChip':
            if var2 == 0:
                handles = self.InitDE_Struct()

            chipID = self.regread(0x112)

            digPwrDownOrigState = self.regread(0x0014, handles['dispStr'])
            success = self.regwrite(0x0014, int(digPwrDownOrigState) & int('0xFFFE',0), handles['dispStr'])
            self.error_assert(success, "Failed to un-clock gate DE")

            handles['DEclkFreq'] = self.ReadDEclkFreqFromChip()

            for i in range(len(handles['mode']['name'])):
                RamStart = self.regread(handles['mode']['regAddrStart'][i]) # Read dump engine settings from chip
                RamEnd = self.regread(handles['mode']['regAddrEnd'][i])
                handles['mode']['repeatCount'][i] = self.regread(handles['mode']['regAddrRepeat'][i])
                success = self.regwrite(0x0E04, 0x0001)
                success = success and self.regwrite(0x0E06, RamStart)
                self.error_assert(success,'Failed to read dump engine settings from chip')
                success, RamVals = self.regreadburst(0x0E0A, (RamEnd-RamStart+1)*4)
                success = success and self.regwrite(0x0E04, 0x0000)
                self.error_assert(success,'Failed to read dump engine settings from chip')
                # clksValue = self.API.legacy_ConvertDumpEngineRamFormatToWaves(RamVals)
                # clksValue = np.array(clksValue) # convert to numpy array
                clksValue = self.ConvertDumpEngineRamFormatToWaves(RamVals)
                handles['mode']['clksValue'][i] = clksValue[(handles['clks']['DE_Addr']).astype('int'), :]

            success = success and self.regwrite(0x0014, digPwrDownOrigState)
            self.error_assert(success, 'Failed to restore original DE clock gate state')

            return handles
        
        elif var1 == 'WriteToChip':
            StartAddr = 0
            [regAddValPairs, ramBlob, memoryUsedPercent] = self.GenerateDumpEngineConfig(handles, StartAddr)
            print('Using', np.round(memoryUsedPercent), '%', 'of dump engine RAM')
            logging.info('Using ' + str(np.round(memoryUsedPercent)) + ' %' + ' of dump engine RAM')
            digPwrDownOrigState = self.regread(0x0014) # Load digPwrDown state
            success = self.regwrite(0x0014, int(digPwrDownOrigState) & 0xFFFE)
            self.error_assert(success, 'Failed to un-clock gate DE')
            success = success and self.regwrite(regAddValPairs[::2], regAddValPairs[1::2]) # Start/End/Repeat values
            success = success and self.regwrite(0x0E04, 0x0001) # Enable configuration of the Dump Engine RAM
            success = success and self.regwrite(0x0E06, StartAddr) # dump engine ia_addr_reg is 0x0E06
            success = success and self.regwriteburst(0x0E08, ramBlob) # Write to RAM
            success = success and self.regwrite(0x0E04, 0x0000) # Disable configuration of the Dump Engine RAM

            success = success and self.regwrite(0x0014, digPwrDownOrigState) # Restore original DE clock gate state
        else:

            return False

    def DumpTimingTransformation(self, *argv):
        """
        Perform Dump Engine transformations
        Inputs:
            variable arguments as below
            DE_struct: Dump Engine structure dictionary 
            mode: sensor mode such as native, binned etc 
            transformtion 

        Outputs:
            DE_struct: updated DE_struct
        """ 
        DE_Struct = argv[0]
        mode = argv[1]
        signalName = argv[2]
        transformation_tmp = argv[3]
        DE_Struct['mode']['clksValue'] = np.array(DE_Struct['mode']['clksValue'])

        tmp = np.where(DE_Struct['mode']['name'] == mode)
        modeIdx = tmp[0][0]

        if len(signalName) > 0: # is array
            signalIdx = np.zeros(len(signalName))
            for i in range(len(signalName)):
                if transformation_tmp.startswith('ampTestSel_'):
                    signalIdx[i] = 0
                else:
                    tmp = np.where(DE_Struct['clks']['Name'] == signalName[i])
                    signalIdx[i] = tmp[0][0]
        
        elif len(signalName) == 0:
            signalIdx = np.zeros(1)
            signalIdx[0] = 0

        transformation = np.zeros(len(signalIdx),dtype=object)
        for i in range(len(signalIdx)):
            transformation[i] = transformation_tmp

        # Perform transformations
        for signalCnt in range(len(signalIdx)):
            clkData = DE_Struct['mode']['clksValue'][modeIdx][int(signalIdx[signalCnt]),:]

            if transformation[signalCnt].startswith('ampTestSel_'):
                MUX_TRANSITION_DELAY = 1
                idx = np.where(np.array(DE_Struct['clks']['DE_Addr'],dtype='int') == 24)[0][0] # DR_Addr = 24 is amp_clk0
                ampclk = DE_Struct['mode']['clksValue'][modeIdx][idx,:]
                idx = np.where(np.array(DE_Struct['clks']['DE_Addr'],dtype='int') == 25)[0][0] # DR_Addr = 25 is amp_clk1
                ampclk = ampclk + 2 * DE_Struct['mode']['clksValue'][modeIdx][idx,:]
                tmp1 = ampclk == 1
                tmp2 = ampclk == 3
                integrateSample = tmp1 | tmp2
                integrateSample = np.roll(integrateSample, MUX_TRANSITION_DELAY)
                amp_testsel_name = np.array(['ATP','ATN','DACAMPTEST0','DACAMPTEST1','DACCMREF'])
                amp_testsel_mux = np.array([[0,0,0],[1,0,0],[0,1,0],[1,1,0],[0,0,1]])
                inputList = transformation[0].split('_')

                amp_testselp_reset_val = amp_testsel_mux[(np.where(inputList[1].upper() == amp_testsel_name))[0][0],:]
                amp_testseln_reset_val = amp_testsel_mux[(np.where(inputList[2].upper() == amp_testsel_name))[0][0],:]
                amp_testselp_int_val = amp_testsel_mux[(np.where(inputList[3].upper() == amp_testsel_name))[0][0],:]
                amp_testseln_int_val = amp_testsel_mux[(np.where(inputList[4].upper() == amp_testsel_name))[0][0],:]

                amp_testselp_idx = np.zeros(3, dtype=int)
                amp_testseln_idx = np.zeros(3, dtype=int)
                amp_testselp_idx[0] = int((np.where(DE_Struct['clks']['DE_Addr'] == 37))[0][0]) # DR_Addr = 37 is amp_testselp[0] (ADSD3100)
                amp_testselp_idx[1] = int((np.where(DE_Struct['clks']['DE_Addr'] == 42))[0][0]) # DR_Addr = 42 is amp_testselp[1] (ADSD3100)
                amp_testselp_idx[2] = int((np.where(DE_Struct['clks']['DE_Addr'] == 43))[0][0]) # DR_Addr = 43 is amp_testselp[2] (ADSD3100)
                amp_testseln_idx[0] = int((np.where(DE_Struct['clks']['DE_Addr'] == 44))[0][0]) # DR_Addr = 44 is amp_testseln[0] (ADSD3100)
                amp_testseln_idx[1] = int((np.where(DE_Struct['clks']['DE_Addr'] == 45))[0][0]) # DR_Addr = 45 is amp_testselp[1] (ADSD3100)
                amp_testseln_idx[2] = int((np.where(DE_Struct['clks']['DE_Addr'] == 46))[0][0]) # DR_Addr = 46 is amp_testselp[2] (ADSD3100)

                for k in range(3):
                    DE_Struct['mode']['clksValue'][modeIdx][amp_testselp_idx[k],:] = 0 # default all amp_testsel signals low
                    DE_Struct['mode']['clksValue'][modeIdx][amp_testseln_idx[k],:] = 0

                    DE_Struct['mode']['clksValue'][modeIdx][amp_testselp_idx[k], np.invert(integrateSample)] = amp_testselp_reset_val[k] # amp_testselp reset
                    DE_Struct['mode']['clksValue'][modeIdx][amp_testseln_idx[k], np.invert(integrateSample)] = amp_testseln_reset_val[k] # amp_testseln reset

                    DE_Struct['mode']['clksValue'][modeIdx][amp_testselp_idx[k], integrateSample] = amp_testselp_int_val[k] # amp_testselp integer
                    DE_Struct['mode']['clksValue'][modeIdx][amp_testseln_idx[k], integrateSample] = amp_testseln_int_val[k] # amp_testseln integer

            else:
                if transformation[signalCnt].startswith('copyTo'):
                    destMode = transformation[signalCnt][6:]
                    destModeIdx = np.where(DE_Struct['mode']['name'] == destMode.lower())[0][0] #+ 1

                    sourceLength = list(np.shape(clkData))[0]
                    destLength = list(np.shape(DE_Struct['mode']['clksValue'][destModeIdx]))[1]
                    DE_Struct['mode']['clksValue'][destModeIdx][(signalIdx).astype(int),:]

                    DE_Struct['mode']['clksValue'][destModeIdx] = list(DE_Struct['mode']['clksValue'][destModeIdx])
                    DE_Struct['mode']['clksValue'][modeIdx] = list(DE_Struct['mode']['clksValue'][modeIdx])


                    if sourceLength > destLength:
                        for i in range(len(DE_Struct['mode']['clksValue'][destModeIdx])):
                            append_bits = np.ones(sourceLength-destLength) * DE_Struct['mode']['clksValue'][destModeIdx][i][-1]
                            DE_Struct['mode']['clksValue'][destModeIdx][i] = np.append(DE_Struct['mode']['clksValue'][destModeIdx][i], append_bits)
                    elif sourceLength < destLength:
                        for i in range(len(DE_Struct['mode']['clksValue'][destModeIdx])):
                            DE_Struct['mode']['clksValue'][destModeIdx][i] = DE_Struct['mode']['clksValue'][destModeIdx][i][0:sourceLength]

                    DE_Struct['mode']['clksValue'][destModeIdx]  = np.array(DE_Struct['mode']['clksValue'][destModeIdx])
                    DE_Struct['mode']['clksValue'][modeIdx] = np.array(DE_Struct['mode']['clksValue'][modeIdx])

                    DE_Struct['mode']['clksValue'][destModeIdx][(signalIdx).astype(int),:] = DE_Struct['mode']['clksValue'][modeIdx][(signalIdx).astype(int),:]

                else:
                    print('Unknown tranformation')
                    logging.error('Unknown transformation' + str(transformation[signalCnt]))

            if transformation[signalCnt] != 'stretchshrink':
                DE_Struct['mode']['clksValue'][modeIdx][int(signalIdx[signalCnt])] = clkData

        return DE_Struct

    def InitDE_Struct(self, *argv):
        """
        Initialize a dump engine structure
        Inputs:
            variable arguments 

        Outputs:
            DE_struct:  Dump engine structure dictionary 
        """ 
        NCLKS_INIT = 8
        DE_Struct = dict()
        if len(argv) > 0:
            DE_Struct['bypassWaveformErrorCheck'] = argv[0]
        else:
            DE_Struct['bypassWaveformErrorCheck'] = False

        if len(argv) > 1:
            DE_Struct['dispStr'] = argv[1]
        else:
            DE_Struct['dispStr'] = ''

        DE_path = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'config/DumpTimingToolConfig_ADSD3100.csv'))

        DE_Struct['clks'], clksValue, LockScale = self.ReadConfigFile(DE_path)
        clksValue = np.transpose(np.reshape(clksValue, [1, len(clksValue)]))
        clksValue = clksValue * np.ones([1, NCLKS_INIT])

        DE_Struct['mode'] = self.InitModes(clksValue, LockScale)

        return DE_Struct

    def ReadConfigFile(self, fname):
        """
        Read dump engine configuration file (.csv)
        Inputs:
            fname: csv filename 

        Outputs:
            DE_struct: updated DE_struct 
        """ 
        clks = dict()
        clks['DE_Addr'] = []
        clks['Name'] = []
        clks['View'] = []
        DefaultValue = []
        LockScale_Default = []
        clks['ToolTip'] = []

        # path = os.path.dirname(os.path.abspath(__file__))
        # with open(os.path.join(path,fname)) as csv_file:
        with open(fname) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            line_count = 0
            for row in csv_reader:
                if line_count == 0:
                    line_count += 1
                else:
                    clks['DE_Addr'] = np.append(clks['DE_Addr'], int(row[0]))
                    clks['Name']  = np.append(clks['Name'] , row[1])
                    clks['View'] = np.append(clks['View'], bool(row[2]))
                    DefaultValue = np.append(DefaultValue, int(row[3]))
                    LockScale_Default = np.append(LockScale_Default, int(row[4]))
                    clks['ToolTip'] = np.append(clks['ToolTip'], row[5])
        csv_file.close()

        return clks, DefaultValue, LockScale_Default

    def ReadDEclkFreqFromChip(self):
        """
        Read dump engine clock frequency
        Inputs:
            none 

        Outputs:
            adc_pllFreq/clk_de_div : adc pll frequency divied by DE clock division 
        """ 
        adcPllRegisterValue = self.regread(0x010A) # Read ADC PLL register
        adc_c = int(adcPllRegisterValue) & 31
        adc_end5 =  (int(adcPllRegisterValue) >> 8) & 15
        adc_pllFreq = 24*(adc_end5 + 4*(adc_c+1))

        clkDeRegisterValue = self.regread(0x011A) # Read Clock DE register
        de_c = int(clkDeRegisterValue) & 31
        de_end5 = (int(clkDeRegisterValue) >> 8) & 15
        clk_de_div = de_end5 + 4*(de_c+1)

        return adc_pllFreq/clk_de_div

        return clks, DefaultValue, LockScale_Default

    def GenerateDumpEngineConfig(self, handles, ramStart):
        """
        Generates DE config register address value pairs
        Inputs:
            handles 
            ramStart: RAM start address 

        Outputs:
            regAddValPairs: register address value pairs 
            ramBlob: sorted clksValue in DE RAM format 
            memoryUsedPercent: RAM memory percent used 
        """ 
        ramOffset = ramStart
        ramBlob = []
        regAddValPairs = []
        signalOrderIdx = np.argsort(handles['clks']['DE_Addr'])

        for i in range(len(handles['mode']['name'])): # Error prone ,not !
            # outputData = self.API.legacy_ConvertWavesToDumpEngineRamFormat((np.array(handles['mode']['clksValue'][i])[signalOrderIdx.astype('int'),:]).astype('double'))
            outputData = self.ConvertWavesToDumpEngineRamFormat((np.array(handles['mode']['clksValue'][i])[signalOrderIdx.astype('int'),:]).astype('double'))
            regAddValPairs = np.append(regAddValPairs, np.append(handles['mode']['regAddrStart'][i], ramOffset))
            regAddValPairs = np.append(regAddValPairs, np.append(handles['mode']['regAddrEnd'][i], ramOffset + len(outputData)/4 -1))
            ramOffset = ramOffset + len(outputData)/4
            regAddValPairs = np.append(regAddValPairs, np.append(handles['mode']['regAddrRepeat'][i], handles['mode']['repeatCount'][i]))
            ramBlob = np.append(ramBlob, outputData)

        memoryUsedPercent = (ramOffset/(1024-6))*100

        if memoryUsedPercent > 100:
            raise Exception('Error using >100 percent of dump engine RAM')
            logging.error('Error using >100 percent of dump engine RAM')
        
        return regAddValPairs, ramBlob, memoryUsedPercent

    def ConvertDumpEngineRamFormatToWaves(self, RamVals):
        """
        Convert DE RAM format to wave format
        Inputs:
            RamVals: RAM values 

        Outputs:
            clksValue: clocks values 
        """ 
        RamVals = np.squeeze(np.array(RamVals).astype(np.uint16))
        DE_Vectors = RamVals.view(np.uint64) & ((2**48)-1) # signals [47:0]
        duration = RamVals[3::4] # bits [63:48]
        DE_VectorsExpanded = np.repeat(DE_Vectors, duration+1)
        if len(DE_VectorsExpanded) == 0:
            clksValue = np.array[[]]
        else:
            clksValue = np.zeros([48, len(DE_VectorsExpanded)])
            for i in range(len(DE_VectorsExpanded)):
                clksValue[:,i] = list(np.binary_repr(DE_VectorsExpanded[i],48))[::-1] 
                # Expand out individual bits, LSB first (convert to double)
        
        return np.array(clksValue)

    def ConvertWavesToDumpEngineRamFormat(self, clksValue):
        """
        Convert wave format to DE RAM format
        Inputs:
            clksValue: clocks values 

        Outputs:
            RamVals: RAM values 
            
        """ 
        DE_Vectors = np.dot(np.transpose(clksValue),np.array((2**(np.arange(0,np.shape(clksValue)[0],dtype=np.uint64)))))
        # Least significant bit first! Matrix multiplication is not supported on integer classes, so
        # keep as double precision here and convert to uint64 before adding the repeat count bits.
        idx = np.where(np.diff(np.append(-1, DE_Vectors)) != 0) # Time index for each unique signal state
        duration = np.diff(np.append(idx, len(DE_Vectors))) - 1 # Duration of each state
        RAM_Data = np.uint64(duration * (2**48)) + np.uint64(DE_Vectors[idx]) # Repeat count [63:48], signals [47:0]
        RamVals = RAM_Data.view(np.uint16) # Output in 16bit chunks
        
        return RamVals

    # MZ: Function to invert LSMOD control register in the ADSD3100, to address Tinker Intersil driver implementation
    def InvertLSMOD( self):
        """
        Function to invert LSMOD control register in the ADSD3100, to address Tinker Intersil driver implementation
        Inputs:
            none

        Outputs:
            none
            
        """ 
        lsmod_cfg1 = self.dev.readSPISingle( 0x0138,1)
        val = lsmod_cfg1[0]
        val = ((val & 0x0F) << 4) + ((lsmod_cfg1[0] & 0xF0) >> 4)
        print ("Swapped = "+ hex(val))
        self.dev.writeSPISingle(0x0138, val)
        print("LSMOD1L Config = "+ hex(val))

    #  MZ: Function to program Archer config registers (I2C only)
    def ConfigADSD3000( self,slaveID,cfg):
        """
        Function to program Archer config registers (I2C only)
        Inputs:
            slaveID: I2C slave ID 
            cfg: configuration array 

        Outputs:
            none
            
        """ 
        for i in range(cfg.shape[0]):
            addr_hi = (cfg[i,0]>>8) & 0xFF
            addr_lo = cfg[i,0] & 0xFF
            data_hi = (cfg[i,1]>>8) & 0xFF
            data_lo = cfg[i,1] & 0xFF
            self.dev.writeI2C(I2C_MODULE, slaveID, [addr_hi, addr_lo, data_hi, data_lo]) 
    
    
    def GetLDTemp( self, skipPtrSet, driver):
        """
        Function read LD temperature
        Inputs:
            skipPtrSet: skip pointer set 
            driver: driver name 

        Outputs: 
            TempOut: Temperature  
            
        """ 
        TempOut = 999
        # return TempOut
        if driver == ARCHER:
            if skipPtrSet == False:
                self.dev.writeI2C(I2C_MODULE, ADSD3000_ID, [0x1, 0x07]) # TempSense No Burst
                #self.dev.writeI2C(I2C_MODULE, ADSD3000_ID, [0x1, 0x01]) # TempSense during bursts
            rd_reg = self.dev.readI2C(I2C_MODULE, ADSD3000_ID, 2)
            print('DigTemp:',str(rd_reg[0]*0x100+rd_reg[1]))
            TempOut = (rd_reg[0]*0x100+rd_reg[1])*0.389214 - 1134.766
        if driver == INTERSIL:
            if skipPtrSet == False:
                self.dev.writeI2C(I2C_MODULE, TEMP_SENSE_ID, [TEMP_REG, 0, 0])
            #TempDig = [0,0]
            TempDig = self.dev.readI2C(I2C_MODULE, TEMP_SENSE_ID, 2)
            TempOut = (TempDig[0]*0x100 + TempDig[1])>>4
            if TempOut > 0x7FF:
                TempOut = ((TempOut ^ 0xFFF) + 1)*-0.0625
            else:
                TempOut = TempOut*0.0625
        return TempOut