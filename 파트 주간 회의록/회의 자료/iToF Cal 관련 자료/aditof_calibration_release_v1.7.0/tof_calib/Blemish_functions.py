import cv2
import numpy as np
import sys
import time
import os
import json
import numpy as np
import array as arr

from imatest.it import ImatestLibrary, ImatestException
import json

# Initialize Imatest Library
imatestLib = ImatestLibrary()

def BlemishProcess(avgFrames, root_dir, ini_file, results_dir): 
  
    # root_dir = 'C:\\Users\\posulliv\\AppData\\Roaming\\Imatest'
    # ini_file = 'C:\\Users\\posulliv\\AppData\\Roaming\\Imatest\\Blemish.ini'

    raw_data = None

    if '_tempCalFiles' in os.listdir():
        pass
    else:
        os.mkdir('_tempCalFiles')

    BaseFile = os.path.join('_tempCalFiles', 'tmp_intermediateFile_blemish')

    np.save(BaseFile,avgFrames) #Passing data directly
    with open(BaseFile+'.npy', 'rb') as fsock:  #Passing data directly
        raw_data = fsock.read()

    try:
        json_args = imatestLib.build_json_args(width=1024,
                                        height=1024,
                                        ncolors=1,
                                        extension='npy',
                                        filename=results_dir)
    
        result = imatestLib.blemish(input_file=None,
                                    root_dir=root_dir,
                                    op_mode=ImatestLibrary.OP_MODE_DIRECT_READ,
                                    ini_file=ini_file,
                                    raw_data=raw_data[128:],
                                    json_args=json_args)

        
        json_result = json.loads(result)
        blemish_count = json_result['blemishResults']['N_blemish_count']

        print('Blemish count = ' + str(blemish_count))
        if (blemish_count[0] > 0):
            clusters = json_result['blemishResults']['blemishSizePxls']
            print('Cluster sizes = ' + str(clusters))
            print('Total bad pixels = ' + str(sum(clusters)))
        print('Saturated pixel count = ' + str(json_result['blemishResults']['nHotPixels']))
        print('Dead pixel count = ' + str(json_result['blemishResults']['nDeadPixels']))

    except ImatestException as iex:
        if iex.error_id == ImatestException.FloatingLicenseException:
            print("All floating license seats are in use.  Exit Imatest on another computer and try again.")
        elif iex.error_id == ImatestException.LicenseException:
            print("License Exception: " + iex.message)
        else:
            print(iex.message)
    except Exception as ex:
        print(str(ex))

    #imatestLib.terminate_library()

    return json_result
