"""
/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2021 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
"""
import os
import numpy as np
import shutil
import sys
import pickle


def Get_AB_Depth_XYZ(depthJsonPath):
    os.chdir("verify")
    sys.path.insert(0, '..')
    from aditofdevicepython import aditofpython as tof

    sys.path.append(os.getcwd())
    print(sys.path)

    # hacky, copy tofi_processor.obj to python exe directory for depth compute to load
    pythonEXEPath = os.path.dirname(sys.executable)
    print(pythonEXEPath)
    #shutil.copyfile(os.getcwd() + "\\tofi_processor.obj", pythonEXEPath + "\\tofi_processor.obj")

    # Native resolution (megapixel) operation, mode # 10
    eModeIndex = 5

    system = tof.System()
    status = system.initialize()
    print("system.initialize()", status)

    cameras = []
    status = system.getCameraList(cameras)
    print("system.getCameraList()", status)

    print("Cameras: ", cameras)
    camera1 = cameras[0]

    # print(os.path.abspath("config_walden.json"))
    print(depthJsonPath)

    status = camera1.initialize(depthJsonPath)
    print("camera1.initialize()", status)
    assert(status == tof.Status.Ok)

    status = camera1.powerUp()
    print("camera1.powerUp()", status)
    assert(status == tof.Status.Ok)

    status = camera1.loadModuleData()
    print("camera1.loadModuleData()", status)
    assert(status == tof.Status.Ok)

    status = camera1.setMode(eModeIndex)
    print("camera1.setMode()", status)

    types = []
    status = camera1.getAvailableFrameTypes(types)
    print("system.getAvailableFrameTypes()", status)
    assert(status == tof.Status.Ok)

    status = camera1.setFrameType(types[0])
    print("camera1.setFrameType()", status)

    status = camera1.start()
    print("camera1.start()", status)

    frame = tof.Frame()
    camera1.requestFrame(frame)
    frameDetails = tof.FrameDetails()
    status = frame.getDetails(frameDetails)

    width = frameDetails.width
    height = frameDetails.height

    # camera1.requestFrame(frame)
    # image = frame.getData(tof.FrameDataType.IR)
    # image = np.array(image, copy=True, dtype=np.uint16)
    # ab_frame = np.reshape(image[width*height*0:width*height*1], [width,height])
    # image = frame.getData(tof.FrameDataType.Depth)
    # image = np.array(image, copy=True, dtype=np.uint16)
    # depth_frame = np.reshape(image[width*height*0:width*height*1], [width,height])

    IR = frame.getData(tof.FrameDataType.IR)
    Depth = frame.getData(tof.FrameDataType.Depth)
    XYZ = frame.getData(tof.FrameDataType.XYZ)

    # print(np.shape(IR))
    # print(np.shape(Depth))
    # print(np.shape(XYZ))

    image = np.array(IR, copy=True, dtype=np.uint16)
    ab_frame = np.reshape(image[width*height*0:width*height*1], [width,height])
    image = np.array(Depth, copy=True, dtype=np.uint16)
    depth_frame = np.reshape(image[width*height*0:width*height*1], [width,height])

    print("Shutting down!")
    status = camera1.stop()
    print("camera1.stop()", status)

    del camera1

    return ab_frame, depth_frame, XYZ


if __name__ == "__main__":

    if len(sys.argv) == 3:
        json_fname = str(sys.argv[1])
        save_dir = str(sys.argv[2])
    else:
        print("FORMAT: python tof_output.py <path to json> <path to directory, def = os.getcwd()>")
        json_fname = str(sys.argv[1])
        save_dir = str(os.getcwd())

    json_fname = os.path.abspath(json_fname)
    AB, Depth, XYZ = Get_AB_Depth_XYZ(json_fname)

    AB = np.array(AB)
    Depth = np.array(Depth)
    XYZ = np.array(XYZ, dtype = np.int16)

    f = open(os.path.join(save_dir, "AB.pkl"), 'wb')
    pickle.dump(AB, f)
    f.close()

    f = open(os.path.join(save_dir, "RadialDepth.pkl"), 'wb')
    pickle.dump(Depth, f)
    f.close()

    f = open(os.path.join(save_dir, "XYZ.pkl"), 'wb')
    pickle.dump(XYZ, f)
    f.close()

    XYZ.tofile(os.path.join(save_dir, "XYZ.bin"), format='int16')