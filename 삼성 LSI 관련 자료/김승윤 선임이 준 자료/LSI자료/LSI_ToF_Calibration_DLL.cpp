#include "stdafx.h"
#include "LSI_ToF_Calibration_DLL.h"
#include "LSI_define.h"
#include "LSI_Convert.h"
#include "LSI_Lens.h"
#include "LSI_DataIO.h"
#include "LSI_FPPN.h"
#include "LSI_Wiggle.h"
#include "subutil.h"

LSI_CAL_PARAM	calParams;
CAL_SET_PARAM*	initParams;

void initCalParams(LSI_CAL_PARAM* calParams);

// Load setting parameter ("TOF_LSI_config.xml")
bool LSI_LoadSettingParams(char* path_iniFile)
{
	initParams = new CAL_SET_PARAM;
	if (ReadXML_CalSet(path_iniFile, initParams))
		return LGIT_ERROR;

	initCalParams(&calParams);

	return LGIT_OK;
}

void initCalParams(LSI_CAL_PARAM* calParams)
{
	memset(calParams, 0x00, sizeof(UINT8) * 9472);	// 9472byte
}

// Load Temperature Cal. File
bool LSI_LoadDataTemperature(int freq, unsigned char* ucTemp)
{
	string path = initParams->inPath_tempDrift;	// read path ("temp_drift" in setting file)
	string binF1 = "temp_f1.bin";
	string binF2 = "temp_f2.bin";

	PARAM_TEMPIN temp_F1;
	PARAM_TEMPIN temp_F2;

	if (freq == initParams->freq1)
	{
		if(readRawFile(path.c_str(), binF1.c_str(), 0x0000, sizeof(PARAM_TEMP), (UINT8*)&temp_F1))
			return LGIT_ERROR;

		calParams.Temperature1.lut_size = (UINT16)temp_F1.lutSize;
		calParams.Temperature1.order_X = (UINT8)temp_F1.orderX;
		calParams.Temperature1.order_Y = (UINT8)temp_F1.orderY;
		
		for (int i = 0; i < 10; i++) {
			calParams.Temperature1.coefficient[i] = temp_F1.coefficient[i];
		}

		calParams.Temperature1.checksum = getChecksum((UINT8*)(calParams.Temperature1.coefficient), sizeof(float)*calParams.Temperature1.lut_size);

		memset(calParams.Temperature1.padding1, 0xFF, sizeof(UINT8) * 12);
		memset(calParams.Temperature1.padding2, 0xFF, sizeof(UINT8) * 55);

		memcpy(ucTemp, &calParams.Temperature1, sizeof(PARAM_TEMP));
	}
	else if(freq == initParams->freq2)
	{
		if(readRawFile(path.c_str(), binF2.c_str(), 0x0000, sizeof(PARAM_TEMP), (UINT8*)&temp_F2))
			return LGIT_ERROR;

		calParams.Temperature2.lut_size = (UINT16)temp_F2.lutSize;
		calParams.Temperature2.order_X = (UINT8)temp_F2.orderX;
		calParams.Temperature2.order_Y = (UINT8)temp_F2.orderY;

		for (int i = 0; i < 10; i++) {
			calParams.Temperature2.coefficient[i] = temp_F2.coefficient[i];
		}

		calParams.Temperature2.checksum = getChecksum((UINT8*)(calParams.Temperature2.coefficient), sizeof(float)*calParams.Temperature2.lut_size);

		memset(calParams.Temperature2.padding1, 0xFF, sizeof(UINT8) * 12);
		memset(calParams.Temperature2.padding2, 0xFF, sizeof(UINT8) * 55);

		memcpy(ucTemp, &calParams.Temperature2, sizeof(PARAM_TEMP));
	}
	else
	{
		return LGIT_ERROR;
	}

	return LGIT_OK;
}

// Load wiggling data ("wiggling")
bool LSI_LoadDataWiggling(short *nshfl, short *shfl, int freq)
{
	string sFreq;
	if		(freq == initParams->freq1)		sFreq = "f1";
	else if (freq == initParams->freq2)		sFreq = "f2";
	else									return LGIT_ERROR;

	if (readRawFileWigg(initParams->inPath_wigg, nshfl, sFreq, "nshfl", initParams->wiggCode, initParams->wiggStep))
		return LGIT_ERROR;
	if (readRawFileWigg(initParams->inPath_wigg, shfl, sFreq, "shfl", initParams->wiggCode, initParams->wiggStep))
		return LGIT_ERROR;

	return LGIT_OK;
}

// wiggling Cal. Processing
bool LSI_CalibrationWiggling(short* nshfl, short* shfl, int freq, unsigned char* ucWigg)
{
	int WIGG_STEP = initParams->wiggStep;
	float* w_phase = new float[LSI_SIZE*WIGG_STEP];
	float* w_avg = new float[WIGG_STEP];

	double* xx = new double[WIGG_STEP];
	double* yy = new double[WIGG_STEP];
	double* yResult = new double[WIGG_LUT_SIZE];

	// 1. converting data (phase->angle)
	Convert_phaseToAngle_wigg(nshfl, shfl, w_phase, LSI_WID, LSI_HGT, WIGG_STEP);
	// 2. averaging wiggling ROI
	Convert_average_wigg(w_phase, w_avg, WIGG_STEP, WIGG_ROI_WID, WIGG_ROI_HGT);
	vector<float>temp_w_avg(WIGG_STEP);
	Convert_average_wigg(w_phase, (float*)temp_w_avg.data(), WIGG_STEP, WIGG_ROI_WID, WIGG_ROI_HGT);
	TestGraph(temp_w_avg);

	// 3. calculation difference (real vs measure)
	Wiggle_computeDiff(w_avg, xx, yy, WIGG_STEP);
	// 4. fitting (step->2048)
	Wiggle_fitting(xx, yy, yResult, WIGG_STEP);

	if (freq == initParams->freq1)
	{
		Wiggle_eeprom(yResult, &calParams.wiggling1);
		memcpy(ucWigg, &calParams.wiggling1, sizeof(PARAM_WIGG));
	}
	else if (freq == initParams->freq2)
	{
		Wiggle_eeprom(yResult, &calParams.wiggling2);
		memcpy(ucWigg, &calParams.wiggling2, sizeof(PARAM_WIGG));
	}
	else
	{
		return LGIT_ERROR;
	}

	delete w_phase;
	delete w_avg;
	delete xx;
	delete yy;
	delete yResult;

	return LGIT_OK;
}

// Load Lens data
bool LSI_LoadDataLens(short* nshfl_on, short* shfl_on, short* nshfl_off, short* shfl_off)
{
	if (meanFromPhaseFiles(initParams->inPath_ledOn, nshfl_on, "f2", "nshfl", initParams->cnt_Lens))
		return LGIT_ERROR;
	if (meanFromPhaseFiles(initParams->inPath_ledOn, shfl_on, "f2", "shfl", initParams->cnt_Lens))
		return LGIT_ERROR;

	if (meanFromPhaseFiles(initParams->inPath_ledOff, nshfl_off, "f2", "nshfl", initParams->cnt_Lens))
		return LGIT_ERROR;
	if(meanFromPhaseFiles(initParams->inPath_ledOff, shfl_off, "f2", "shfl", initParams->cnt_Lens))
		return LGIT_ERROR;

	return LGIT_OK;
}

// Lnes Cal. Processing
bool LSI_CalibrationLens(short* nshfl_on, short* shfl_on, short* nshfl_off, short* shfl_off, float* gt, unsigned char* ucLens)
{
	float* intensity_on = new float[LSI_SIZE];
	float* intensity_off = new float[LSI_SIZE];
	float* intensity_diff = new float[LSI_SIZE];
	double* lensParams = NULL;
	int fov[2] = { 0, 0 };

	Convert_phase2int(nshfl_on, shfl_on, intensity_on, LSI_WID, LSI_HGT);
	Convert_phase2int(nshfl_off, shfl_off, intensity_off, LSI_WID, LSI_HGT);
	
	LENS_CAL_OBJECT* lens_obj = new LENS_CAL_OBJECT;
	ReadJson_LensCalSet(initParams->iniPath, lens_obj, &(lens_obj->pattern_mask));

	for (int i = 0; i < LSI_SIZE; i++) 
	{
		intensity_diff[i] = intensity_on[i] - intensity_off[i];
	}

	// Lens Main
	if (LENS_CalibrationMain(lens_obj, intensity_diff, gt) == LGIT_ERROR) {
		return LGIT_ERROR;
	}

	lensParams = lens_obj->lens_params;
	fov[0] = lens_obj->fov_h;
	fov[1] = lens_obj->fov_v;
	
	Lens_eeprom(fov, lensParams, &calParams.lenscal);
	memcpy(ucLens, &calParams.lenscal, sizeof(PARAM_LENS));

	delete[] intensity_on;
	delete[] intensity_off;
	delete[] intensity_diff;
	delete lens_obj->pattern_mask;
	delete lens_obj;

	return LGIT_OK;
}

// Load FPPN data
bool LSI_LoadDataFppn(short* nshfl_off, short* shfl_off, byte* emb_nshfl_off, int freq)
{
	string sFreq;
	if		(freq == initParams->freq1)		sFreq = "f1";
	else if (freq == initParams->freq2)		sFreq = "f2";
	else									return LGIT_ERROR;

	if (meanFromPhaseFiles(initParams->inPath_ledOff, nshfl_off, sFreq, "nshfl", initParams->cnt_Fppn))
		return LGIT_ERROR;
	if (meanFromPhaseFiles(initParams->inPath_ledOff, shfl_off, sFreq, "shfl", initParams->cnt_Fppn))
		return LGIT_ERROR;
	if (meanFromPhaseFiles_emb(initParams->inPath_ledOff, emb_nshfl_off, initParams->cnt_Fppn))
		return LGIT_ERROR;

	return LGIT_OK;
}

// FPPN Cal. Processing
bool LSI_CalibrationFppn(short* nshfl_off, short* shfl_off, byte* emb_nshfl_off, float* gt, int freq, unsigned char* ucFppn)
{
	float* norm = new float[LSI_SIZE];
	float* phase = new float[LSI_SIZE];
	float* phaseGT = new float[LSI_SIZE];
	float* depthGT = new float[LSI_SIZE];
	float* depth = new float[LSI_SIZE];

	Convert_phaseToAngle(nshfl_off, shfl_off, phase, LSI_WID, LSI_HGT);
	Convert_scaleValue(phase, LSI_SIZE, 1.0f / 2.0f / PI, norm);
	Convert_scaleGT(gt, LSI_SIZE, depth);
	Convert_distanceToAngle(depth, freq, LSI_SIZE, depthGT);
	Convert_scaleValue(depthGT, LSI_SIZE, 1.0f / 2.0 / PI, phaseGT);

	delete depthGT;
	delete phase;
	delete depth;

	if (freq == initParams->freq1)
	{
		FPPN_getParams(norm, &(calParams.fppn1), &(calParams.Temperature1), &(calParams.wiggling1), phaseGT, emb_nshfl_off);
		memcpy(ucFppn, &calParams.fppn1, sizeof(PARAM_FPPN));
	}
	else if (freq == initParams->freq2)
	{
		FPPN_getParams(norm, &(calParams.fppn2), &(calParams.Temperature2), &(calParams.wiggling2), phaseGT, emb_nshfl_off);
		memcpy(ucFppn, &calParams.fppn2, sizeof(PARAM_FPPN));
	}
	else
	{
		return LGIT_ERROR;
	}
	
	delete phaseGT;	
	delete norm;

	return LGIT_OK;
}

// write total bin file(EEPROM) 
bool LSI_WriteTotalBin(unsigned char* ucLens, unsigned char* ucTempF1, unsigned char* ucTempF2, unsigned char* ucWiggF1, unsigned char* ucWiggF2, unsigned char* ucFppnF1, unsigned char* ucFppnF2)
{
	LSI_CAL_PARAM	paramCal;

	char* path_TotalBin = (char*)initParams->outPath_total.c_str();
	char* file_Totalbin = (char*)initParams->outBin_total.c_str();
	
	if (writeParam_Totalbin(path_TotalBin, file_Totalbin, 0x0000, paramCal))
		return LGIT_ERROR;

	if (writeParam_Totalparam(path_TotalBin, file_Totalbin, 0x0020, sizeof(PARAM_TEMP), ucTempF1))
		return LGIT_ERROR;
	if (writeParam_Totalparam(path_TotalBin, file_Totalbin, 0x0090, sizeof(PARAM_TEMP), ucTempF2))
		return LGIT_ERROR;

	if (writeParam_Totalparam(path_TotalBin, file_Totalbin, 0x0100, sizeof(PARAM_LENS), ucLens))
		return LGIT_ERROR;

	if (writeParam_Totalparam(path_TotalBin, file_Totalbin, 0x0200, sizeof(PARAM_WIGG), ucWiggF1))
		return LGIT_ERROR;
	if (writeParam_Totalparam(path_TotalBin, file_Totalbin, 0x1280, sizeof(PARAM_WIGG), ucWiggF2))
		return LGIT_ERROR;

	if (writeParam_Totalparam(path_TotalBin, file_Totalbin, 0x2300, sizeof(PARAM_FPPN), ucFppnF1))
		return LGIT_ERROR;
	if (writeParam_Totalparam(path_TotalBin, file_Totalbin, 0x2400, sizeof(PARAM_FPPN), ucFppnF2))
		return LGIT_ERROR;

	return LGIT_OK;

}

// Write total Cal. data (w/input)
void LSI_OutTotalCalData(unsigned char* ucLens, unsigned char* ucTempF1, unsigned char* ucTempF2, unsigned char* ucWiggF1, unsigned char* ucWiggF2, unsigned char* ucFppnF1, unsigned char* ucFppnF2, unsigned char* ucParams)
{
	calParams.comm_version = 20303;
	calParams.comm_versionTM = 20201;
	calParams.comm_board_offset_f1 = 0;
	calParams.comm_board_offset_f2 = 0;

	for (int i = 0; i < 16; i++)
	{
		calParams.padding[i] = 0xFF;
	}
	
	memcpy(&calParams.lenscal, ucLens, sizeof(PARAM_LENS));
	memcpy(&calParams.Temperature1, ucTempF1, sizeof(PARAM_TEMP));
	memcpy(&calParams.Temperature2, ucTempF2, sizeof(PARAM_TEMP));
	memcpy(&calParams.wiggling1, ucWiggF1, sizeof(PARAM_WIGG));
	memcpy(&calParams.wiggling2, ucWiggF2, sizeof(PARAM_WIGG));
	memcpy(&calParams.fppn1, ucFppnF1, sizeof(PARAM_FPPN));
	memcpy(&calParams.fppn2, ucFppnF2, sizeof(PARAM_FPPN));

	memcpy(ucParams, &calParams, sizeof(LSI_CAL_PARAM));
}

// Write total Cal. data (wo/input)
void LSI_OutTotalCalDataBuf(unsigned char* ucParams)
{
	calParams.comm_version = 20303;		// LSI Calibration library version
	calParams.comm_versionTM = 20201;	// LSI Temp. Drift Cal. version
	calParams.comm_board_offset_f1 = 0;
	calParams.comm_board_offset_f2 = 0;

	for (int i = 0; i < 16; i++)
	{
		calParams.padding[i] = 0xFF;
	}

	memcpy(ucParams, &calParams, sizeof(LSI_CAL_PARAM));
}





////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// 20.09.21 add image path /////////////////////////////////
bool LSI_LoadDataTemperature_path(char* path_TempDrift, int freq, unsigned char* ucTemp)
{
	string binF1 = "temp_f1.bin";
	string binF2 = "temp_f2.bin";

	PARAM_TEMPIN temp_F1;
	PARAM_TEMPIN temp_F2;

	if (freq == initParams->freq1)
	{
		if (readRawFile(path_TempDrift, binF1.c_str(), 0x0000, sizeof(PARAM_TEMP), (UINT8*)&temp_F1))
			return LGIT_ERROR;

		calParams.Temperature1.lut_size = (UINT16)temp_F1.lutSize;
		calParams.Temperature1.order_X = (UINT8)temp_F1.orderX;
		calParams.Temperature1.order_Y = (UINT8)temp_F1.orderY;

		for (int i = 0; i < 10; i++) {
			calParams.Temperature1.coefficient[i] = temp_F1.coefficient[i];
		}

		calParams.Temperature1.checksum = getChecksum((UINT8*)(calParams.Temperature1.coefficient), sizeof(float)*calParams.Temperature1.lut_size);

		memset(calParams.Temperature1.padding1, 0xFF, sizeof(UINT8) * 12);
		memset(calParams.Temperature1.padding2, 0xFF, sizeof(UINT8) * 55);

		memcpy(ucTemp, &calParams.Temperature1, sizeof(PARAM_TEMP));
	}
	else if (freq == initParams->freq2)
	{
		if (readRawFile(path_TempDrift, binF2.c_str(), 0x0000, sizeof(PARAM_TEMP), (UINT8*)&temp_F2))
			return LGIT_ERROR;

		calParams.Temperature2.lut_size = (UINT16)temp_F2.lutSize;
		calParams.Temperature2.order_X = (UINT8)temp_F2.orderX;
		calParams.Temperature2.order_Y = (UINT8)temp_F2.orderY;

		for (int i = 0; i < 10; i++) {
			calParams.Temperature2.coefficient[i] = temp_F2.coefficient[i];
		}

		calParams.Temperature2.checksum = getChecksum((UINT8*)(calParams.Temperature2.coefficient), sizeof(float)*calParams.Temperature2.lut_size);

		memset(calParams.Temperature2.padding1, 0xFF, sizeof(UINT8) * 12);
		memset(calParams.Temperature2.padding2, 0xFF, sizeof(UINT8) * 55);


		memcpy(ucTemp, &calParams.Temperature2, sizeof(PARAM_TEMP));
	}
	else
	{
		return LGIT_ERROR;
	}

	return LGIT_OK;
}


bool LSI_LoadDataLens_path(char* path_ImgLEDon, char* path_ImgLEDoff, short* nshfl_on, short* shfl_on, short* nshfl_off, short* shfl_off)
{
	if (meanFromPhaseFiles(path_ImgLEDon, nshfl_on, "f2", "nshfl", initParams->cnt_Lens))
		return LGIT_ERROR;
	if (meanFromPhaseFiles(path_ImgLEDon, shfl_on, "f2", "shfl", initParams->cnt_Lens))
		return LGIT_ERROR;

	if (meanFromPhaseFiles(path_ImgLEDoff, nshfl_off, "f2", "nshfl", initParams->cnt_Lens))
		return LGIT_ERROR;
	if (meanFromPhaseFiles(path_ImgLEDoff, shfl_off, "f2", "shfl", initParams->cnt_Lens))
		return LGIT_ERROR;

	return LGIT_OK;
}

bool LSI_LoadDataWiggling_path(char* path_ImgWigg, short *nshfl, short *shfl, int freq)
{
	string sFreq;
	if		(freq == initParams->freq1)		sFreq = "f1";
	else if (freq == initParams->freq2)		sFreq = "f2";
	else									return LGIT_ERROR;

	if (readRawFileWigg(path_ImgWigg, nshfl, sFreq, "nshfl", initParams->wiggCode, initParams->wiggStep))
		return LGIT_ERROR;
	if (readRawFileWigg(path_ImgWigg, shfl, sFreq, "shfl", initParams->wiggCode, initParams->wiggStep))
		return LGIT_ERROR;

	return LGIT_OK;
}


bool LSI_LoadDataFppn_path(char* path_imgOff, short* nshfl_off, short* shfl_off, byte* emb_nshfl_off, int freq)
{
	string sFreq;
	if		(freq == initParams->freq1)		sFreq = "f1";
	else if (freq == initParams->freq2)		sFreq = "f2";
	else									return LGIT_ERROR;

	if (meanFromPhaseFiles(path_imgOff, nshfl_off, sFreq, "nshfl", initParams->cnt_Fppn))
		return LGIT_ERROR;
	if (meanFromPhaseFiles(path_imgOff, shfl_off, sFreq, "shfl", initParams->cnt_Fppn))
		return LGIT_ERROR;
	if (meanFromPhaseFiles_emb(path_imgOff, emb_nshfl_off, initParams->cnt_Fppn))
		return LGIT_ERROR;

	return LGIT_OK;
}