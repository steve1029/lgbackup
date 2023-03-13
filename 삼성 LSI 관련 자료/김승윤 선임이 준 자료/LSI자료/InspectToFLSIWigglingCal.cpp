#include "StdAfx.h"
#include "InspectToFLSIWigglingCal.h"
#include <vector>
#include <ctime>
#include "LibAtomImage\Include\AtomImageOpticalCenter.h"
#include "TestDEPSDll.h"
#include "Image_3D_Util.h"
#include <opencv2/opencv.hpp>
#include <direct.h>
#include "LSI_ToF_Cal.h"
//#include "LSI_ToF_Calibration_DLL.h"
#include "..\..\Inspection\iccsd_incabinsolution\LSI_ToF_Calibration_DLL\LSI_ToF_Calibration_DLL\LSI_ToF_Calibration_DLL.h"



#ifdef _DEBUG
//#pragma comment(lib, "opencv_contrib2410d.lib")
#pragma comment(lib, "opencv_world420d.lib")
#else
//#pragma comment(lib, "opencv_contrib2410.lib")
#pragma comment(lib, "opencv_world420.lib")
#endif

#pragma comment(lib, "LSI_ToF_Calibration_DLL.lib")

using namespace AtomUtil;
using namespace AtomSoftISP;
using namespace std;

IMPLEMENT_INSPECTION_DYNCREATE(CInspectToFLSIWigglingCal, "ToFLSIWigglingCal")


CInspectToFLSIWigglingCal::CInspectToFLSIWigglingCal(int nData, BYTE *& _FrameBuffer, MeImageObject & _2ByteFrame, MeImageObject & _8BitFrame,
	MeImageObject & _Image, MeImageObject & _YImage, IImageSensor & _Sensor, IBoardControl & _Board, IGraphic* pGO)
	: CInspectionInterface(nData, _FrameBuffer, _2ByteFrame, _8BitFrame, _Image, _YImage, _Sensor, _Board, pGO), m_pLog{ false }
{
	m_pTestDEPSDll = new CTestDEPSDll;
	//m_pSerialComm = &(CSerialControl::GetInstance());
	m_pSerialComm = new SerialComm;
}

CInspectToFLSIWigglingCal::~CInspectToFLSIWigglingCal(void)
{
	if (m_pLog)
	{
		delete m_pLog;
	}

	if (m_pSerialComm)
	{
		delete m_pSerialComm;
	}



	if(m_pTestDEPSDll)
	{
		delete m_pTestDEPSDll;
	}
}

void CInspectToFLSIWigglingCal::Init()
{
	TimeRecord("[Init]",0);
	int nObjCount = GetObjectCount();

	m_pLog = new CInspectionLog(_T("ToFLSIWigglingCal"), nObjCount, m_pfnStartLog, m_pfnWriteLog);
	MeString strHeader;
	strHeader = _T("result,depth_cm,f1_nshfl_mean0,f1_nshfl_mean1,f1_nshfl_mean2,f1_nshfl_mean3,f1_shfl_mean0,f1_shfl_mean1,f1_shfl_mean2,f1_shfl_mean3,f2_nshfl_mean0,f2_nshfl_mean1,f2_nshfl_mean2,f2_nshfl_mean3,f2_shfl_mean0,f2_shfl_mean1,f2_shfl_mean2,f2_shfl_mean3,");
	//	strHeader += "test";
	m_pLog->WriteLogHeader(strHeader);
	TimeRecord("[Init]",1);


}

void CInspectToFLSIWigglingCal::SetImageFormat(void)
{
	TimeRecord("[OnUpdateSpec],[SetImageFormat]",0);
	int nSuperFrameWidth = m_ImageObject.GetWidth();
	int nSuperFrameHeight = m_ImageObject.GetHeight();
	int nWidth = nSuperFrameWidth;
	int nHeightEmbedded = nSuperFrameHeight / m_nFrameNumPerSuperFrame;
	int nHeight = nHeightEmbedded - m_nEmbeddedLineCount;

	m_vRawImageWembedded.clear();
	m_vRawImageWOembedded.clear();
	m_vPhaseSplitImage.clear();
	for(int i = 0; i < m_nFrameNumPerSuperFrame; i++)
	{
		m_vRawImageWembedded.push_back(Mat::zeros(nHeightEmbedded, nWidth, CV_16SC1));
		m_vRawImageWOembedded.push_back(Mat::zeros(nHeight, nWidth, CV_16SC1));
		m_vPhaseSplitImage.push_back(Mat::zeros(nHeight, nWidth, CV_8UC1));
	}
	m_vRawImageMIPI10 = vector<vector<unsigned char>>(m_nFrameNumPerSuperFrame,  vector<unsigned char>(nWidth * nHeightEmbedded * 5 / 4));//m_nEmbeddedDataFileSize
	m_vEmbedded = vector<vector<unsigned char>>(m_nFrameNumPerSuperFrame,  vector<unsigned char>(m_nEmbeddedDataFileSize, 0xff));//
	m_tempImageObject.Create(nWidth, nHeight, 8, 1);

	m_fn_map["f1_nshfl"]=0;
	m_fn_map["f1_shfl"]=1;
	m_fn_map["f2_nshfl"]=2;
	m_fn_map["f2_shfl"]=3;

	m_vsMIPI10Format.clear();
	m_vsMIPI10Format.push_back("_%02d_ALL%d_%s_MIPI10.raw");
	m_vsMIPI10Format.push_back("_%02d_ALL%d_%s_MIPI10.raw");
	m_vsMIPI10Format.push_back("_%02d_ALL%d_%s_MIPI10.raw");
	m_vsMIPI10Format.push_back("_%02d_ALL%d_%s_MIPI10.raw");

	m_vsRaw16wEmbdFormat.clear();
	m_vsRaw16wEmbdFormat.push_back("_%02d_ALL%d_%s_wEmbd.raw");
	m_vsRaw16wEmbdFormat.push_back("_%02d_ALL%d_%s_wEmbd.raw");
	m_vsRaw16wEmbdFormat.push_back("_%02d_ALL%d_%s_wEmbd.raw");
	m_vsRaw16wEmbdFormat.push_back("_%02d_ALL%d_%s_wEmbd.raw");

	m_vsEmbeddedFormat.clear();
	m_vsEmbeddedFormat.push_back("_%02d_ALL%d_%s_VC(0)_DT(Embedded_Data).raw");
	m_vsEmbeddedFormat.push_back("_%02d_ALL%d_%s_VC(0)_DT(Embedded_Data).raw");
	m_vsEmbeddedFormat.push_back("_%02d_ALL%d_%s_VC(0)_DT(Embedded_Data).raw");
	m_vsEmbeddedFormat.push_back("_%02d_ALL%d_%s_VC(0)_DT(Embedded_Data).raw");

	m_vsRaw16Format.clear();
	m_vsRaw16Format.push_back("_%02d_ALL%d_%s.raw");
	m_vsRaw16Format.push_back("_%02d_ALL%d_%s.raw");
	m_vsRaw16Format.push_back("_%02d_ALL%d_%s.raw");
	m_vsRaw16Format.push_back("_%02d_ALL%d_%s.raw");
	TimeRecord("[OnUpdateSpec],[SetImageFormat]",1);
}

void CInspectToFLSIWigglingCal::SetCalImage(void)
{
	TimeRecord("[OnUpdateSpec],[SetCalImage]",0);
	int nSuperFrameWidth = m_ImageObject.GetWidth();
	int nSuperFrameHeight = m_ImageObject.GetHeight();
	int nWidth = nSuperFrameWidth;
	int nHeightEmbedded = nSuperFrameHeight / m_nFrameNumPerSuperFrame;
	int nHeight = nHeightEmbedded - m_nEmbeddedLineCount;

	m_sCalImageSet.wiggling_nshfl_f1.create(Size(nWidth, nHeight * MAX_PHASE_SHIFT_NUM), CV_16SC1);
	m_sCalImageSet.wiggling_shfl_f1.create(Size(nWidth, nHeight * MAX_PHASE_SHIFT_NUM), CV_16SC1);
	m_sCalImageSet.wiggling_nshfl_f2.create(Size(nWidth, nHeight * MAX_PHASE_SHIFT_NUM), CV_16SC1);
	m_sCalImageSet.wiggling_shfl_f2.create(Size(nWidth, nHeight * MAX_PHASE_SHIFT_NUM), CV_16SC1);

	m_sCalImageSet.LEDoff_nshfl_f1.create(Size(nWidth, nHeight), CV_16SC1);
	m_sCalImageSet.LEDoff_shfl_f1.create(Size(nWidth, nHeight), CV_16SC1);
	m_sCalImageSet.LEDoff_nshfl_f2.create(Size(nWidth, nHeight), CV_16SC1);
	m_sCalImageSet.LEDoff_shfl_f2.create(Size(nWidth, nHeight), CV_16SC1);

	m_sCalImageSet.LEDon_nshfl_f1.create(Size(nWidth, nHeight), CV_16SC1);
	m_sCalImageSet.LEDon_shfl_f1.create(Size(nWidth, nHeight), CV_16SC1);
	m_sCalImageSet.LEDon_nshfl_f2.create(Size(nWidth, nHeight), CV_16SC1);
	m_sCalImageSet.LEDon_shfl_f2.create(Size(nWidth, nHeight), CV_16SC1);

	m_sCalImageSet.gt.create(Size(nWidth / 2, nHeight / 2), CV_32FC1);

	m_sCalImageSet.LEDoff_nshfl_f1_embd.resize(5120);
	m_sCalImageSet.LEDoff_nshfl_f2_embd.resize(5120);
	TimeRecord("[OnUpdateSpec],[SetCalImage]",1);
}


void CInspectToFLSIWigglingCal::OnUpdateSpec(CSpecValue::MapSpec& spec)
{
	//TimeRecord("[OnUpdateSpec]",0);
	CSpecHelper helper(spec);
	// AE & AWB
	CInspectGeneric::OnUpdateSpecAEAWB(spec);
	//env
	m_bSaveImage = helper.GetValueInt("env.SaveImage", 0) == 1 ? true : false;
	m_sImagePath = helper.GetValueString("env.ImagePath", "c:/log");	
	m_UIDLLFlag = helper.GetValueInt("env.UIDLLFlag", -1);
	m_DelayForPreheating = helper.GetValueInt("env.DelayForPreheating", -1);
	m_sInspectSite = (string)helper.GetValueString("env.InspectSite", "");
	//LED setting
	m_bUseLED = helper.GetValueInt("LED.enable", 0) == 1 ? true : false;
	m_nCOMportNumLED = helper.GetValueInt("LED.COMportNum", -1);
	m_nCOMportBaudRateLED = helper.GetValueInt("LED.COMportBaudRate", -1);
	m_nDelayAfterLEDON = helper.GetValueInt("LED.DelayAfterLEDON", -1);
	m_nDelayAfterLEDOFF = helper.GetValueInt("LED.DelayAfterLEDOFF", -1);
	//image format
	m_nFrameNumPerSuperFrame = helper.GetValueInt("image.FrameNumPerSuperFrame", -1);
	m_nSuperFrameCountPerDistance = helper.GetValueInt("image.SuperFrameCountPerDistance", -1);
	m_nEmbeddedLineCount = helper.GetValueInt("image.EmbeddedLineCount", -1);
	m_nEmbeddedDataFileSize = helper.GetValueInt("image.EmbeddedDataFileSize", -1);
	m_bEnablePNGimage = helper.GetValueInt("image.enablePNGimage", 0);
	m_bEnableMIPIimage = helper.GetValueInt("image.enableMIPIimage", 0);
	m_bEnableRawWembdImage = helper.GetValueInt("image.enableRawWembdImage", 0);
	m_bEnableEmbeddedImage = helper.GetValueInt("image.enableEmbeddedImage", 0);
	m_bEnableRawWoEmbdImage = helper.GetValueInt("image.enableRawWoEmbdImage", 0);
	m_bEnableDepthView = helper.GetValueInt("image.enableDepthView", 0);
	m_bEnableDepthImage = helper.GetValueInt("image.enableDepthImage", 0);
	m_bEnableDepth3Dimage = helper.GetValueInt("image.enableDepth3Dimage", 0);
	m_bEnableDepthCSV = helper.GetValueInt("image.enableDepthCSV", 0);
	m_dNormDepthMin = helper.GetValueDouble("image.NormDepthMin", 0);
	m_dNormDepthMax = helper.GetValueDouble("image.NormDepthMax", 0);
	//cal
	m_nFreqF1 = helper.GetValueInt("cal.Frequency1", 0);
	m_nFreqF2 = helper.GetValueInt("cal.Frequency2", 0);
	m_nDelayAfterPhaseShiftChange = helper.GetValueInt("cal.DelayAfterPhaseShiftChange", -1);
	m_bEnableWiggCapture = helper.GetValueInt("cal.enableWiggCapture", 0);
	m_bEnableWiggCaptureThread = helper.GetValueInt("cal.enableWiggCaptureThread", 0)== 1 ? true : false;
	m_bEnableLEDOffCapture = helper.GetValueInt("cal.enableLEDOffCapture", 0);
	m_bEnableLEDonCapture = helper.GetValueInt("cal.enableLEDonCapture", 0);
	m_bEnableCalAlgo = helper.GetValueInt("cal.enableCalAlgo", 0);
	m_bEnableSaveCalBin = helper.GetValueInt("cal.enableSaveCalBin", 0);
	m_sCalFilePath = (string)helper.GetValueString("cal.InputCalFilePathForDepthDebug", "");
	m_sCalParamsLSIlibXMLpath = (string)helper.GetValueString("cal.CalParamsLSIlibXMLpath", "");
	m_sPath_Tempbin = (string)helper.GetValueString("cal.TempBinFolder", "");
	m_sCalParamsXMLpath = (string)helper.GetValueString("cal.CalParamsLGITlibXMLpath", "");
	m_nWiggStep = helper.GetValueInt("cal.WiggStep", 0);

	//write cal
	m_bWriteCalData = helper.GetValueInt("eeprom.WriteCalData", 0)? true : false;
	m_nWriteCalAddr = helper.GetValueInt("eeprom.WriteCalAddr", 0);
	string temp_m_nWriteCalAddr = (string)helper.GetValueString("eeprom.WriteCalAddr", 0);
	string temp_m_nWriteCalSize = (string)helper.GetValueString("eeprom.WriteCalSize", 0);
	
	m_nWriteCalAddr = (int)strtol(temp_m_nWriteCalAddr.c_str(),NULL, 0);
	m_nWriteCalSize = (int)strtol(temp_m_nWriteCalSize.c_str(), NULL, 0);

	m_vPreVerifyCalData.clear();
	std::string strItem;
	CxString strTemp = helper.GetValueString("eeprom.PreVerifyCalData");

	


	while( strTemp.GetLength() )
	{
		strItem = strTemp.ExtractLeft(_T(';'));
		m_vPreVerifyCalData.push_back(strItem);
	}

	m_bReReadCalData = helper.GetValueInt("eeprom.ReReadCalData", 0)? true : false;

	{
		
		m_nRoi_Wid = helper.GetValueInt("val.Roi_Wid", 0);
		m_nRoi_Hgt = helper.GetValueInt("val.Roi_Hgt", 0);

		cur_f1_pBuF=new float[m_nRoi_Wid * m_nRoi_Hgt];
		cur_f2_pBuF=new float[m_nRoi_Wid * m_nRoi_Hgt];

		cur_fX_pBuF[0]=new float[m_nRoi_Wid * m_nRoi_Hgt];
		cur_fX_pBuF[1]=new float[m_nRoi_Wid * m_nRoi_Hgt];
		
		m_f1_ub = helper.GetValueDouble("val.f1_ub", 0);//0.56745f;
		m_f1_lb = helper.GetValueDouble("val.f1_lb", 0);//0.379316f;
		m_f2_ub = helper.GetValueDouble("val.f2_ub", 0);//0.453594f;
		m_f2_lb = helper.GetValueDouble("val.f2_lb", 0);//0.298199f;

	}
	

	//if (LSI_LoadSettingParams((char*)m_sCalParamsXMLpath.c_str()) != LGIT_ERROR) 
	if (LSI_LoadSettingParams((char*)m_sCalParamsXMLpath.c_str()) != 1) 
		AddLog(_T("[TOFLSICAL]LSI_LoadSettingParams OK"));


	{
		/*
		fp_1 = fopen(path_1.c_str(), "rb");
		fp_2 = fopen(path_2.c_str(), "rb");
		if (fp_1 == NULL)
			AddLog("Can't read a file! %s \n", path_1.c_str());
		if (fp_2 == NULL)
			AddLog("Can't read a file! %s \n", path_2.c_str());
		fread((unsigned char *)&m_pTCalBin.table.temp_f1, 1, EEPROM_TEMP_SIZE, fp_1);
		fread((unsigned char *)&m_pTCalBin.table.temp_f2, 1, EEPROM_TEMP_SIZE, fp_2);
		fclose(fp_1);
		fclose(fp_2);
		*/
		//cout << m_pTCalBin.table.temp_f1.coefficient << " " << m_pTCalBin.table.temp_f2.coefficient << endl;
		/*
		FILE *fp_1,*fp_2;
		m_sPath_Tempbin = "D:\\LSI\\IO\\Sample_Output\\Calibration_Table";
		string path_1 = m_sPath_Tempbin + "\\temp_f1.bin";
		string path_2 = m_sPath_Tempbin + "\\temp_f2.bin";
		fp_1 = fopen(path_1.c_str(), "rb");
		fp_2 = fopen(path_2.c_str(), "rb");
		if (fp_1 == NULL)
		AddLog("Can't read a file! %s \n", path_1.c_str());
		if (fp_2 == NULL)
		AddLog("Can't read a file! %s \n", path_2.c_str());
		fread((unsigned char *)&m_pTCalBin.table.temp_f1, 1, EEPROM_TEMP_SIZE, fp_1);
		fread((unsigned char *)&m_pTCalBin.table.temp_f2, 1, EEPROM_TEMP_SIZE, fp_2);
		fclose(fp_1);
		fclose(fp_2);
		*/
		/*
		string path_1 = m_sPath_Tempbin + "\\temp_f1.bin";
		string path_2 = m_sPath_Tempbin + "\\temp_f2.bin";
		TLSIcalBin temp;
		if (LSI_LoadDataTemperature_path((char*)path_1.c_str(),FREQ_F1, (unsigned char *)&temp.table.temp_f1))
			AddLog(_T("[TOFLSICAL]FREQ_F1 LSI_LoadDataTemperature error"));
		if (LSI_LoadDataTemperature_path((char*)path_2.c_str(),FREQ_F2, (unsigned char *)&temp.table.temp_f2))
			AddLog(_T("[TOFLSICAL]FREQ_F2 LSI_LoadDataTemperature error"));
			*/
		int off = offsetof(sLSIcalBin, lens);
		int size = sizeof(m_pTCalBin.table.lens) + 0x0200 - 0x014C - 1;
	}
	SetImageFormat();	
	SetCalImage();

	(*m_pTestDEPSDll).loadCalTableFromFile(m_sCalFilePath);
	LoadXML2CALPARAMS(m_sCalParamsLSIlibXMLpath, m_CalParamsLSIlib);
	

	//m_pSerialComm->Open(m_nCOMportNumLED, m_nCOMportBaudRateLED);
#if 0
	if(m_pSerialComm->Open(m_nCOMportNumLED, m_nCOMportBaudRateLED) != TRUE)
	{
		AddLog(_T("[TOFLSICAL]LED Port %d BaudRate %d Open Failed!"),m_nCOMportNumLED,m_nCOMportBaudRateLED);
	}
	else
	{
		AddLog(_T("[TOFLSICAL]LED Port %d BaudRate %d Open Sucess!"),m_nCOMportNumLED,m_nCOMportBaudRateLED);
	}
#else
	
	if (m_pSerialComm->Start(m_nCOMportNumLED, m_nCOMportBaudRateLED) != TRUE)
	{
		AddLog(_T("[TOFLSICAL]LED Port %d BaudRate %d Open Failed!"), m_nCOMportNumLED, m_nCOMportBaudRateLED);
	}
	else
	{
		AddLog(_T("[TOFLSICAL]LED Port %d BaudRate %d Open Sucess!"), m_nCOMportNumLED, m_nCOMportBaudRateLED);
	}
#endif

	TimeRecord("[OnUpdateSpec]",1);
}

vector<double> CInspectToFLSIWigglingCal::SplitPhaseForLSITOFfor1Frame(Mat& pRaw16, Mat& pBMP, int nWidth, int nHeight)
{
	TimeRecord("[OnTestProc],[Capture],[Inspection],[ConverImageFormat],[SplitPhaseForLSITOFfor1Frame]",0);
	int y_offset = nHeight / 2;
	int x_offset = nWidth / 2;
	vector<double> vMean(4, 0.0);
	int count = 0;
	for(int y = 0; y < nHeight; y+=2)
	{
		short *raw_ptr1 = pRaw16.ptr<short>(y);
		short *raw_ptr2 = pRaw16.ptr<short>(y + 1);
		int y2 = y>>1;
		unsigned char *bmp_ptr1 = pBMP.ptr<unsigned char>(y2);
		unsigned char *bmp_ptr2 = pBMP.ptr<unsigned char>(y2 + y_offset);
		for(int x=0; x<nWidth; x+=2)
		{				
			int x1 = (x >> 1);
			int x2 = x1 + x_offset;
			bmp_ptr1[x1] = (unsigned char)(raw_ptr1[x + 0]>>2);
			bmp_ptr1[x2] = (unsigned char)(raw_ptr1[x + 1]>>2);
			bmp_ptr2[x1] = (unsigned char)(raw_ptr2[x + 0]>>2);
			bmp_ptr2[x2] = (unsigned char)(raw_ptr2[x + 1]>>2);
			vMean[0] += (double)bmp_ptr1[x1];
			vMean[1] += (double)bmp_ptr1[x2];
			vMean[2] += (double)bmp_ptr2[x1];
			vMean[3] += (double)bmp_ptr2[x2];
			count++;
		}
	}
	vMean[0]/=(double)count;
	vMean[1]/=(double)count;
	vMean[2]/=(double)count;
	vMean[3]/=(double)count;
	TimeRecord("[OnTestProc],[Capture],[Inspection],[ConverImageFormat],[SplitPhaseForLSITOFfor1Frame]",1);
	return vMean;
}

int CInspectToFLSIWigglingCal::ConverImageFormat(void)
{
	TimeRecord("[OnTestProc],[Capture],[Inspection],[ConverImageFormat]",0);
	int idx = 0;
	int nSuperFrameWidth = m_ImageObject.GetWidth();
	int nSuperFrameHeight = m_ImageObject.GetHeight();
	int nWidth = nSuperFrameWidth;
	int nHeightEmbedded = nSuperFrameHeight / m_nFrameNumPerSuperFrame;
	int nHeight = nHeightEmbedded - m_nEmbeddedLineCount;
	int nFrameEmbeddedSizeShort = nWidth * nHeightEmbedded;
	int nFrameEmbeddedSizeByte = nFrameEmbeddedSizeShort * 2;
	int nFrameEmbeddedSizeMIPI10 = nFrameEmbeddedSizeShort * 5 / 4;
	int nFrameSizeShort = nWidth * nHeight;
	int nFrameSizeByte = nFrameSizeShort * 2;
	int nEmbeddedSizeShort = nWidth * m_nEmbeddedLineCount;
	int nEmbeddedSizeByte = nEmbeddedSizeShort * 2;
	int nEmbeddedSizeMIPI10 = nEmbeddedSizeShort * 5 / 4;

	AddLog(_T("[TOFLSICAL]ConverFormat"));

	//short* pRaw16 = (short*)m_vFrame2ByteObject[idx]->GetImageBuffer();
	//BYTE* pRawMIPI10 = m_vFrameBuffer[idx];
	//BYTE* pRaw8 = (BYTE* )m_vFrame8BitObject[idx]->GetImageBuffer();

	short* pRaw16 = (short*)m_Frame2ByteObject.GetImageBuffer();
	BYTE* pRawMIPI10 = m_pFrameBuffer;
	BYTE* pRaw8 = (BYTE* )m_Frame8BitObject.GetImageBuffer();


	m_vAll_frame_mean_per_phase.clear();
	for(int i = 0; i < m_nFrameNumPerSuperFrame; i++)
	{
		short *pCurRaw16 = &pRaw16[nFrameEmbeddedSizeShort * i];
		memcpy((void*)m_vRawImageWembedded[i].data, (void*)pCurRaw16, nFrameEmbeddedSizeByte);
		memcpy((void*)m_vRawImageWOembedded[i].data, (void*)&pCurRaw16[nEmbeddedSizeShort], nFrameSizeByte);

		BYTE* pCurRawMipi = &pRawMIPI10[nFrameEmbeddedSizeMIPI10 * i];
		memcpy((void*)m_vRawImageMIPI10[i].data(), (void*)pCurRawMipi, nFrameEmbeddedSizeMIPI10);

		BYTE* pCurRaw8 = &pRaw8[nFrameEmbeddedSizeShort * i];
		memcpy((void*)m_vEmbedded[i].data(), (void*)pCurRaw8, nEmbeddedSizeShort);

		vector<double> vMean = SplitPhaseForLSITOFfor1Frame(m_vRawImageWOembedded[i], m_vPhaseSplitImage[i], nWidth, nHeight);
		m_vAll_frame_mean_per_phase.push_back(vMean);
		AddLog(_T("[TOFLSICAL]frame %d, mean = %.0f,%.0f,%.0f,%.0f"), i, vMean[0], vMean[1], vMean[2], vMean[3]);
	}
	TimeRecord("[OnTestProc],[Capture],[Inspection],[ConverImageFormat]",1);
	return 1;
}
vector<string> CInspectToFLSIWigglingCal::tokenize_getline(const string& data, const char delimiter = ' ') 
{
	vector<string> result;
	string token;
	stringstream ss(data);

	while (getline(ss, token, delimiter)) {
		result.push_back(token);
	}
	return result;
}
string CInspectToFLSIWigglingCal::SaveImageForLSI(string path, int index, int frame_num, string mode, vector<string>& file_name_format, BYTE* pBuffer, int size, bool csv)
{

	TimeRecord("[OnTestProc],[Capture],[Inspection],[SaveImage],[SaveImageForLSI]",0);
	char tempStr[1000];
	int i = frame_num;
	sprintf(tempStr, file_name_format[i].c_str(), index, i, mode.c_str());
	string final_path = path + tempStr;

	int cap_index = 0;
	string base_final_path = final_path;
	vector<string> token_final_path = tokenize_getline(base_final_path,'.');
	string temp_final_path = "";
	for (int t = 0; t < token_final_path.size()-1; t++)
	{
		temp_final_path += token_final_path[t];
	}

	while(IsExistFile(final_path.c_str()))
	{
		char temp_path[1000];
		sprintf(temp_path, "%s_%04d.%s",temp_final_path.c_str(),cap_index,token_final_path[token_final_path.size()-1].c_str());
		final_path = (string)temp_path;
		cap_index++;
	}

	if(!m_bSaveImage) return final_path;

	SaveBufferToFile( final_path.c_str(), const_cast<const BYTE*>(pBuffer), (long)(size) );

	if(csv)
	{
		FILE* fp = fopen((final_path + ".csv").c_str(), "w");
		for (int j = 0; j <size; j++)
		{
			if (j != 0 && (j % 16) == 0) fprintf(fp, "\n");
			fprintf(fp, "0x%02X,", pBuffer);
		}
		fclose(fp);
	}

	TimeRecord("[OnTestProc],[Capture],[Inspection],[SaveImage],[SaveImageForLSI]",1);
	return final_path;
}


string CInspectToFLSIWigglingCal::SaveDepthImageForLSI(string path, int index, string file_name_format)
{
	TimeRecord("[OnTestProc],[Capture],[Inspection],[SaveImage],[SaveDepthImageForLSI]",0);
	sDEPSdata* pBuffer = &(*m_pTestDEPSDll).m_sDEPSdata;
	int idx = 0;
	char tempStr[1000];
	MeImageObject tempImageObject;
	tempImageObject.Create(IMAGE_WIDTH, IMAGE_HEIGHT, 8, 1);
	MeImageObject tempImageObject2;
	tempImageObject2.Create(IMAGE_WIDTH, IMAGE_HEIGHT, 8, 3);

	Mat DepthMM = Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_32FC1, &pBuffer->DepthMM[0]).clone();
	Mat DepthCM = Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_32FC1, &pBuffer->DepthCM[0]).clone();
	Mat Intensity = Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_32FC1, &pBuffer->Intensity[0]).clone();
	Mat Amplitude = Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_32FC1, &pBuffer->Amplitude[0]).clone();
	Mat X = Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_32FC1, &pBuffer->X[0]).clone();
	Mat Y = Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_32FC1, &pBuffer->Y[0]).clone();
	Mat SC = Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_32FC1, &pBuffer->SC[0]).clone();

	int crop_size = 50;
	Mat center_ROI = DepthCM(Rect(IMAGE_WIDTH/2 - crop_size/2, IMAGE_HEIGHT/2 - crop_size/2, crop_size, crop_size));

	//m_pLog->SetCommonLog(m_vSensorID[0], m_strBarCode, m_nParaIndex);
	m_pLog->LogOut( _T("%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f"), m_nErrorCode, mean(center_ROI)[0],
		(m_mMean_per_phase["f1_nshfl"])[0],(m_mMean_per_phase["f1_nshfl"])[1],(m_mMean_per_phase["f1_nshfl"])[2],(m_mMean_per_phase["f1_nshfl"])[3],
		(m_mMean_per_phase["f1_shfl"])[0],(m_mMean_per_phase["f1_shfl"])[1],(m_mMean_per_phase["f1_shfl"])[2],(m_mMean_per_phase["f1_shfl"])[3],
		(m_mMean_per_phase["f2_nshfl"])[0],(m_mMean_per_phase["f2_nshfl"])[1],(m_mMean_per_phase["f2_nshfl"])[2],(m_mMean_per_phase["f2_nshfl"])[3],
		(m_mMean_per_phase["f2_shfl"])[0],(m_mMean_per_phase["f2_shfl"])[1],(m_mMean_per_phase["f2_shfl"])[2],(m_mMean_per_phase["f2_shfl"])[3]);


	float min = (float)m_dNormDepthMin;//cm
	float max = (float)m_dNormDepthMax;//cm
	Mat normal = (DepthCM - min) / (max - min) * 255.0f;
	normal.convertTo(normal, CV_8U);
	Mat normal_color;
	applyColorMap(normal, normal_color, COLORMAP_JET);

	if(m_bEnableDepthImage)
	{
		sprintf(tempStr,"%s%s%02d_DEPS_Znormal.png",path.c_str(), file_name_format.c_str(), index);
		memcpy((void*)tempImageObject.GetImageBuffer(),(void*)normal.data, normal.total());
		tempImageObject.SaveToFile(tempStr);

		sprintf(tempStr,"%s%s%02d_DEPS_Zcolormap.png",path.c_str(), file_name_format.c_str(), index);
		memcpy((void*)tempImageObject2.GetImageBuffer(),(void*)normal_color.data, IMAGE_WIDTH * IMAGE_HEIGHT * 3 );
		tempImageObject2.SaveToFile(tempStr);

		sprintf(tempStr,"%s%s%02d_DEPS_Z.raw",path.c_str(), file_name_format.c_str(), index);
		SaveBufferToFile( tempStr, const_cast<const BYTE*>((BYTE*)&pBuffer->DepthCM[0]), (long)(IMAGE_WIDTH * IMAGE_HEIGHT * sizeof(DEPTH_TYPE)) );
		sprintf(tempStr,"%s%s%02d_DEPS_Intensity.raw",path.c_str(), file_name_format.c_str(), index);
		SaveBufferToFile( tempStr, const_cast<const BYTE*>((BYTE*)&pBuffer->Intensity[0]), (long)(IMAGE_WIDTH * IMAGE_HEIGHT * sizeof(DEPTH_TYPE)) );
		sprintf(tempStr,"%s%s%02d_DEPS_Amplitude.raw",path.c_str(), file_name_format.c_str(), index);
		SaveBufferToFile( tempStr, const_cast<const BYTE*>((BYTE*)&pBuffer->Amplitude[0]), (long)(IMAGE_WIDTH * IMAGE_HEIGHT * sizeof(DEPTH_TYPE)) );
		sprintf(tempStr,"%s%s%02d_DEPS_X.raw",path.c_str(), file_name_format.c_str(), index);
		SaveBufferToFile( tempStr, const_cast<const BYTE*>((BYTE*)&pBuffer->X[0]), (long)(IMAGE_WIDTH * IMAGE_HEIGHT * sizeof(DEPTH_TYPE)) );
		sprintf(tempStr,"%s%s%02d_DEPS_Y.raw",path.c_str(), file_name_format.c_str(), index);
		SaveBufferToFile( tempStr, const_cast<const BYTE*>((BYTE*)&pBuffer->Y[0]), (long)(IMAGE_WIDTH * IMAGE_HEIGHT * sizeof(DEPTH_TYPE)) );
		sprintf(tempStr,"%s%s%02d_DEPS_SC.raw",path.c_str(), file_name_format.c_str(), index);
		SaveBufferToFile( tempStr, const_cast<const BYTE*>((BYTE*)&pBuffer->SC[0]), (long)(IMAGE_WIDTH * IMAGE_HEIGHT * sizeof(DEPTH_TYPE)) );
	}

	BYTE* pBMP = (BYTE*)m_ImageObject.GetImageBuffer();
	int nSuperFrameWidth = m_ImageObject.GetWidth();
	int nSuperFrameHeight = m_ImageObject.GetHeight();
	cvtColor(normal, normal, cv::COLOR_GRAY2BGR);
	normalize(Intensity, Intensity, 0, 255, NORM_MINMAX); 
	Intensity.convertTo(Intensity, CV_8U);
	cvtColor(Intensity, Intensity, cv::COLOR_GRAY2BGR);
	normalize(Amplitude, Amplitude, 0, 255, NORM_MINMAX); 
	Amplitude.convertTo(Amplitude, CV_8U);
	cvtColor(Amplitude, Amplitude, cv::COLOR_GRAY2BGR);


	for(int y = 0; y < IMAGE_HEIGHT; y++)
	{
		auto normal_ptr = normal.ptr<Vec3b>(y);
		auto color_ptr = normal_color.ptr<Vec3b>(y);
		auto Intensity_ptr = Intensity.ptr<Vec3b>(y);
		auto Amplitude_ptr = Amplitude.ptr<Vec3b>(y);

		BYTE* pBMPline = &pBMP[nSuperFrameWidth * 3 * y];
		memcpy((void*)pBMPline, (void*)normal_ptr, IMAGE_WIDTH *3);
		memcpy((void*)&pBMPline[IMAGE_WIDTH *3], (void*)color_ptr, IMAGE_WIDTH *3);

		BYTE* pBMPline2 = &pBMP[nSuperFrameWidth * 3 * (y + IMAGE_HEIGHT)];
		memcpy((void*)pBMPline2, (void*)Intensity_ptr, IMAGE_WIDTH *3);
		memcpy((void*)&pBMPline2[IMAGE_WIDTH *3], (void*)Amplitude_ptr, IMAGE_WIDTH *3);
	}

	if(m_bEnableDepth3Dimage)
	{
		CPointCloudView temp(IMAGE_WIDTH * 2, IMAGE_HEIGHT * 2);
		temp.SetNormalization(m_dNormDepthMin, m_dNormDepthMax);
		temp.SetScale(4, 3.5, -1.5);
		temp.SetDegree(0.0 - 20.0, 90.0, 225.0 - 20.0);
		temp.DrawAxis();
		temp.DrawXYPlane(50.0, 50.0, 0.0, 10);
		Mat x_image;X.convertTo(x_image, CV_64FC1);
		Mat y_image;Y.convertTo(y_image, CV_64FC1);
		Mat z_image;DepthCM.convertTo(z_image, CV_64FC1);
		temp.PlotPoints(x_image, y_image, z_image);

		for(int y = 0; y < temp.image.rows; y++)
		{
			auto ptr = temp.image.ptr<Vec3b>(y);
			BYTE* pBMPline = &pBMP[nSuperFrameWidth * 3 * (y + temp.image.rows)];
			memcpy((void*)pBMPline, (void*)ptr, nSuperFrameWidth *3);
		}

		sprintf(tempStr,"%s%s%02d_DPES_3Dplot.png",path.c_str(), file_name_format.c_str(), index);
		resize(temp.image, temp.image, Size(IMAGE_WIDTH, IMAGE_HEIGHT));
		memcpy((void*)tempImageObject2.GetImageBuffer(),(void*)temp.image.data, IMAGE_WIDTH * IMAGE_HEIGHT * 3 );
		tempImageObject2.SaveToFile(tempStr);

	}

	if(m_bEnableDepthCSV)
	{
		sprintf(tempStr,"%s%s%02d_DEPS_Z.csv",path.c_str(), file_name_format.c_str(), index);
		FILE* fp = fopen(tempStr, "w");
		for (int j = 0; j < IMAGE_HEIGHT; j++)
		{
			for (int i = 0; i < IMAGE_WIDTH; i++)
			{
				fprintf(fp, "%.2f,", pBuffer->DepthCM[IMAGE_WIDTH * j + i]);
			}
			fprintf(fp, "\n");
		}
		fclose(fp);
	}
	TimeRecord("[OnTestProc],[Capture],[Inspection],[SaveImage],[SaveDepthImageForLSI]",1);
	return tempStr;
}

BOOL CInspectToFLSIWigglingCal::FindCurrentPath(string root,string sensorID, SYSTEMTIME &time, string& path)
{
	vector<string> vec_dir;
	/*
	struct _finddata_t fd;
	intptr_t handle;
	if ((handle = _findfirst(root.c_str(), &fd)) == -1L)
		AddLog(_T("[SetImagePath] No file in directory!"));

	do
	{
		string dir(fd.name);
		if (dir.find(sensorID) != std::string::npos)
		{
			vec_dir.push_back(dir);
		}

	} while (_findnext(handle, &fd) == 0);
	_findclose(handle);
	*/
	
	for (const auto & entry : fs::directory_iterator(path))
	{
		string dir(entry.path().string());
		if (dir.find(sensorID) != std::string::npos)
		{
			vec_dir.push_back(dir);
		}
	}
		

	int n_dir = vec_dir.size();
	if (vec_dir.empty())
	{
		AddLog(_T("[SetImagePath] No Same SensorID Directory"));
		return FALSE;
	}
	string dir_time, dir_date, dir_year, dir_month, dir_day, dir_hour, dir_minute, dir_second;
	int min_diff = 10000000000;
	for (int i = 0; i < vec_dir.size(); i++)
	{
		vector<string> token_dir = tokenize_getline(vec_dir[i], '_');
		dir_time = token_dir[token_dir.size() - 1];
		dir_date = token_dir[token_dir.size() - 2];
		if (dir_time.size() != 6 || dir_date.size() != 8) 
		{
			AddLog(_T("[SetImagePath] time:%s date:%s, Time Formate is not correct"), dir_time.c_str(), dir_date.c_str());
			return FALSE; 
		}
		dir_year = dir_date.substr(0,4);
		dir_month = dir_date.substr(4, 2);
		dir_day = dir_date.substr(6, 2);
		dir_hour = dir_time.substr(0, 2);
		dir_minute = dir_time.substr(2, 2);
		dir_second = dir_time.substr(4, 2);

		int diff =
			60 * 60 * 24 * 30 * 12 * ((int)time.wYear - stoi(dir_year)) +
			60 * 60 * 24 * 30 * ((int)time.wMonth - stoi(dir_month)) +
			60 * 60 * 24 * ((int)time.wDay - stoi(dir_day)) +
			60 * 60 * ((int)time.wHour - stoi(dir_hour)) +
			60 * ((int)time.wMinute - stoi(dir_minute)) +
			((int)time.wSecond - stoi(dir_second));
		if (diff > 0 && min_diff > diff)
		{
			path = vec_dir[i];
			min_diff = diff;
		}
	}
	return TRUE;
}

int CInspectToFLSIWigglingCal::SetImagePath(void)
{
	TimeRecord("[OnTestProc],[SetImagePath]",0);
	char temp_time[255];
	SYSTEMTIME time;
	GetLocalTime(&time);
	string path = (string)m_sImagePath;
	mkdir(path.c_str());
	string sensorId = m_Sensor.GetSensorID();
	if (m_bSaveImage == true)
	//if (m_sInspectSite == "A")
	{
		sprintf(temp_time, "_%02d%02d%02d_%02d%02d%02d", time.wYear, time.wMonth, time.wDay, time.wHour, time.wMinute, time.wSecond);
		
		path = (string)m_sImagePath + "\\" + sensorId + (string)(temp_time);
		mkdir(path.c_str());
	}
	else
	//else if (m_sInspectSite == "B"|| m_sInspectSite == "C")
	{
		FindCurrentPath(path, sensorId, time, path);
	}
	//path = "C:\\log\\test1\\8C86DF3B3458_20201027_105110";
	m_sBasePath_wiggling = (string)path + "\\" + "wiggling";
	mkdir(m_sBasePath_wiggling.c_str());

	m_sBasePath_LEDon = (string)path + "\\" + "LedOn";
	mkdir(m_sBasePath_LEDon.c_str());

	m_sBasePath_LEDoff = (string)path + "\\" + "LedOff";
	mkdir(m_sBasePath_LEDoff.c_str());

	m_sPath_BinRoot = (string)path + "\\" + "output";
	mkdir(m_sPath_BinRoot.c_str());

	m_sPath_Wiggbin = (string)m_sPath_BinRoot + "\\" + "wiggling";
	mkdir(m_sPath_Wiggbin.c_str());

	m_sPath_Lensfppnbin = (string)m_sPath_BinRoot + "\\" + "lens_ffpn";
	mkdir(m_sPath_Lensfppnbin.c_str());

	m_sPath_Totalbin = (string)m_sPath_BinRoot + "\\" + "total";
	mkdir(m_sPath_Totalbin.c_str());

	m_svImageNamePath_wiggling.clear();
	for (int i = 0; i < MAX_PHASE_SHIFT_NUM; i++)
	{
		m_svImageNamePath_wiggling.push_back(m_sBasePath_wiggling + "\\" + phase_shift_distance_mm[i]);
	}
	m_sImageNamePath_LEDon = m_sBasePath_LEDon + "\\lens";
	m_sImageNamePath_LEDoff = m_sBasePath_LEDoff + "\\fppn";

	
	TimeRecord("[OnTestProc],[SetImagePath]",1);
	return 1;
}

int CInspectToFLSIWigglingCal::GetEffectiveImageIndex(void)
{
	//TimeRecord("[OnTestProc],[Capture],[Inspection],[GetEffectiveImageIndex]",0);
	int index = m_nCurrentSuperFrameIndex;
	AddLog(_T("[TOFLSICAL]SaveImage"));

	int embd_byte_index_frame_count = 16;
	int embd_byte_index_phase_map = 182;
	int embd_byte_index_frequency = 184;

	m_vsMode.clear();
	m_effective_image_index_table.clear();
	m_effective_image_index_table.clear();
	int flag = -1;
	for(int i = 0; i < m_nFrameNumPerSuperFrame; i++)
	{
		auto& Embd = m_vEmbedded[i];
		int frame_count = Embd[embd_byte_index_frame_count] - (m_vEmbedded[0])[embd_byte_index_frame_count];
		string phase_map = Embd[embd_byte_index_phase_map] == 0x10 ? "nshfl" : "shfl";
		string frequency = Embd[embd_byte_index_frequency] == 0x00 ? "f1" : "f2";
		m_vsMode.push_back(frequency + "_" + phase_map);

		if(flag == -1)
		{
			if(m_fn_map[m_vsMode[i]] == 0) 
			{
				m_effective_image_index_table.push_back(m_fn_map[m_vsMode[i]]);
				flag = 0;
			}
			else
			{
				m_effective_image_index_table.push_back(-1);
			}
		}
		else if(flag == 0)
		{
			m_effective_image_index_table.push_back(m_fn_map[m_vsMode[i]]);				
			if(m_fn_map[m_vsMode[i]] == 3) 
			{
				flag = 1;				
			}
		}
		else if(flag == 1)
		{
			m_effective_image_index_table.push_back(-1);
		}
		AddLog(_T("[TOFLSICAL]idx%2d,cnt%2d(%2d),%s,%s"), index, Embd[embd_byte_index_frame_count], m_effective_image_index_table[i], frequency.c_str(), phase_map.c_str());
	}
	//TimeRecord("[OnTestProc],[Capture],[Inspection],[GetEffectiveImageIndex]",1);
	return 1;
}


int CInspectToFLSIWigglingCal::SaveImage(void)
{	
	TimeRecord("[OnTestProc],[Capture],[Inspection],[SaveImage]",0);
	string path;
	switch(m_eCalMode)
	{
	case eCALMODE::WIGGLING:
		path = m_svImageNamePath_wiggling[m_nCurrentPhaseShiftIndex];
		break;
	case eCALMODE::LEDOFF:
		path = m_sImageNamePath_LEDoff;
		break;

	case eCALMODE::LEDON:
		path = m_sImageNamePath_LEDon;
		break;
	default:
		break;
	}

	Mat temp;
	vector<vector<RAW_TYPE*>> vp_RawData(1);
	vector<vector<EMB_TYPE*>> vp_EmbData(1);

	int index = m_nCurrentSuperFrameIndex;
	string image_full_path;
	for(int i = 0; i < m_nFrameNumPerSuperFrame; i++)
	{
		if(m_effective_image_index_table[i] < 0) continue;
		m_mMean_per_phase[m_vsMode[i]] = m_vAll_frame_mean_per_phase[i];
		//raw image		
		if(m_bEnableMIPIimage) 
			SaveImageForLSI(path, index, m_fn_map[m_vsMode[i]], m_vsMode[i], m_vsMIPI10Format, m_vRawImageMIPI10[i].data(), (int)m_vRawImageMIPI10[i].size(), false);
		if(m_bEnableRawWembdImage) 
			SaveImageForLSI(path, index, m_fn_map[m_vsMode[i]], m_vsMode[i], m_vsRaw16wEmbdFormat, m_vRawImageWembedded[i].data, (int)(m_vRawImageWembedded[i].total() * 2), true);
		if(m_bEnableEmbeddedImage)
			SaveImageForLSI(path, index, m_effective_image_index_table[i], m_vsMode[i], m_vsEmbeddedFormat, m_vEmbedded[i].data(), (int)m_vEmbedded[i].size(), false);
		if(m_bEnableRawWoEmbdImage) 
			image_full_path = SaveImageForLSI(path, index, m_effective_image_index_table[i], m_vsMode[i], m_vsRaw16Format, m_vRawImageWOembedded[i].data, (int)(m_vRawImageWOembedded[i].total() * 2), false);

		if(m_bEnableDepthView)
		{
			vp_RawData[0].push_back((RAW_TYPE*)m_vRawImageWOembedded[i].data);
			vp_EmbData[0].push_back((EMB_TYPE*)m_vEmbedded[i].data());
		}

		if(m_bEnablePNGimage)
		{
			temp = m_vRawImageWOembedded[i] / 4;
			temp.convertTo(temp,CV_8UC1);
			memcpy((void*)m_tempImageObject.GetImageBuffer(),(void*)temp.data, temp.total());
			m_tempImageObject.SaveToFile((image_full_path  + ".png").c_str());
			memcpy((void*)m_tempImageObject.GetImageBuffer(),(void*)m_vPhaseSplitImage[i].data, m_vPhaseSplitImage[i].total());
			m_tempImageObject.SaveToFile((image_full_path + "_split.png").c_str());
		}

	}

	if(m_bEnableDepthView)
	{
	//	(*m_pTestDEPSDll).SetLog(0x41, m_pLogUserData, m_pfnLogOut);
		//(*m_pTestDEPSDll).SetLog(m_nParaIndex, m_pLogUserData, m_pfnLogOut);
		(*m_pTestDEPSDll).run_all(vp_RawData, vp_EmbData);
		SaveDepthImageForLSI(path, index, "_");
	}
	TimeRecord("[OnTestProc],[Capture],[Inspection],[SaveImage]",1);
	return 1;
}


vector<pair<unsigned char, int>> CInspectToFLSIWigglingCal::MakeSerialSequence(string PD1_2nd, string PD0_2nd, string PD1_1st, string PD0_1st)
{
	TimeRecord("[OnTestProc],[Capture],[SetPhaseShift],[MakeSerialSequence]",0);
	vector<pair<unsigned char, int>> temp;
	unsigned char en_bit = SETBIT(NB6L295_0_EN) + SETBIT(NB6L295_1_EN);
	unsigned char clk_bit = SETBIT(NB6L295_0_SCLK) + SETBIT(NB6L295_1_SCLK);
	unsigned char load_bit = SETBIT(NB6L295_0_SLOAD) + SETBIT(NB6L295_1_SLOAD);
	int delay_EN01 = 0;
	int delay_EN11 = 0;
	int delay_CLK01 = 0;
	int delay_CLK11 = 0;
	int delay_CLK10 = 0;
	int delay_LOAD01 = 0;
	int delay_LOAD10 = 0;
	int delay_EN10 = 0;
	//for(int i = 0; i < MAX_PHASE_SHIFT_NUM; i++)
	{
		temp.push_back(make_pair(0, delay_EN01));
		temp.push_back(make_pair(en_bit, delay_EN11));
		for(int j = 0; j < MAX_NB6L295_SERIAL_BIT_NUM; j++)
		{
			unsigned char PD0_2nd_bit = PD0_2nd.data()[MAX_NB6L295_SERIAL_BIT_NUM - j - 1] == '1' ? SETBIT(NB6L295_1_SDIN):0;
			unsigned char PD0_1st_bit = PD0_1st.data()[MAX_NB6L295_SERIAL_BIT_NUM - j - 1] == '1' ? SETBIT(NB6L295_0_SDIN):0;

			temp.push_back(make_pair(PD0_2nd_bit + PD0_1st_bit + en_bit, delay_CLK01));
			temp.push_back(make_pair(PD0_2nd_bit + PD0_1st_bit + en_bit + clk_bit, delay_CLK11));
			temp.push_back(make_pair(PD0_2nd_bit + PD0_1st_bit + en_bit, delay_CLK10));
		}
		temp.push_back(make_pair(en_bit + load_bit, delay_LOAD01));
		temp.push_back(make_pair(en_bit, delay_LOAD10));
		temp.push_back(make_pair(0, delay_EN10));

		temp.push_back(make_pair(0, delay_EN01));
		temp.push_back(make_pair(en_bit, delay_EN11));
		for(int j = 0; j < MAX_NB6L295_SERIAL_BIT_NUM; j++)
		{
			unsigned char PD1_2nd_bit = PD1_2nd.data()[MAX_NB6L295_SERIAL_BIT_NUM - j - 1] == '1' ? SETBIT(NB6L295_1_SDIN):0;
			unsigned char PD1_1st_bit = PD1_1st.data()[MAX_NB6L295_SERIAL_BIT_NUM - j - 1] == '1' ? SETBIT(NB6L295_0_SDIN):0;
			temp.push_back(make_pair(en_bit,0));
			temp.push_back(make_pair(PD1_2nd_bit + PD1_1st_bit + en_bit, delay_CLK01));
			temp.push_back(make_pair(PD1_2nd_bit + PD1_1st_bit + en_bit + clk_bit, delay_CLK11));
			temp.push_back(make_pair(PD1_2nd_bit + PD1_1st_bit + en_bit, delay_CLK10));
		}
		temp.push_back(make_pair(en_bit + load_bit, delay_LOAD01));
		temp.push_back(make_pair(en_bit, delay_LOAD10));
		temp.push_back(make_pair(0, delay_EN10));
	}
	TimeRecord("[OnTestProc],[Capture],[SetPhaseShift],[MakeSerialSequence]",1);
	return temp;
}

BOOL CInspectToFLSIWigglingCal::SetPhaseShift(int index)
{
	bool ret;
	ret = m_Grabber.WriteI2C(BIT8_BIT8, SLAVE_ADDR_TCA6416A, Configuration0, 0x00);//set output port0
	if(!ret) {AddLog(_T("[TOFLSICAL]WriteI2C fail Configuration0, 0x00"));return false;}
	ret = m_Grabber.WriteI2C(BIT8_BIT8, SLAVE_ADDR_TCA6416A, Configuration1, 0x00);//set output port1
	if(!ret) {AddLog(_T("[TOFLSICAL]WriteI2C fail Configuration1, 0x00"));return false;}
	ret = m_Grabber.WriteI2C(BIT8_BIT8, SLAVE_ADDR_TCA6416A, RELAY_CONTROL_REG, SETBIT(RELAY0_EN));//delay IC bypass

	if(!ret) {AddLog(_T("[TOFLSICAL]WriteI2C fail RELAY_CONTROL_REG, SETBIT(RELAY0_EN)"));	return false;}
	int i = index;
	vector<pair<unsigned char, int>> v_sequence = MakeSerialSequence(phase_shift_sequence[i][0], phase_shift_sequence[i][1], phase_shift_sequence[i][2], phase_shift_sequence[i][3]);
	for(auto&it:v_sequence)
	{
		ret = m_Grabber.WriteI2C(BIT8_BIT8, SLAVE_ADDR_TCA6416A, NB6L295_CONTROL_REG, (WORD)it.first);
		if(!ret) {AddLog(_T("[TOFLSICAL]WriteI2C fail NB6L295_CONTROL_REG"));	return false;}
		//else
		//	AddLog(_T("[TOFLSICAL]%02X"),it.first);
		Wait(it.second);		
	}

	ret = m_Grabber.WriteI2C(BIT8_BIT8, SLAVE_ADDR_TCA6416A, RELAY_CONTROL_REG, SETBIT(RELAY1_EN));//use delay ID
	if(!ret) {AddLog(_T("[TOFLSICAL]WriteI2C fail RELAY_CONTROL_REG, SETBIT(RELAY1_EN)"));	return false;}

	return TRUE;
}


BOOL CInspectToFLSIWigglingCal::DisablePhaseShift(void)
{

	bool ret;
	ret = m_Grabber.WriteI2C(BIT8_BIT8, SLAVE_ADDR_TCA6416A, Configuration0, 0x00);//set output port0
	if(!ret) {AddLog(_T("[TOFLSICAL]WriteI2C fail Configuration0, 0x00"));	return false;}
	ret = m_Grabber.WriteI2C(BIT8_BIT8, SLAVE_ADDR_TCA6416A, Configuration1, 0x00);//set output port1
	if(!ret) {AddLog(_T("[TOFLSICAL]WriteI2C fail Configuration1, 0x00"));	return false;}
	ret = m_Grabber.WriteI2C(BIT8_BIT8, SLAVE_ADDR_TCA6416A, RELAY_CONTROL_REG, SETBIT(RELAY0_EN));//delay IC bypass

	return TRUE;
}

bool CInspectToFLSIWigglingCal::PassFailWigglingImg()
{
	int index = m_nCurrentPhaseShiftIndex;
	int nSuperFrameWidth = m_ImageObject.GetWidth();
	int nSuperFrameHeight = m_ImageObject.GetHeight();
	int nWidth = nSuperFrameWidth;
	int nHeightEmbedded = nSuperFrameHeight / m_nFrameNumPerSuperFrame;
	int nHeight = nHeightEmbedded - m_nEmbeddedLineCount;
	int raw_image_size = nWidth * nHeight * 2;
	bool f1_rst = true;
	bool f2_rst = true;

	switch(m_eCalMode)
	{
		case eCALMODE::WIGGLING:
		{
			int offset = m_nCurrentPhaseShiftIndex*raw_image_size;
		
			short *f1_nshfl = (short *)&m_sCalImageSet.wiggling_nshfl_f1.data[offset]; // f1_nshfl
			short *f1_shfl = (short *)&m_sCalImageSet.wiggling_shfl_f1.data[offset]; // f1_shfl
			short *f2_nshfl = (short *)&m_sCalImageSet.wiggling_nshfl_f2.data[offset]; // f2_nshfl
			short *f2_shfl = (short *)&m_sCalImageSet.wiggling_shfl_f2.data[offset]; // f2_shfl
			Convert_phaseToAngle_wigg(f1_nshfl,f1_shfl,cur_fX_pBuF[0],nWidth,nHeight,10,10);
			Convert_phaseToAngle_wigg(f2_nshfl,f2_shfl,cur_fX_pBuF[1],nWidth,nHeight,10,10);
			Convert_average_wigg(cur_fX_pBuF[0],&cur_fX_phase_avg[0][index],m_nRoi_Wid,m_nRoi_Hgt);
			Convert_average_wigg(cur_fX_pBuF[1],&cur_fX_phase_avg[1][index],m_nRoi_Wid,m_nRoi_Hgt);
			AddLog(_T("freq1 ang 100 : %f, avg : %f "),cur_fX_pBuF[0][99],cur_fX_phase_avg[0][index]);
			AddLog(_T("freq2 ang 100 : %f, avg : %f "),cur_fX_pBuF[1][99],cur_fX_phase_avg[1][index]);
			if(index > 0)
			{
				f1_rst = Wiggle_Validation_using_Reference(m_nFreqF1,cur_fX_phase_avg[0][index],cur_fX_phase_avg[0][index-1]);
				f2_rst = Wiggle_Validation_using_Reference(m_nFreqF2,cur_fX_phase_avg[1][index],cur_fX_phase_avg[1][index-1]);
			}
		}
	}
	if(f1_rst == false)
	{
		cur_fX_phase_avg[0][index] = -1;
	}
	if(f2_rst == false)
	{
		cur_fX_phase_avg[1][index] = -1;
	}
	return ( f1_rst && f2_rst);
}
void CInspectToFLSIWigglingCal::Convert_average_wigg(float* input, float* avg, int wid,int hgt)
{
	int curPos = 0;
	float min_input, max_input;
	float pre_min, pre_max;
	float* temp = new float[hgt * wid];

	int cntpixel = 0;
	double sum = 0.;
	double sum_temp = 0.;

	for (int i = 0; i < hgt; i++)
	{
		for (int j = 0; j < wid; j++)
		{
			curPos = i*wid + j;

			sum += input[curPos];
			
			temp[cntpixel] = input[curPos];

			if (cntpixel == 0)
			{
				min_input = temp[0];
				max_input = temp[0];
			}
			else
			{
				min_input = min_input < temp[cntpixel] ? min_input : temp[cntpixel];
				//pre_min = min_input;

				max_input = max_input > temp[cntpixel] ? max_input : temp[cntpixel];
				//pre_max = max_input;
			}

			cntpixel++;
		}
	}
	//min_input = -M_PI;
	if (max_input - min_input > M_PI)
	{
		for (int i = 0; i < m_nRoi_Hgt; i++)
		{
			for (int j = 0; j < m_nRoi_Wid; j++)
			{
				if (temp[i * m_nRoi_Wid + j] < M_PI)
					temp[i * m_nRoi_Wid + j] += 2 * M_PI;

				sum_temp += temp[i * m_nRoi_Wid + j];
			}
		}
		*avg = sum_temp / cntpixel;
	}
	else
	{
		*avg = sum / cntpixel;
	}
	delete[] temp;
}

bool CInspectToFLSIWigglingCal::Wiggle_Validation_using_Reference( int freq,float cur_wig,float pre_wig ) {
	
	bool result = true;
	float ub = 0.0f;
	float lb = 0.0f;
	float pi_b = 5.9f;

	if (freq == m_nFreqF1) {
		ub = m_f1_ub * 1.2f;
		lb = m_f1_lb * 0.8f;
	}
	else if(freq == m_nFreqF2) {
		ub = m_f2_ub * 1.2f;
		lb = m_f2_lb * 0.8f;
	}

	float diff_wig = cur_wig - pre_wig;
	if (diff_wig < (-1.0 * M_PI) ) {
		diff_wig += (2.0 * M_PI);
	}
	
	AddLog(_T("Freq : %d, cur_wig : %.4f, pre_wig : %.4f, diff_wig : %.4f"), freq, cur_wig, pre_wig, diff_wig);

	result = !((diff_wig > ub) || (diff_wig < lb));
	

	return result;
}


void CInspectToFLSIWigglingCal::Convert_phaseToAngle_wigg(short* nshfl, short* shfl, float* phase, int wid, int hgt,int roi_wid,int roi_hgt)
{
	int nsA0, nsA1, nsA2, nsA3;
	int sA0, sA1, sA2, sA3;
	short A0, A1, A2, A3;
	int size = hgt * wid;
	int a,b;
	int start_hgt = (hgt/2-roi_hgt/2);
	int start_wid =(wid/2-roi_wid/2);
	int start_pos = (hgt/2-roi_hgt/2)*wid + (wid/2-roi_wid/2);
	double angle;

	for (int i = 0; i < roi_hgt*2; i+=2)
	{
		for (int j = 0; j < roi_wid*2; j+=2)
		{
			nsA0 = start_pos + i* wid  + j;
			nsA1 = start_pos + (i+1) * wid  + j ;
			nsA2 = start_pos + (i+1) * wid + j + 1 ;
			nsA3 = start_pos + i * wid  + j + 1;

			sA2 = start_pos + i * wid  + j;
			sA3 = start_pos + (i+1) * wid  + j ;
			sA0 = start_pos + (i+1) * wid + j + 1 ;
			sA1 = start_pos + i * wid  + j + 1;

			A0 = (nshfl[nsA0] + shfl[sA0]) / 2;
			A1 = (nshfl[nsA1] + shfl[sA1]) / 2;
			A2 = (nshfl[nsA2] + shfl[sA2]) / 2;
			A3 = (nshfl[nsA3] + shfl[sA3]) / 2;

			short x = (A1 - A3) + (A0 - A2);
			short y = (A1 - A3) - (A0 - A2);
			angle = atan2(y, x);
		
			if (angle < 0)
				phase[i/2 * roi_wid + j/2] = (float)(angle)+(M_PI / 4) + (M_PI * 2);
			else
				phase[i/2 * roi_wid + j/2] = (float)(angle)+(M_PI / 4);

			if (phase[i/2 * roi_wid + j/2] >= 2 * M_PI)
				phase[i/2 * roi_wid + j/2] -= 2 * M_PI;
				
		}
	}
	return;
}


int CInspectToFLSIWigglingCal::CollectImage(void)
{	
	TimeRecord("[OnTestProc],[Capture],[Inspection],[CollectImage]",0);
	map<string, short*> pBuF_map;
	for(auto it : m_fn_map)
		pBuF_map[it.first] = nullptr;

	int nSuperFrameWidth = m_ImageObject.GetWidth();
	int nSuperFrameHeight = m_ImageObject.GetHeight();
	int nWidth = nSuperFrameWidth;
	int nHeightEmbedded = nSuperFrameHeight / m_nFrameNumPerSuperFrame;
	int nHeight = nHeightEmbedded - m_nEmbeddedLineCount;
	int raw_image_size = nWidth * nHeight * 2;

	switch(m_eCalMode)
	{
	case eCALMODE::WIGGLING:
		{
			int offset = m_nCurrentPhaseShiftIndex * raw_image_size;
			pBuF_map["f1_nshfl"] = (short *)&m_sCalImageSet.wiggling_nshfl_f1.data[offset];
			pBuF_map["f1_shfl"] = (short *)&m_sCalImageSet.wiggling_shfl_f1.data[offset];
			pBuF_map["f2_nshfl"] = (short *)&m_sCalImageSet.wiggling_nshfl_f2.data[offset];
			pBuF_map["f2_shfl"] = (short *)&m_sCalImageSet.wiggling_shfl_f2.data[offset];
		}
		break;
	case eCALMODE::LEDOFF:
		pBuF_map["f1_nshfl"] = (short *)m_sCalImageSet.LEDoff_nshfl_f1.data;
		pBuF_map["f1_shfl"] = (short *)m_sCalImageSet.LEDoff_shfl_f1.data;
		pBuF_map["f2_nshfl"] = (short *)m_sCalImageSet.LEDoff_nshfl_f2.data;
		pBuF_map["f2_shfl"] = (short *)m_sCalImageSet.LEDoff_shfl_f2.data;
		break;
	case eCALMODE::LEDON:
		pBuF_map["f1_nshfl"] = (short *)m_sCalImageSet.LEDon_nshfl_f1.data;
		pBuF_map["f1_shfl"] = (short *)m_sCalImageSet.LEDon_shfl_f1.data;
		pBuF_map["f2_nshfl"] = (short *)m_sCalImageSet.LEDon_nshfl_f2.data;
		pBuF_map["f2_shfl"] =(short *)m_sCalImageSet.LEDon_shfl_f2.data;
		break;
	default:
		break;
	}

	int index = m_nCurrentSuperFrameIndex;
	string image_full_path;
	for(int i = 0; i < m_nFrameNumPerSuperFrame; i++)
	{
		if(m_effective_image_index_table[i] < 0) continue;		
		string mode = m_vsMode[i];
		auto size = m_vRawImageWOembedded[i].total() * m_vRawImageWOembedded[i].elemSize();
		memcpy(pBuF_map[mode], m_vRawImageWOembedded[i].data, size);
		if(m_eCalMode == eCALMODE::LEDOFF)
		{
			if(mode == "f1_nshfl") 
				memcpy(m_sCalImageSet.LEDoff_nshfl_f1_embd.data(), m_vEmbedded[i].data(), m_nEmbeddedDataFileSize);
			else if(mode == "f2_nshfl") 
				memcpy(m_sCalImageSet.LEDoff_nshfl_f2_embd.data(), m_vEmbedded[i].data(), m_nEmbeddedDataFileSize);
		}
	}
	TimeRecord("[OnTestProc],[Capture],[Inspection],[CollectImage]",1);
	return 1;
}

BOOL CInspectToFLSIWigglingCal::VerifyCalData()
{
	BYTE temp_f1_checksum = 0;
	BYTE temp_f2_checksum = 0;
	BYTE lens_checksum = 0;
	BYTE wigl_f1_checksum= 0;
	BYTE wigl_f2_checksum= 0;

	BYTE uint16_wigl_f1_checksum = 0;
	BYTE uint16_wigl_f2_checksum = 0;

	BYTE fppn_f1_checksum= 0;
	BYTE fppn_f2_checksum= 0;

	for (int i = 0x30; i < 0x58; i++) temp_f1_checksum += m_pTCalBinFinal.raw[i];
	for (int i = 0xA0; i < 0xC8; i++) temp_f2_checksum += m_pTCalBinFinal.raw[i];
	for (int i = 0x0110; i < 0x014C; i++) lens_checksum += m_pTCalBinFinal.raw[i];
	for (int i = 0x0210; i < 0x1210; i++) wigl_f1_checksum += m_pTCalBinFinal.raw[i];
	for (int i = 0x1290; i < 0x2290; i++) wigl_f2_checksum += m_pTCalBinFinal.raw[i];
	for (int i = 0x2310; i < 0x23E8; i++) fppn_f1_checksum += m_pTCalBinFinal.raw[i];
	for (int i = 0x2410; i < 0x24E8; i++) fppn_f2_checksum += m_pTCalBinFinal.raw[i];
	temp_f1_checksum = temp_f1_checksum % 0xFF + 1;
	temp_f2_checksum = temp_f2_checksum % 0xFF + 1;
	lens_checksum = lens_checksum % 0xFF + 1;
	wigl_f1_checksum = wigl_f1_checksum % 0xFF + 1;
	wigl_f2_checksum = wigl_f2_checksum % 0xFF + 1;
	fppn_f1_checksum = fppn_f1_checksum % 0xFF + 1;
	fppn_f2_checksum = fppn_f2_checksum % 0xFF + 1;
	/*
	for (int i = 0; i < 2048; i++)
	{
		uint16_wigl_f1_checksum += m_pTCalBinFinal.table.wigl_f1.LUT[i];
		uint16_wigl_f2_checksum += m_pTCalBinFinal.table.wigl_f2.LUT[i];
	}
	uint16_wigl_f1_checksum = uint16_wigl_f1_checksum % 0xFF + 1;
	uint16_wigl_f2_checksum = uint16_wigl_f2_checksum % 0xFF + 1;



Temperature_F1_Bin,Temperature_F1_Cal,Temperature_F1_Result,
Temperature_F2_Bin,Temperature_F2_Cal,Temperature_F2_Result,
Lens_Calibration_Bin,Lens_Calibration_Cal,Lens_Calibration_Result,
FPPN_F1_Bin,FPPN_F1_Cal,FPPN_F1_Result,
FPPN_F2_Bin,FPPN_F2_Cal,FPPN_F2_Result,

Wiggling_F1_Bin,
Wiggling_F1_BYTE_Cal,Wiggling_F1_BYTE_Result,
Wiggling_F1_TYPE_Cal,Wiggling_F1_TYPE_Result,

Wiggling_F2_Bin,
Wiggling_F2_BYTE_Cal,Wiggling_F2_BYTE_Result,
Wiggling_F2_TYPE_Cal,Wiggling_F2_TYPE_Result"

	*/


	AddLog(_T("[TOFLSICAL-CheckSum] : Temperature_F1	- Calculated : 0x%2x , Record : 0x%2x"),temp_f1_checksum,m_pTCalBinFinal.table.temp_f1.coefficient_cheksum);
	AddLog(_T("[TOFLSICAL-CheckSum] : Temperature_F2	- Calculated : 0x%2x , Record : 0x%2x"),temp_f2_checksum,m_pTCalBinFinal.table.temp_f2.coefficient_cheksum);
	AddLog(_T("[TOFLSICAL-CheckSum] : Lens Calibration	- Calculated : 0x%2x , Record : 0x%2x"),lens_checksum,m_pTCalBinFinal.table.lens.lens_cheksum);
	AddLog(_T("[TOFLSICAL-CheckSum] : FPPN_F1			- Calculated : 0x%2x , Record : 0x%2x"),fppn_f1_checksum,m_pTCalBinFinal.table.ffpn_f1.coefficient_cheksum);
	AddLog(_T("[TOFLSICAL-CheckSum] : FPPN_F2			- Calculated : 0x%2x , Record : 0x%2x"),fppn_f2_checksum,m_pTCalBinFinal.table.ffpn_f2.coefficient_cheksum);
	AddLog(_T("[TOFLSICAL-CheckSum] : Wiggling_F1		- BYTE Calculated : 0x%2x , Record : 0x%2x"), wigl_f1_checksum, m_pTCalBinFinal.table.wigl_f1.LUT_cheksum);
	AddLog(_T("[TOFLSICAL-CheckSum] : Wiggling_F2		- BYTE Calculated : 0x%2x , Record : 0x%2x"), wigl_f2_checksum, m_pTCalBinFinal.table.wigl_f2.LUT_cheksum);

	
	BOOL ret = TRUE;
	if(temp_f1_checksum !=  m_pTCalBinFinal.table.temp_f1.coefficient_cheksum ||
		temp_f2_checksum !=  m_pTCalBinFinal.table.temp_f2.coefficient_cheksum ||
		lens_checksum !=  m_pTCalBinFinal.table.lens.lens_cheksum||
		wigl_f1_checksum !=  m_pTCalBinFinal.table.wigl_f1.LUT_cheksum ||
		wigl_f2_checksum !=  m_pTCalBinFinal.table.wigl_f2.LUT_cheksum ||
		fppn_f1_checksum !=  m_pTCalBinFinal.table.ffpn_f1.coefficient_cheksum||
		fppn_f2_checksum !=  m_pTCalBinFinal.table.ffpn_f2.coefficient_cheksum )
	{
		ret = FALSE;
	}
	

#if 0 
	if(reflag){
		BYTE bin = m_pTCalBinFinal.table.temp_f1.coefficient_cheksum;
		BYTE cal = temp_f1_checksum;
		string temp = "";
		sprintf((char*)temp.c_str(),"0x%2x,", bin);
		checksum_check << temp.c_str();
		temp.clear();
		sprintf((char*)temp.c_str(), "0x%2x,", cal);
		checksum_check << temp.c_str();
		if (bin == cal)  checksum_check << "TRUE,";
		else checksum_check << "FALSE,";
		
		bin = m_pTCalBinFinal.table.temp_f2.coefficient_cheksum;
		cal = temp_f2_checksum;
		temp.clear();
		sprintf((char*)temp.c_str(), "0x%2x,", bin);
		checksum_check << temp.c_str();
		temp.clear();
		sprintf((char*)temp.c_str(), "0x%2x,", cal);
		checksum_check << temp.c_str();
		if (bin == cal) checksum_check << "TRUE,";
		else checksum_check << "FALSE,";

		bin = m_pTCalBinFinal.table.lens.lens_cheksum;
		cal = lens_checksum;
		temp.clear();
		sprintf((char*)temp.c_str(), "0x%2x,", bin);
		checksum_check << temp.c_str();
		temp.clear();
		sprintf((char*)temp.c_str(), "0x%2x,", cal);
		checksum_check << temp.c_str();
		if (bin == cal) checksum_check << "TRUE,";
		else checksum_check << "FALSE,";


		bin = m_pTCalBinFinal.table.ffpn_f1.coefficient_cheksum;
		cal = fppn_f1_checksum;
		temp.clear();
		sprintf((char*)temp.c_str(), "0x%2x,", bin);
		checksum_check << temp.c_str();
		temp.clear();
		sprintf((char*)temp.c_str(), "0x%2x,", cal);
		checksum_check << temp.c_str();
		if (bin == cal) checksum_check << "TRUE,";
		else checksum_check << "FALSE,";

		bin = m_pTCalBinFinal.table.ffpn_f2.coefficient_cheksum;
		cal = fppn_f2_checksum;
		temp.clear();
		sprintf((char*)temp.c_str(), "0x%2x,", bin);
		checksum_check << temp.c_str();
		temp.clear();
		sprintf((char*)temp.c_str(), "0x%2x,", cal);
		checksum_check << temp.c_str();
		if (bin == cal) checksum_check << "TRUE,";
		else checksum_check << "FALSE,";

		bin = m_pTCalBinFinal.table.wigl_f1.LUT_cheksum;
		cal = wigl_f1_checksum;
		temp.clear();
		sprintf((char*)temp.c_str(), "0x%2x,", bin);
		checksum_check << temp.c_str();
		temp.clear();
		sprintf((char*)temp.c_str(), "0x%2x,", cal);
		checksum_check << temp.c_str();
		if (bin == cal) checksum_check << "TRUE,";
		else checksum_check << "FALSE,";

		cal = uint16_wigl_f1_checksum;
		temp.clear();
		sprintf((char*)temp.c_str(), "0x%2x,", cal);
		checksum_check << temp.c_str();
		if (bin == cal) checksum_check << "TRUE,";
		else checksum_check << "FALSE,";

		bin = m_pTCalBinFinal.table.wigl_f2.LUT_cheksum;
		cal = wigl_f2_checksum;
		temp.clear();
		sprintf((char*)temp.c_str(), "0x%2x,", bin);
		checksum_check << temp.c_str();
		temp.clear();
		sprintf((char*)temp.c_str(), "0x%2x,", cal);
		checksum_check << temp.c_str();
		if (bin == cal) checksum_check << "TRUE,";
		else checksum_check << "FALSE,";

		cal = uint16_wigl_f2_checksum;
		temp.clear();
		sprintf((char*)temp.c_str(), "0x%2x,", cal);
		checksum_check << temp.c_str();
		if (bin == cal) checksum_check << "TRUE,";
		else checksum_check << "FALSE,";
	}
#endif

	return ret;
}



BOOL  CInspectToFLSIWigglingCal::WriteCalData()
{
	bool rResult = false;
	bool wResult = false ;
	bool lResult = false;
	

	if(m_pEEPROM == NULL )
	{
		AddLog(_T("[TOFLSICAL-EEPROM] EEPROM is NULL"));
		return FALSE;
	}

	m_pEEPROM->UpdateBuffer(m_nWriteCalAddr, (unsigned char *)&m_pTCalBinFinal.raw, m_nWriteCalSize);
	if ((wResult = m_pEEPROM->Write()) != true)
	{
		AddLog(_T("[TOFLSICAL-EEPROM] Write Cal Data Fail!"));
		return false;
	}
	else
	{
		AddLog(_T("[TOFLSICAL-EEPROM] Write Cal Data Success!"));
	}


	if(m_bReReadCalData == true)
	{
		//byte * tempData = new byte[m_nWriteCalSize];
		m_pEEPROM->Read();
		memcpy(m_pReReadTCalBin.raw,(void*)(m_pEEPROM-> GetMemoryBuffer()),m_nWriteCalSize);
		
		if(m_bSaveReReadCalData)
		{
			FILE *fp;
			string path = m_sPath_Totalbin + "\\_ReRead_ToF_cal.bin";
			fp = fopen(path.c_str(), "wb");
			if (fp == NULL)
				AddLog("Can't write a file! %s \n", path.c_str());
			fwrite((unsigned char *)&m_pReReadTCalBin.raw, 1, sizeof(TLSIcalBin), fp);
			fclose(fp);
		}
		int i = 0;
		while((m_pTCalBinFinal.raw[i] == m_pReReadTCalBin.raw[i++]) == true)
		{
			if(i == m_nWriteCalSize) break;
		}
		if (i == m_nWriteCalSize)
		{
			AddLog(_T("[TOFLSICAL-EEPROM] Re-Read Data Compare Success!"));
		}
		else
		{
			AddLog(_T("[TOFLSICAL-EEPROM] Re-Read Data Compare Fail! : %d data different!"),i);
			return false;
		}
	}
	/*
	//?? 0x2600 ??? read
	//rResult = m_vEEPROM[0]->Read(test_addr,test_Length); 
	before_Data = (byte*)(m_vEEPROM[0] -> GetMemoryBuffer()); 

	//?? ??? ??? write

	for (int i = 0; i < test_Length; i++)
	{
	temp_Data[i] = before_Data[test_addr + i];
	}
	temp_Data[0] = 120;

	m_vEEPROM[0]->UpdateBuffer(test_addr,temp_Data,test_Length);
	wResult = m_vEEPROM[0]->Write(test_addr,test_Length);

	//?? ??? ?? ??
	m_vEEPROM[0]->Read(test_addr,test_Length); 
	after_Data = (byte*)(m_vEEPROM[0] -> GetMemoryBuffer()); 

	//??
	for (int i = 0; i < test_Length; i++)
	{
	cout << i << " : " << before_Data[test_addr + i] << " "<< after_Data[test_addr + i]<< endl;
	}
	*/
	return true;
}


BOOL CInspectToFLSIWigglingCal::Inspection(BOOL bManual/*=FALSE*/)
{

	TimeRecord("[OnTestProc],[Capture],[Inspection]",0);
	ConverImageFormat();
	GetEffectiveImageIndex();
	SaveImage();
	CollectImage();

	bool ret;
	if((ret = PassFailWigglingImg()) != true)
	{
		AddLog(_T("[TOFLSICAL] %d Frame is Not Valid For Wiggling!"),m_nCurrentPhaseShiftIndex);
	}
	else
	{
		AddLog(_T("[TOFLSICAL] %d Frame is Valid For Wiggling!"),m_nCurrentPhaseShiftIndex);
	}

	USER_STOP_CHECK(m_hStopHandle, _gotoEnd, m_nErrorCode);
	AddLog(_T("[TOFLSICAL]end Inspection"));

_gotoEnd:
	TimeRecord("[OnTestProc],[Capture],[Inspection]",1);
	return ret;
}


void CInspectToFLSIWigglingCal::OnPreProcessing(HANDLE hStopHandle)
{
	TimeRecord("[OnPreProcessing]",0);
	//CInspectGeneric::OnPreProcessing(m_vSensorID, strBarCode, nParaIndex, hStopHandle);
	CInspectGeneric::OnPreProcessing(hStopHandle);
	TimeRecord("[OnPreProcessing]",1);
}


float CInspectToFLSIWigglingCal::getAvgROI(float *buf,int WID, int sx, int sy, int wid, int hgt)
{
	float avg, sum;

	if (wid*hgt == 0) return 0;

	sum = 0.0;
	for (int i = sy; i < (sy + hgt); i++)
	{
		for (int j = sx; j < (sx + wid); j++)
		{
			sum += buf[i*WID + j];
		}
	}

	avg = sum / (wid*hgt);
	return avg;
}

void CInspectToFLSIWigglingCal::Convert_shflPhaseToAngle(short *shfl, float *phase, int wid, int hgt)
{
	int sA0, sA1, sA2, sA3;
	short A0, A1, A2, A3;
	double aMin = M_PI, aMax = 0;    // Kevin

	for (int i = 0; i < hgt; i++)
	{
		for (int j = 0; j < wid; j++)
		{
			sA2 = (i * wid * 2 + j) * 2;
			sA3 = (i * wid * 2 + j) * 2 + wid * 2;
			sA0 = (i * wid * 2 + j) * 2 + wid * 2 + 1;
			sA1 = (i * wid * 2 + j) * 2 + 1;

			A0 = shfl[sA0];
			A1 = shfl[sA1];
			A2 = shfl[sA2];
			A3 = shfl[sA3];

			short x = (A1 - A3) + (A0 - A2);
			short y = (A1 - A3) - (A0 - A2);

			double angle = atan2(y, x);

			if (angle < aMin) aMin = angle;
			if (angle > aMax) aMax = angle;

			phase[i*wid + j] = (float)(angle)+M_PI * 1 / 4;
		}
	}

	if ((aMax - aMin) > M_PI*3.0 / 2.0)
	{
		for (int i = 0; i < wid*hgt; i++)
		{
			if (phase[i] < 0)
				phase[i] += 2 * M_PI;
		}
	}
}


int CInspectToFLSIWigglingCal::PhaseShiftCaptureSequence(int i)
{
	int local_errcode = R_RESULT_NONE;
	TimeRecord("[OnTestProc],[Capture],[SetPhaseShift]",0);
	SetPhaseShift(i);
	AddLog(_T("[TOFLSICAL]DelayAfterPhaseShiftChange %d"),m_nDelayAfterPhaseShiftChange);
	Wait(m_nDelayAfterPhaseShiftChange);
	
	TimeRecord("[OnTestProc],[Capture],[SetPhaseShift]",1);
	int thread_ret;
	if(m_bEnableWiggCaptureThread && (i > 0) && (thread_ret = thread_Inspection[i - 1].get() != 1))
	{
		return R_FAIL_CAPTURE;
	}
	//delete &thread_Inspection[i-1];
	//if(i == OPT_MAX_PHASE_SHIFT_NUM) return local_errcode;
	
	for(int j = 0; j < m_nSuperFrameCountPerDistance; j++)
	{
		TimeRecord("[OnTestProc],[Capture],[CheckFrame]",0);
		m_nCurrentSuperFrameIndex = j;	

		local_errcode = CheckFrame(0, m_nTestSkipFrame, 0, nImgPreproc );
		if( local_errcode != R_RESULT_NONE )
		{
			Sleep(50);
			local_errcode = CheckFrame(0, m_nTestSkipFrame, 0, nImgPreproc );
			if( local_errcode != R_RESULT_NONE )
				goto _gotoResult;
		}
		USER_STOP_CHECK(m_hStopHandle, _gotoResult , m_nErrorCode);
		if(m_UIDLLFlag == 0)
		{
			PostMessage(m_hWnd, WM_UPDATE_IMAGE, 0, 0);
		}
		else
		{
			// skip
			*m_gImageFlag = 1;
		}
		TimeRecord("[OnTestProc],[Capture],[CheckFrame]",1);
		//while(*m_gImageFlag == 1);
	}

_gotoResult:
	return local_errcode;
}
int CInspectToFLSIWigglingCal::OnTestProc(HANDLE hStopHandle, void* pUserData)
{
	TimeRecord("[OnTestProc]", 0);
	CInspectGeneric::OnPreProcessing(hStopHandle);
	//fprintf(stderr, "LENS_PatternMatching2Mask : Detected LED Pattern Number less than minimum\n");
#ifdef FILE_WRITE_TEST

	if (m_Grabber.GrabStart() <= 0)
	{
		m_nErrorCode = R_FAIL_CAPTURE;
		goto _gotoResult;
	}

	if (R_RESULT_NONE != (m_nErrorCode = AEAWBProcess(0, m_stAEAWB.nSleepTime, m_stAEAWB.nSkipFrame)))
		goto _gotoResult;

	USER_STOP_CHECK(m_hStopHandle, _gotoResult, m_nErrorCode);

	SetImagePath();
	//int nImgPreproc = 0;
	nImgPreproc = 0;

	{
		AddLog(_T("[TOFLSICAL] Preheating delay %d"), m_DelayForPreheating);
		Wait(m_DelayForPreheating);
		AddLog(_T("[TOFLSICAL] Preheating delay end"));
	}
	//if (m_sInspectSite == "A") {
	if (m_bEnableWiggCapture)
	{
		TimeRecord("[OnTestProc],[WiggCapture]", 0);
		m_eCalMode = eCALMODE::WIGGLING;
		if (m_bEnableWiggCaptureThread)
		{
			for (int i = 0; i <= m_nWiggStep; i++)
			{
				if (i != 0) thread_Inspection[i - 1] = async(launch::async, &CInspectToFLSIWigglingCal::Inspection, this, 0);
				if (i == m_nWiggStep) break;
				if ((m_nErrorCode = PhaseShiftCaptureSequence(i)) != R_RESULT_NONE)
				{
					goto _gotoResult;
				}
				m_nCurrentPhaseShiftIndex = i;
			}
			TimeRecord("[OnTestProc],[WiggCapture]", 1);
		}
		else //if(!m_bEnableWiggCaptureThread)
		{
			for (int i = 0; i < m_nWiggStep; i++)
			{
				m_nCurrentPhaseShiftIndex = i;
				if ((m_nErrorCode = PhaseShiftCaptureSequence(i)) != R_RESULT_NONE)
				{
					goto _gotoResult;
				}
				Inspection();

			}
			TimeRecord("[OnTestProc],[WiggCapture]", 1);
		}
	}

	{
		AddLog(_T("[TOFLSICAL] post_ffpn delay %d"), m_nDelayAfterPhaseShiftChange);
		TimeRecord("[OnTestProc],[DisablePhaseShift]", 0);
		DisablePhaseShift();
		TimeRecord("[OnTestProc],[DisablePhaseShift]", 1);
		Wait(m_nDelayAfterPhaseShiftChange);
		AddLog(_T("[TOFLSICAL] DisablePhaseShift delay %d"), m_nDelayAfterPhaseShiftChange);
	}
	//if (m_bEnableWiggCapture && thread_Inspection[m_nWiggStep - 1].get() != 1)
	{
		AddLog(_T("[TOFLSICAL] %d Frame Capture/Save Fail"), m_nWiggStep - 1);
	}


	if (m_bEnableLEDOffCapture)
	{//capture LED off image
		TimeRecord("[OnTestProc],[LEDOffCapture]", 0);

		m_eCalMode = eCALMODE::LEDOFF;
		m_nCurrentPhaseShiftIndex = MAX_PHASE_SHIFT_NUM + 1;
		for (int j = 0; j < m_nSuperFrameCountPerDistance; j++)
		{
			m_nCurrentSuperFrameIndex = j;
			m_nErrorCode = CheckFrame(0, m_nTestSkipFrame, 0, nImgPreproc);
			if (m_nErrorCode != R_RESULT_NONE)
			{
				Sleep(50);
				m_nErrorCode = CheckFrame(0, m_nTestSkipFrame, 0, nImgPreproc);
				if (m_nErrorCode != R_RESULT_NONE)
					goto _gotoResult;
			}

			Inspection();
			USER_STOP_CHECK(m_hStopHandle, _gotoResult, m_nErrorCode);
			if (m_UIDLLFlag == 0)
			{
				PostMessage(m_hWnd, WM_UPDATE_IMAGE, 0, 0);
			}
			else
			{
				// skip
				*m_gImageFlag = 1;
			}
		}
		TimeRecord("[OnTestProc],[LEDOffCapture]", 1);
	}

	if (m_bEnableLEDonCapture)
	{//LED ON
		m_eCalMode = eCALMODE::LEDON;
		TimeRecord("[OnTestProc],[LEDonCapture]", 0);

		AddLog(_T("[TOFLSICAL] LED ON"));
		string cmd_on = "ON\r\n";
		vector<char> temp(cmd_on.begin(), cmd_on.end());
		if (!m_pSerialComm->Write((unsigned char*)&temp[0], static_cast<int>(temp.size())));
		AddLog(_T("[TOFLSICAL] LED ON delay %d"), m_nDelayAfterLEDON);
		Wait(m_nDelayAfterLEDON);

		m_nCurrentPhaseShiftIndex = MAX_PHASE_SHIFT_NUM + 2;
		for (int j = 0; j < m_nSuperFrameCountPerDistance; j++)
		{
			m_nCurrentSuperFrameIndex = j;
			m_nErrorCode = CheckFrame(0, m_nTestSkipFrame, 0, nImgPreproc);
			if (m_nErrorCode != R_RESULT_NONE)
			{
				Sleep(50);
				m_nErrorCode = CheckFrame(0, m_nTestSkipFrame, 0, nImgPreproc);
				if (m_nErrorCode != R_RESULT_NONE)
					goto _gotoResult;
			}
			Inspection();

			USER_STOP_CHECK(m_hStopHandle, _gotoResult, m_nErrorCode);
			if (m_UIDLLFlag == 0)
			{
				PostMessage(m_hWnd, WM_UPDATE_IMAGE, 0, 0);
			}
			else
			{
				// skip
				*m_gImageFlag = 1;
			}
		}
		AddLog(_T("[TOFLSICAL] LED ON image capture complete"));
		TimeRecord("[OnTestProc],[LEDonCapture]", 1);
	}

	m_eCalMode = eCALMODE::NONE;
	{//LED OFF
		AddLog(_T("[TOFLSICAL] LED off"));
		string cmd_on = "OFF\r\n";
		vector<char> temp(cmd_on.begin(), cmd_on.end());
		if (!m_pSerialComm->Write((unsigned char*)&temp[0], static_cast<int>(temp.size())));
		//AddLog(_T("[TOFLSICAL] LED off delay %d"), m_nDelayAfterLEDOFF);
		//Wait(m_nDelayAfterLEDOFF);
	}
	//}

	if (m_bEnableWiggCapture)
	{
		ofstream wiggresult(m_sBasePath_wiggling + "\\" + "result.csv");
		for (int i = 0; i < 2; i++)
		{
			wiggresult << "Freq_" << i + 1 << endl;
			for (int j = 0; j < m_nWiggStep; j++)
			{
				if (cur_fX_phase_avg[i][j] < 0)
				{
					wiggresult << "FAIL" << ",";
				}
				else
				{
					wiggresult << cur_fX_phase_avg[i][j] << ",";
				}
			}
			wiggresult << endl;
		}
		wiggresult.close();
	}

	if(m_bEnableCalAlgo)
	{ 
	TimeRecord("[OnTestProc],[CalAlgo]", 0);
	//if (m_sInspectSite == "B")
	
		unsigned char *ucTempF1 = (unsigned char *)&m_pTCalBin.table.temp_f1;
		unsigned char *ucTempF2 = (unsigned char *)&m_pTCalBin.table.temp_f2;
		unsigned char *ucWiggF1 = (unsigned char *)&m_pTCalBin.table.wigl_f1;
		unsigned char *ucWiggF2 = (unsigned char *)&m_pTCalBin.table.wigl_f2;
		unsigned char *ucLens = (unsigned char *)&m_pTCalBin.table.lens;
		unsigned char *ucFppnF1 = (unsigned char *)&m_pTCalBin.table.ffpn_f1;
		unsigned char *ucFppnF2 = (unsigned char *)&m_pTCalBin.table.ffpn_f2;

		short* nshfl_on = (short *)&m_sCalImageSet.LEDon_nshfl_f2.data[0];
		short* nshfl_off = (short *)&m_sCalImageSet.LEDoff_nshfl_f2.data[0];
		short* shfl_on = (short *)&m_sCalImageSet.LEDon_shfl_f2.data[0];
		short* shfl_off = (short *)&m_sCalImageSet.LEDoff_shfl_f2.data[0];

		short* nshfl_off_f1 = (short *)&m_sCalImageSet.LEDoff_nshfl_f1.data[0];
		short* nshfl_off_f2 = (short *)&m_sCalImageSet.LEDoff_nshfl_f2.data[0];
		short* shfl_off_f1 = (short *)&m_sCalImageSet.LEDoff_shfl_f1.data[0];
		short* shfl_off_f2 = (short *)&m_sCalImageSet.LEDoff_shfl_f2.data[0];
		vector<byte> *temp_emb_nshfl_off_f1 = &m_sCalImageSet.LEDoff_nshfl_f1_embd;
		vector<byte> *temp_emb_nshfl_off_f2 = &m_sCalImageSet.LEDoff_nshfl_f2_embd;
		byte* emb_nshfl_off_f1 = (byte*)temp_emb_nshfl_off_f1->data();
		byte* emb_nshfl_off_f2 = (byte*)temp_emb_nshfl_off_f2->data();
		byte* comp_emb_nshfl_off_f1 = new byte[DEPTH_SIZE];

		short* nshfl_16_f1 = (short *)&m_sCalImageSet.wiggling_nshfl_f1.data[0];
		short* nshfl_16_f2 = (short *)&m_sCalImageSet.wiggling_nshfl_f2.data[0];
		short* shfl_16_f1 = (short *)&m_sCalImageSet.wiggling_shfl_f1.data[0];
		short* shfl_16_f2 = (short *)&m_sCalImageSet.wiggling_shfl_f2.data[0];
		{
			/*m_sBasePath_LEDon = "C:\\log\\boardCmpD\\5DD28A938360_20210823_140949\\LedOn";
			m_sBasePath_LEDoff = "C:\\log\\boardCmpD\\5DD28A938360_20210823_140949\\LedOff";
			m_sBasePath_wiggling = "C:\\log\\boardCmpD\\5DD28A938360_20210823_140949\\wiggling";
			m_sPath_Tempbin = "C:\\log\\temp_drift";*/
			AddLog(_T("LEDon Image Path : %s"), m_sBasePath_LEDon.c_str());
			AddLog(_T("LEDoff Image Path : %s"), m_sBasePath_LEDoff.c_str());
			AddLog(_T("wiggling Image Path : %s"), m_sBasePath_wiggling.c_str());
			AddLog(_T("Temperature drift Bin File Path : %s"), m_sPath_Tempbin.c_str());

			if (LSI_LoadDataTemperature_path((char*)m_sPath_Tempbin.c_str(), m_nFreqF1, ucTempF1)
				|| LSI_LoadDataTemperature_path((char*)m_sPath_Tempbin.c_str(), m_nFreqF2, ucTempF2))
			{
				AddLog(_T("[TOFLSICAL] LSI_LoadDataTemperature error"));
				m_nErrorCode = R_RESULT_FAIL;
			}
			if (LSI_LoadDataLens_path((char*)m_sBasePath_LEDon.c_str(), (char*)m_sBasePath_LEDoff.c_str(), nshfl_on, shfl_on, nshfl_off, shfl_off))
			{
				AddLog(_T("Lens Cal Data Loading error"));
				m_nErrorCode = R_RESULT_FAIL;
			}
			if (LSI_LoadDataFppn_path((char*)m_sBasePath_LEDoff.c_str(), nshfl_off_f1, shfl_off_f1, emb_nshfl_off_f1, m_nFreqF1) ||
				LSI_LoadDataFppn_path((char*)m_sBasePath_LEDoff.c_str(), nshfl_off_f2, shfl_off_f2, emb_nshfl_off_f2, m_nFreqF2) ||
				LSI_LoadDataFppn_path((char*)m_sBasePath_LEDoff.c_str(), nshfl_off_f1, shfl_off_f1, comp_emb_nshfl_off_f1, m_nFreqF1))
			{
				AddLog(_T("Fppn Cal Data Loading error"));
				m_nErrorCode = R_RESULT_FAIL;
			}

			if (LSI_LoadDataWiggling_path((char*)m_sBasePath_wiggling.c_str(), nshfl_16_f1, shfl_16_f1, m_nFreqF1) ||
				LSI_LoadDataWiggling_path((char*)m_sBasePath_wiggling.c_str(), nshfl_16_f2, shfl_16_f2, m_nFreqF2))
			{
				AddLog(_T("Wiggling Cal Data Loading error"));
				m_nErrorCode = R_RESULT_FAIL;
			}
			if (m_nErrorCode == R_RESULT_FAIL)
				goto _gotoResult;


			if (LSI_CalibrationWiggling(nshfl_16_f1, shfl_16_f1, m_nFreqF1, ucWiggF1))
			{
				AddLog(_T("Freq 1 Wiggling Calibration error"));
				m_nErrorCode = R_RESULT_FAIL;
			}
			if (LSI_CalibrationWiggling(nshfl_16_f2, shfl_16_f2, m_nFreqF2, ucWiggF2))
			{
				AddLog(_T("Freq 2 Wiggling Calibration error"));
				m_nErrorCode = R_RESULT_FAIL;
			}

			double m_dLensParams[9];
			double m_dReprojectionError;

			if (LSI_CalibrationLens(nshfl_on, shfl_on, nshfl_off, shfl_off, (float *)m_sCalImageSet.gt.data, ucLens))
			{
				AddLog(_T("Lens Wiggling Calibration error"));
				m_nErrorCode = R_RESULT_FAIL;
			}
			if (LSI_CalibrationFppn(nshfl_off_f1, shfl_off_f1, emb_nshfl_off_f1, (float *)m_sCalImageSet.gt.data, m_nFreqF1, ucFppnF1))
			{
				AddLog(_T("Freq 1 FPPN Wiggling Calibration error"));
				m_nErrorCode = R_RESULT_FAIL;
			}
			if (LSI_CalibrationFppn(nshfl_off_f2, shfl_off_f2, emb_nshfl_off_f2, (float *)m_sCalImageSet.gt.data, m_nFreqF2, ucFppnF2))
			{
				AddLog(_T("Freq 2 FPPN Wiggling Calibration error"));
				m_nErrorCode = R_RESULT_FAIL;
			}
			AddLog(_T("[TOFLSICAL] LensParams : %lf , %lf , %lf , %lf , %lf , %lf , %lf , %lf"), m_dLensParams[0], m_dLensParams[1], m_dLensParams[2], m_dLensParams[3], m_dLensParams[4], m_dLensParams[5], m_dLensParams[6], m_dLensParams[7]);
			AddLog(_T("[TOFLSICAL] Reprojection Error : %lf"), m_dReprojectionError);

			if (m_nErrorCode == R_RESULT_FAIL)
				goto _gotoResult;
			LSI_OutTotalCalData(ucLens, ucTempF1, ucTempF2, ucWiggF1, ucWiggF2, ucFppnF1, ucFppnF2, m_pTCalBinFinal.raw);

			//파일로 빼는 작업
			int verify_errCode = R_RESULT_NONE;
			if (verify_errCode = VerifyCalData() == TRUE)
			{
				AddLog(_T("[TOFLSICAL-EEPROM] Data PreVerify Success! result : %d"), verify_errCode);
			}
			else
			{
				AddLog(_T("[TOFLSICAL-EEPROM] Data PreVerify Fail! result : %d"), verify_errCode);
			}

			//write cal bin file
			{
				FILE *fp;
				string path = m_sPath_Totalbin + "\\_ToF_cal_" + to_string(m_nFreqF2) + ".bin";
				fp = fopen(path.c_str(), "wb");
				if (fp == NULL)
					AddLog("Can't write a file! %s \n", path.c_str());
				fwrite(m_pTCalBinFinal.raw, 1, sizeof(TLSIcalBin), fp);
				fclose(fp);
				AddLog(_T("[TOFLSICAL]Write Total bin file OK"));
			}
		}
	}


	if (m_bEnableSaveCalBin == true)
	//if (m_sInspectSite == "C")
	{
		//read cal bin file
		FILE *fp = fopen((m_sPath_Totalbin + "\\_ToF_cal_" + to_string(m_nFreqF2) + ".bin").c_str(), "rb");
		if (fp == NULL)
			AddLog("Can't write a file! %s \n", m_sCalFilePath.c_str());
		fread((unsigned char *)&m_pTCalBinFinal.raw, 1, sizeof(TLSIcalBin), fp);
		AddLog(_T("[TOFLSICAL-EEPROM] Load Cal bin File Success!"));

		WriteCalData();
	}

	
	TimeRecord("[OnTestProc],[CalAlgo]", 1);
	/*}//cal algo */

#else 
	WriteCalData();
#endif
_gotoResult:

	m_Grabber.GrabStop();
	TimeRecord("[OnTestProc]",1);
	return JudgeResult();
}

int CInspectToFLSIWigglingCal::JudgeResult(BOOL bManual/*=FALSE*/, void* pUserData/*=NULL*/)
{
	TimeRecord("[JudgeResult]",0);

	if( !bManual && m_nErrorCode != R_RESULT_NONE )
		return m_nErrorCode;

	m_nErrorCode = R_RESULT_PASS;


	int nResult = R_RESULT_PASS;

	TimeRecord("[JudgeResult]",1);
	



	if(m_UIDLLFlag)
	{
		*m_gImageFlag = 2;
		TimeBreakDown();
	}
	return m_nErrorCode;
}

void CInspectToFLSIWigglingCal::TimeBreakDown()
{
	AddLog("[Elapsed Time],Function,Record(sec),start,end");
	for (int i = 0; i < m_clock_seq.size(); i++)
	{
		AddLog("[Elapsed Time],%s,%lf", m_clock_seq[i].func_name.c_str() ,m_clock_seq[i].time);
		//AddLog("[Elapsed Time],%s,%lf", m_clock_seq[i].func_name.c_str() ,m_clock_seq[i].time,m_clock_seq[i].start,m_clock_seq[i].end);
	}
}

void CInspectToFLSIWigglingCal::TimeRecord(string func_name,int flag)
{
	if(flag == 0)
	{
		m_clock[func_name] = clock();
	}
	else
	{
		TIME temp;
		temp.start = m_clock[func_name];
		temp.end = clock();
		temp.func_name = func_name;
		temp.time = (double)(temp.end - temp.start)/CLOCKS_PER_SEC;
		m_clock_seq.push_back(temp);
	}
}

void CInspectToFLSIWigglingCal::GetWhitePatchRect(CxDRect& rt)
{
	TimeRecord("[GetWhitePatchRect]",0);
	if( !m_stAEAWB.dCalRoiSizeRatio ) return;

	int nWidth = m_ImageObject.GetWidth();
	int nHeight = m_ImageObject.GetHeight();

	rt.left = (nWidth/2)*(1-m_stAEAWB.dCalRoiSizeRatio);
	rt.top = (nHeight/2)*(1-m_stAEAWB.dCalRoiSizeRatio);
	rt.right= (nWidth/2)*(1+m_stAEAWB.dCalRoiSizeRatio);
	rt.bottom= (nHeight/2)*(1+m_stAEAWB.dCalRoiSizeRatio);
	TimeRecord("[GetWhitePatchRect]",1);
}


void CInspectToFLSIWigglingCal::DisplayGrapics(int nImageWidth, int nImageHeight)
{

}