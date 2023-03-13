#include "stdafx.h"
#include <iostream>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\opencv.hpp>
#include "LSI_Convert.h"
#include "subutil.h"

using namespace cv;
using namespace std;

#define VALID_LEVEL 20

void Convert_phase2depth(short *nshfl, short *shfl, float *depth, float *ampl, float *inten, int wid, int hgt, int f, char mode)
{
	//  0  270        180 90
	//  90 180        270 0
	//  Normal        shuffle

	int fm = f * 1000000;
	int nsA0, nsA1, nsA2, nsA3;
	int sA0, sA1, sA2, sA3;
	short A0, A1, A2, A3;

	for (int i = 0; i < hgt; i++)
	{
		for (int j = 0; j < wid; j++)
		{
			nsA0 = (i * wid * 2 + j) * 2;
			nsA1 = (i * wid * 2 + j) * 2 + wid * 2;
			nsA2 = (i * wid * 2 + j) * 2 + wid * 2 + 1;
			nsA3 = (i * wid * 2 + j) * 2 + 1;

			sA2 = (i * wid * 2 + j) * 2;
			sA3 = (i * wid * 2 + j) * 2 + wid * 2;
			sA0 = (i * wid * 2 + j) * 2 + wid * 2 + 1;
			sA1 = (i * wid * 2 + j) * 2 + 1;

			A0 = (nshfl[nsA0] + shfl[sA0]) / 2;
			A1 = (nshfl[nsA1] + shfl[sA1]) / 2;
			A2 = (nshfl[nsA2] + shfl[sA2]) / 2;
			A3 = (nshfl[nsA3] + shfl[sA3]) / 2;

			if (mode & PHASE_DEPTH)
			{
				short x = (A1 - A3) + (A0 - A2);
				short y = (A1 - A3) - (A0 - A2);
				
				double angle = atan2(y,x);
				float th;

				if (angle < 0)
					th = (float)(angle)+PI * 9 / 4;
				else
					th = (float)(angle)+PI * 1 / 4;

				depth[i*wid + j] = SOL * th / (2 * fm * 2 * PI);
			}
			if (mode & PHASE_AMPLITUDE)
				ampl[i*wid + j] = (float)sqrt((A0 - A2)*(A0 - A2) + (A1 - A3)*(A1 - A3)) / 2;
			if (mode & PHASE_INTENSITY)
				inten[i*wid + j] = (float)(A0 + A1 + A2 + A3) / 4;
		}
	}
}

//
void Convert_phase2int(short* nshfl, short* shfl, float* inten, int wid, int hgt)
{
	int nsA0, nsA1, nsA2, nsA3;
	int sA0, sA1, sA2, sA3;
	short A0, A1, A2, A3;

	for (int i = 0; i < hgt; i++)
	{
		for (int j = 0; j < wid; j++)
		{
			nsA0 = (i * wid * 2 + j) * 2;
			nsA1 = (i * wid * 2 + j) * 2 + wid * 2;
			nsA2 = (i * wid * 2 + j) * 2 + wid * 2 + 1;
			nsA3 = (i * wid * 2 + j) * 2 + 1;

			sA2 = (i * wid * 2 + j) * 2;
			sA3 = (i * wid * 2 + j) * 2 + wid * 2;
			sA0 = (i * wid * 2 + j) * 2 + wid * 2 + 1;
			sA1 = (i * wid * 2 + j) * 2 + 1;

			A0 = (nshfl[nsA0] + shfl[sA0]) / 2;
			A1 = (nshfl[nsA1] + shfl[sA1]) / 2;
			A2 = (nshfl[nsA2] + shfl[sA2]) / 2;
			A3 = (nshfl[nsA3] + shfl[sA3]) / 2;
				
			inten[i * wid + j] = (float)(A0 + A1 + A2 + A3) / 4;
		}
	}
}

//
void Convert_phaseToAngle(short *nshfl, short *shfl, float *phase, int wid, int hgt)
{
	int nsA0, nsA1, nsA2, nsA3;
	int sA0, sA1, sA2, sA3;
	short A0, A1, A2, A3;
	double aMin = PI, aMax = 0;    // Kevin

	for (int i = 0; i < hgt; i++)
	{
		for (int j = 0; j < wid; j++)
		{
			nsA0 = (i * wid * 2 + j) * 2;
			nsA1 = (i * wid * 2 + j) * 2 + wid * 2;
			nsA2 = (i * wid * 2 + j) * 2 + wid * 2 + 1;
			nsA3 = (i * wid * 2 + j) * 2 + 1;

			sA2 = (i * wid * 2 + j) * 2;
			sA3 = (i * wid * 2 + j) * 2 + wid * 2;
			sA0 = (i * wid * 2 + j) * 2 + wid * 2 + 1;
			sA1 = (i * wid * 2 + j) * 2 + 1;

			A0 = (nshfl[nsA0] + shfl[sA0]) / 2;
			A1 = (nshfl[nsA1] + shfl[sA1]) / 2;
			A2 = (nshfl[nsA2] + shfl[sA2]) / 2;
			A3 = (nshfl[nsA3] + shfl[sA3]) / 2;

			short x = (A1 - A3) + (A0 - A2);
			short y = (A1 - A3) - (A0 - A2);

			double angle = atan2(y, x);

			if (angle < aMin) aMin = angle;
			if (angle > aMax) aMax = angle;

			phase[i*wid + j] = (float)(angle)+PI * 1 / 4;
		}
	}

	if ((aMax - aMin) > PI*3.0 / 2.0)
	{
		for (int i = 0; i < wid*hgt; i++)
		{
			if (phase[i] < 0)
				phase[i] += 2 * PI;
		}
	}
}

void Convert_phaseToAngle(float *nshfl, float *shfl, float *phase, int wid, int hgt)
{
	int nsA0, nsA1, nsA2, nsA3;
	int sA0, sA1, sA2, sA3;
	float A0, A1, A2, A3;
	double aMin = PI, aMax = 0;    // Kevin

	for (int i = 0; i < hgt; i++)
	{
		for (int j = 0; j < wid; j++)
		{
			nsA0 = (i * wid * 2 + j) * 2;
			nsA1 = (i * wid * 2 + j) * 2 + wid * 2;
			nsA2 = (i * wid * 2 + j) * 2 + wid * 2 + 1;
			nsA3 = (i * wid * 2 + j) * 2 + 1;

			sA2 = (i * wid * 2 + j) * 2;
			sA3 = (i * wid * 2 + j) * 2 + wid * 2;
			sA0 = (i * wid * 2 + j) * 2 + wid * 2 + 1;
			sA1 = (i * wid * 2 + j) * 2 + 1;

			A0 = (nshfl[nsA0] + shfl[sA0]) / 2;
			A1 = (nshfl[nsA1] + shfl[sA1]) / 2;
			A2 = (nshfl[nsA2] + shfl[sA2]) / 2;
			A3 = (nshfl[nsA3] + shfl[sA3]) / 2;

			short x = (A1 - A3) + (A0 - A2);
			short y = (A1 - A3) - (A0 - A2);

			double angle = atan2(y, x);

			if (angle < aMin) aMin = angle;
			if (angle > aMax) aMax = angle;

			phase[i*wid + j] = (float)(angle)+PI * 1 / 4;
		}
	}

	if ((aMax - aMin) > PI*3.0 / 2.0)
	{
		for (int i = 0; i < wid*hgt; i++)
		{
			if (phase[i] < 0)
				phase[i] += 2 * PI;
		}
	}
}

void Convert_angleToDistance(float* angle, int freq_Mhz, int size, float* depth_out)
{
	for (int i = 0; i < size; i++)
	{
		depth_out[i] = SOL * angle[i] / (2 * freq_Mhz * 2 * PI) / 1000.0f;    // mm unit
	}
}

//
void Convert_distanceToAngle(float* distance, int freq_Mhz, int size, float* angle)
{
	for (int i = 0; i < size; i++)
	{
		angle[i] = distance[i] * 2 * freq_Mhz * 2 * PI * 1000.0 / SOL;
	}
}

//
void Convert_scaleGT(float* input, int size, float* output)
{
	for (int i = 0; i < size; i++)
	{
		output[i] = input[i] * 1000.;
		//angle[i] = (gt[i] * 1000.) * 2 * freq_Mhz * 2 * PI * 1000.0 / SOL;
		//angle[i] = angle[i] * (1.0f / 2.0 / PI);
	}
}

void Convert_phaseToIntensity(short *nshfl, short *shfl, float *inten, int wid, int hgt)
{
	int nsA0, nsA1, nsA2, nsA3;
	int sA0, sA1, sA2, sA3;
	short A0, A1, A2, A3;

	for (int i = 0; i < hgt; i++)
	{
		for (int j = 0; j < wid; j++)
		{
			nsA0 = (i * wid * 2 + j) * 2;
			nsA1 = (i * wid * 2 + j) * 2 + wid * 2;
			nsA2 = (i * wid * 2 + j) * 2 + wid * 2 + 1;
			nsA3 = (i * wid * 2 + j) * 2 + 1;

			sA2 = (i * wid * 2 + j) * 2;
			sA3 = (i * wid * 2 + j) * 2 + wid * 2;
			sA0 = (i * wid * 2 + j) * 2 + wid * 2 + 1;
			sA1 = (i * wid * 2 + j) * 2 + 1;

			A0 = (nshfl[nsA0] + shfl[sA0]) / 2;
			A1 = (nshfl[nsA1] + shfl[sA1]) / 2;
			A2 = (nshfl[nsA2] + shfl[sA2]) / 2;
			A3 = (nshfl[nsA3] + shfl[sA3]) / 2;

			inten[i*wid + j] = (float)(A0 + A1 + A2 + A3)/4;
		}
	}
}

void Convert_phaseToAmplitude(short *nshfl, short *shfl, float *amplitude, int wid, int hgt)
{
	int nsA0, nsA1, nsA2, nsA3;
	int sA0, sA1, sA2, sA3;
	short A0, A1, A2, A3;

	for (int i = 0; i < hgt; i++)
	{
		for (int j = 0; j < wid; j++)
		{
			nsA0 = (i * wid * 2 + j) * 2;
			nsA1 = (i * wid * 2 + j) * 2 + wid * 2;
			nsA2 = (i * wid * 2 + j) * 2 + wid * 2 + 1;
			nsA3 = (i * wid * 2 + j) * 2 + 1;

			sA2 = (i * wid * 2 + j) * 2;
			sA3 = (i * wid * 2 + j) * 2 + wid * 2;
			sA0 = (i * wid * 2 + j) * 2 + wid * 2 + 1;
			sA1 = (i * wid * 2 + j) * 2 + 1;

			A0 = (nshfl[nsA0] + shfl[sA0]) / 2;
			A1 = (nshfl[nsA1] + shfl[sA1]) / 2;
			A2 = (nshfl[nsA2] + shfl[sA2]) / 2;
			A3 = (nshfl[nsA3] + shfl[sA3]) / 2;

			amplitude[i*wid + j] = (float)sqrt((A0 - A2)*(A0 - A2) + (A1 - A3)*(A1 - A3))/2;
		}
	}
}

void Convert_phase2depth_intp(short *nshfl, short *shfl, float *depth, float *ampl, float *inten, int wid, int hgt, int f, char mode)
{
	//  0  270        180 90
	//  90 180        270 0
	//  Normal        shuffle

	int fm = f * 1000000;
	int nsA0, nsA1, nsA2, nsA3;
	int sA0, sA1, sA2, sA3;
	short A0, A1, A2, A3, tmp;

	for (int i = 0; i < hgt-1; i++)
	{
		for (int j = 0; j < wid-1; j++)
		{
			nsA0 = (i * wid + j);
			nsA1 = (i * wid + j) + wid;
			nsA2 = (i * wid + j) + wid + 1;
			nsA3 = (i * wid + j) + 1;

			sA2 = (i * wid + j);
			sA3 = (i * wid + j) + wid;
			sA0 = (i * wid + j) + wid + 1;
			sA1 = (i * wid + j) + 1;

			A0 = nshfl[nsA0] +shfl[sA0];
			A1 = nshfl[nsA1] + shfl[sA1];
			A2 = nshfl[nsA2] + shfl[sA2];
			A3 = nshfl[nsA3] + shfl[sA3];

			if (i & 1)
			{
				tmp = A0; A0 = A1; A1 = tmp;
				tmp = A2; A2 = A3; A3 = tmp;
			}

			if (j & 1)
			{
				tmp = A0; A0 = A3; A3 = tmp;
				tmp = A1; A1 = A2; A2 = tmp;
			}

			if (mode & PHASE_DEPTH)
			{
				short x = (A1 - A3) + (A0 - A2);
				short y = (A1 - A3) - (A0 - A2);

				double angle = atan2(y, x);
				float th;

				if (angle < 0)
					th = (float)(angle)+PI * 9 / 4;
				else
					th = (float)(angle)+PI * 1 / 4;

				depth[i*wid + j] = SOL * th / (2 * fm * 2 * PI);
			}
			if (mode & PHASE_AMPLITUDE)
				ampl[i*wid + j] = (float)sqrt((A0 - A2)*(A0 - A2) + (A1 - A3)*(A1 - A3)) / 2;
			if (mode & PHASE_INTENSITY)
				inten[i*wid + j] = (float)(A0 + A1 + A2 + A3) / 4;
		}
	}
}


// Calculate distance using two frequency.
#define N1 3
#define N2 4
#define DUAL_MAX_DIST 7500
void Convert_combinedDepth(float* depth_f1, float* depth_f2, int size, float* depth_out)
{
	float depth;
	float diff;
	int scale, remainder;
	int offset[8] = { 4500, 3000, 1500, 0, 6000, 4500, 3000, 1500 };
	for (int i = 0; i < size; i++)
	{
		diff = (depth_f2[i] - depth_f1[i]);
		scale = int(diff / 375 + 0.5);
		remainder = diff - scale * 375;

		depth = offset[scale+3] + remainder / 2 + depth_f1[i];
		depth_out[i] = depth - int(depth / DUAL_MAX_DIST) * DUAL_MAX_DIST;
	}
}

//
void Convert_scaleValue(float *input, int size, float scale, float *output)
{
	for (int i = 0; i < size; i++)
		output[i] = input[i] * scale;
}

//
bool Convert_scaleSize(float *input, int wid, int hgt, int scaleX, int scaleY, float *output)
{
	float *scaleXbuf = new float[wid*hgt];

	memset(scaleXbuf, 0, wid*hgt * sizeof(float));
	memset(output, 0, wid*hgt * sizeof(float));

	if ((wid % scaleX) || (hgt % scaleY)) return false;

	for (int i = 0; i < hgt; i ++)
	{
		for (int j = 0; j < wid; j++)
		{
			scaleXbuf[i*wid / scaleX + j / scaleX] += input[i*wid+j];
		}
	}

	wid = wid / scaleX;
	for (int i = 0; i < hgt; i++)
	{
		for (int j = 0; j < wid; j++)
		{
			output[(i/scaleY)*wid + j] += scaleXbuf[i*wid+j];
		}
	}

	hgt = hgt / scaleY;
	for (int i = 0; i < hgt; i++)
	{
		for (int j = 0; j < wid; j++)
		{
			output[i*wid + j] = output[i*wid + j] / scaleX / scaleY;
		}
	}

	delete scaleXbuf;

	return true;
}

void Convert_diffValue(float *in1, float *in2, int size, float *out)
{
	for (int i = 0; i < size; i++)
		out[i] = in1[i] - in2[i];
}

// converting phase to angle (for wiggling)
void Convert_phaseToAngle_wigg(short *nshfl, short *shfl, float *phase, int wid, int hgt, int wiggStep)
{
	int nsA0, nsA1, nsA2, nsA3;
	int sA0, sA1, sA2, sA3;
	short A0, A1, A2, A3;
	int size = hgt * wid;

	for (int cnt = 0; cnt < wiggStep; cnt++)
	{
		for (int i = 0; i < hgt; i++)
		{
			for (int j = 0; j < wid; j++)
			{
				nsA0 = (cnt*size * 4) + (i*wid * 2 + j) * 2;
				nsA1 = (cnt*size * 4) + (i*wid * 2 + j) * 2 + wid * 2;
				nsA2 = (cnt*size * 4) + (i*wid * 2 + j) * 2 + wid * 2 + 1;
				nsA3 = (cnt*size * 4) + (i*wid * 2 + j) * 2 + 1;

				sA2 = (cnt*size * 4) + (i*wid * 2 + j) * 2;
				sA3 = (cnt*size * 4) + (i*wid * 2 + j) * 2 + wid * 2;
				sA0 = (cnt*size * 4) + (i*wid * 2 + j) * 2 + wid * 2 + 1;
				sA1 = (cnt*size * 4) + (i*wid * 2 + j) * 2 + 1;

				A0 = (nshfl[nsA0] + shfl[sA0]) / 2;
				A1 = (nshfl[nsA1] + shfl[sA1]) / 2;
				A2 = (nshfl[nsA2] + shfl[sA2]) / 2;
				A3 = (nshfl[nsA3] + shfl[sA3]) / 2;

				short x = (A1 - A3) + (A0 - A2);
				short y = (A1 - A3) - (A0 - A2);

				double angle = atan2(y, x);
				//phase[(cnt*size) + i * wid + j] = (float)(angle) + PI;

				if (angle < 0)
					phase[(cnt*size) + i * wid + j] = (float)(angle)+(PI / 4) + (PI * 2);
				else
					phase[(cnt*size) + i * wid + j] = (float)(angle)+(PI / 4);


				if (phase[(cnt*size) + i * wid + j] >= 2 * PI) 
				{
					phase[(cnt*size) + i * wid + j] -= 2 * PI;
				}

			}
		}
	}
}

// wigggling ROI averaging
void Convert_average_wigg(float* input, float* avg, int cnt, int wid, int hgt)
{
	int curPos = 0;
	float min_input, max_input;
	float pre_min, pre_max;
	int pos_hgt = (LSI_HGT/2) - (hgt/2);
	int pos_wid = (LSI_WID/2) - (wid/2);

	float* temp = new float[hgt*wid];

	for (int count = 0; count < cnt; count++)
	{
		int cntpixel = 0;
		double sum = 0.;
		double sum_temp = 0.;

		vector<float> temp_phase(hgt*wid);
		for (int i = 0; i < hgt; i++)
		{
			for (int j = 0; j < wid; j++)
			{
				curPos = (LSI_HGT*LSI_WID*count) + LSI_WID*(pos_hgt + i) + (pos_wid + j);

				sum += input[curPos];
				temp[cntpixel] = input[curPos];
				temp_phase[cntpixel] = temp[cntpixel];
				if (cntpixel == 0)
				{
					pre_min = temp[0];
					pre_max = temp[0];
				}
				else
				{
					min_input = min(pre_min, temp[cntpixel]);
					pre_min = min_input;

					max_input = max(pre_max, temp[cntpixel]);
					pre_max = max_input;
				}
				
				cntpixel++;
			}
		}
		TestGraph(temp_phase);
		if (max_input - min_input > PI)
		{
			for (int i = 0; i < hgt; i++)
			{
				for (int j = 0; j < wid; j++)
				{
					if(temp[i*wid+j] < PI)
						temp[i*wid + j] += 2 * PI;
					
					sum_temp += temp[i*wid + j];
				}
			}
			avg[count] = sum_temp / cntpixel;
		}
		else
		{
			avg[count] = sum / cntpixel;
		}		
	}

	delete temp;
}

