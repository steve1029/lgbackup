#pragma once

#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\opencv.hpp>
#include <opencv2\core\cvstd.hpp>

#define LSI_WID 640
#define LSI_HGT 480
#define LED_WID 13
#define LED_HGT 11
#define LEDMASK_LEN 19

#define MIN_BLOB_SIZE 2
#define LED_PATTERN_MAX 142

// 2020_02_13 KDS : Lens Calibration C Porting from Matlab 
typedef struct {
	// 추후 json 이나 config 파일에서 load 하도록 수정해야함	
	float template_radius = 5.0f;
	float inner_radius = 1.5f;
	float outer_radius = 6.0f;
	float noise_threshold = 23.0f;
	int ratio_threshold = 25;
	int detected_blob_num;
	int spot_size = 5;
	std::vector<cv::Point2d> pixel_pos;
	double reweight_border[3] = { -1.0, 0.0, 1.0 };
	
}LENS_SPOT_DETECTOR;

typedef struct {
	LENS_SPOT_DETECTOR spot_detector;
	// led Pattern Mask Data
	// 19 by 19 led pattern mask
	int pattern_len[2];
	int* pattern_mask;
	// led center when letf top point is org == (0, 0), (meter)
	// pattern_center[2] is ground truth distance between sensor and led plane
	float pattern_center[3] = { 0.2700f, 0.2700f, 0.3722f };
	// distance between led points(meter)
	float pattern_spacing = 0.03f;
	// lens model
	// fx, fy, cx, cy, k1, k2, p1, p2, p3
	double lens_params[9] = { 561.9844, 562.44361, 300.52637, 234.26241, 0.10949, -0.25437, 0.0, 0.00038, 0.0 };
	double rVec[3] = { 0.0, 0.0, 0.0 };
	double tVec[3] = { 0.0, 0.0, 0.0 };
	double reprojection_error = 0.0f;
	double projection_error = 0.0f;
	// Cx Cy Limit in Pattern Matching
	int cx_lim[2];
	int cy_lim[2];

	// LED Pattern Distance when left top is origin(0m,0m)
	// X,Y,Z
	std::vector<cv::Point3d> pattern;
	std::vector<cv::Point2d> pr_imagePoints;
}LENS_CAL_OBJECT;

typedef struct {
	std::vector<cv::Point2d> imagePoints;
	std::vector<cv::Point2d> pr_imagePoints;
	std::vector<cv::Point3d> objectPoints;
}PROJECTION_DATA;

void LENS_CalibrationMain( LENS_CAL_OBJECT* lens_obj, float* intensity_diff_image, float *gt);
void LENS_CalculatePattern( LENS_CAL_OBJECT* lens_obj);
void LENS_DetectPattern(LENS_CAL_OBJECT* lens_obj, float* led_image);
void LENS_PatternMatching2Mask( LENS_CAL_OBJECT* lens_obj);
void LENS_FitLensParams( LENS_CAL_OBJECT* lens_obj);
std::vector<std::pair<int, int>> LENS_subfuntion_Blob_floodFill(bool* logical_image, short* blob_image, short* pixel_num_blob, bool* isVisited, int arg_h, int arg_w, int* blob_count);
void LENS_subfunction_CreateTemplate(std::vector<cv::Point2d> points, int wid, int hgt, int spot_size, double* template_image);
void LENS_subfuntion_Blob_floodFill_Equal_Matlab(bool* logical_image, short* blob_image, int* blob_found_ptr, std::vector<std::pair<int, int>> *pixel_location_per_blob);
cv::Point2i LENS_subfunction_FindMaxIdx(double* arr, int wid, int hgt);
std::pair<double, int> LENS_subfunction_Find_Min_Value_Idx(double* arr, int size);
double LENS_subfunction_Calc_Error_Polynomial(const std::vector<double>& x, std::vector<double>& grad, void* my_func_data);
double LENS_subfunction_Calc_Reprojection_Error(const std::vector<double>& x, PROJECTION_DATA* d);
void LENS_MakeGT(LENS_CAL_OBJECT* lens_obj, float* gt);
void LENS_Calc_Undistortion_Points(double* lens_params, double* x_correct, double* y_correct);
int LENS_subfunction_Find_CxCy_rulebase(std::vector<double> image_points1d, int th, int limit_low, int limit_high);
void ReadJson_LensCalSet(cv::String path_config, LENS_CAL_OBJECT* lens_obj, int** ptr);
void Lens_Convert(string imgFilepath_on, string imgFilepath_off, int lenscnt, float* int_diff);
void Lens_SetParameter(double* lensParams, PARAM_LENS* lenscal);
