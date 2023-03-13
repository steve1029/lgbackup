#include "stdafx.h"
#include <iostream>
#include "LSI_DataIO.h"
#include "LSI_Lens.h"
#include <queue>
#include <vector>
#include <algorithm>
#include "nlopt.hpp"
#include "math.h"
#include <time.h>
#include "subutil.h"


bool LENS_CalibrationMain(LENS_CAL_OBJECT* lens_obj, float* intensity_diff_image, float* gt)
{	
	// 반복 사용을 위한 변수 초기화
	Lens_Init();
	//Lens_Config_Parser("TOF_LSI_config.ini", lens_obj);
	// Calculate Physical LED Pattern(3d point) in meter unit
	LENS_CalculatePattern(lens_obj); 
	
	// make binary image by Thresholding apply at Intensity and Calc. LED Center point each led
	LENS_DetectPattern(lens_obj, intensity_diff_image);
		// projection object point3d to image Plane and matching each led between Captured LED and Physical LED pattern
	if (LENS_PatternMatching2Mask(lens_obj) == LGIT_OK){
		// Calculate lens param. using optimization library
		LENS_FitLensParams(lens_obj);
		// make Target plane g.t. for FPPN Cal.
		LENS_MakeGT(lens_obj, gt);

		if (abs(lens_obj->reprojection_error) > lens_obj->reprojection_error_max) {
			fprintf(stderr, "LENS_CalibrationMain : Led Matching Fail / Reproejction Error <%.3lf>\n", lens_obj->reprojection_error);
			return LGIT_ERROR;
		}
	}
	else {
		return LGIT_ERROR;
	}
	
	return LGIT_OK;
}

void LENS_CalculatePattern(LENS_CAL_OBJECT* lens_obj){
	// LED Pattern Calculation Physically, in meter unit, LED Center is Origin
	// Output : led pattern X,Y,Z Point3d 
	float space = lens_obj->pattern_spacing;
	float left_top = lens_obj->pattern_center[0] * -1.0f;
	
	int idx = 0;
	for (int h = 0; h < LEDMASK_LEN; h++) {
		for (int r = 0; r < LEDMASK_LEN; r++) {
			if (lens_obj->pattern_mask[h * LEDMASK_LEN + r] == 1) {
				// X, Y, Z + left top to left bottom scan order
				lens_obj->pattern.push_back(
					cv::Point3d(
						left_top + (r * space), 
						left_top + (h * space), 
						lens_obj->pattern_center[2]
					)
				);
			}
		}
	}
}

void LENS_DetectPattern(LENS_CAL_OBJECT* lens_obj, float* led_image) {
	//Detect led(blob) points and center point in each blobs from (led_on - led_off image)
	bool *logical_image = new bool[LSI_HGT * LSI_WID];
	short *blob_image = new short[LSI_HGT * LSI_WID];
	bool *isVisited = new bool[LSI_HGT * LSI_WID];
	short *pixel_num_per_blob = new short[LED_PATTERN_MAX];

	if (logical_image == NULL || blob_image == NULL || isVisited == NULL || pixel_num_per_blob == NULL) {
		fprintf(stderr, "LENS_FitLensParams : Memory Alloc Error\n");
		return;
	}
		
	int blob_found = 0;
	int final_blob_num = 0;
	float threshold_noise = lens_obj->spot_detector.noise_threshold;
	std::vector<std::pair<int, int>> pixel_location_per_blob[LED_PATTERN_MAX];

	// Make Logical Image
	for (int h = 0; h < LSI_HGT; h++) {
		for (int w = 0; w < LSI_WID; w++) {
			int idx = h * LSI_WID + w;
			if (led_image[idx] > threshold_noise) {
				logical_image[idx] = true;
			}
			else {
				logical_image[idx] = false;
			}
			// initialization blob image
			blob_image[idx] = 0;
			isVisited[idx] = false;
		}
	}
	Mat temp_led_image = toMat(led_image, LSI_HGT, LSI_WID);
	Mat temp_logical_image = toMat(logical_image, LSI_HGT, LSI_WID);
	
	
	imwrite("..\\_out\\led_img.png", autoMaximizedMat(temp_led_image));
	imwrite("..\\_out\\logical_img.png", autoMaximizedMat(temp_logical_image));


	// LED floodfiil using BFS algorithm
	for (int h = 0; h < LSI_HGT; h++) {
		for (int w = 0; w < LSI_WID; w++) {
			int idx = h * LSI_WID + w;
			if (logical_image[idx] && (!isVisited[idx])) {
				++blob_found;
				pixel_location_per_blob[blob_found] 
					= LENS_subfuntion_Blob_floodFill(
					logical_image, 
					blob_image, 
					pixel_num_per_blob, 
					isVisited, 
					h, w, 
					&blob_found);

				// eraze blob when the blob size too small
				if (pixel_location_per_blob[blob_found].size() < MIN_BLOB_SIZE) {
					blob_found--;
				}
			}
		}
	}
	Mat temp_blob_image = toMat(blob_image, LSI_HGT, LSI_WID);
	Mat temp_isVisited = toMat(isVisited, LSI_HGT, LSI_WID);

	lens_obj->spot_detector.detected_blob_num = blob_found;

	// Calculate Center Location per Blob using weighted sum
	double reweighet_border[3] = { -1.0, 0.0, 1.0 };

	for (int idx = 1; idx <= blob_found; idx++) {
		std::vector<std::pair<int, int>> v_temp = pixel_location_per_blob[idx];
		double pixel_loc_x = 0.0;
		double pixel_loc_y = 0.0;
		double sum_x = 0.0;
		double sum_y = 0.0;
		double sum_image = 0.0; 
		double y_range[3] = { 0.0, 0.0, 0.0 };
		double x_range[3] = { 0.0, 0.0, 0.0 };
			
		for (int v_idx = 0; v_idx < v_temp.size(); v_idx++) {
			int y = v_temp[v_idx].first;
			int x = v_temp[v_idx].second;

			sum_image += led_image[y * LSI_WID + x];
			sum_x += x * led_image[y * LSI_WID + x];
			sum_y += y * led_image[y * LSI_WID + x];
		}
		pixel_loc_x = sum_x / sum_image;
		pixel_loc_y = sum_y / sum_image;

		bool addedBorder = false;
		for (int r_idx = 0; r_idx < 3; r_idx++) {
			bool isPresent = false;
			y_range[r_idx] = round(pixel_loc_y + lens_obj->spot_detector.reweight_border[r_idx]);
			x_range[r_idx] = round(pixel_loc_x + lens_obj->spot_detector.reweight_border[r_idx]);

			for (int v_idx = 0; v_idx < v_temp.size(); v_idx++) {
				if( (v_temp[v_idx].first == y_range[r_idx]) && (v_temp[v_idx].second == x_range[r_idx]) ) {
					isPresent = true;
				}
			}
			if (!isPresent) {
				v_temp.push_back(std::make_pair(y_range[r_idx], x_range[r_idx]));
				addedBorder = true;
			}
		}

		if((y_range[0]) < 0 || (y_range[2] >= LSI_HGT) || (x_range[0]) < 0 || (x_range[2] >= LSI_WID)) {
			continue;
		}
		else if (!addedBorder) {
			lens_obj->spot_detector.pixel_pos.push_back(cv::Point2d(pixel_loc_x, pixel_loc_y));
			final_blob_num++;
		}
		else {
			sum_image = 0.0;
			sum_x = 0.0;
			sum_y = 0.0;

			for (int v_idx = 0; v_idx < v_temp.size(); v_idx++) {
				int y = v_temp[v_idx].first;
				int x = v_temp[v_idx].second;

				sum_image += led_image[y * LSI_WID + x];
				sum_x += x * led_image[y * LSI_WID + x];
				sum_y += y * led_image[y * LSI_WID + x];
			}
			pixel_loc_x = sum_x / sum_image;
			pixel_loc_y = sum_y / sum_image;
			lens_obj->spot_detector.pixel_pos.push_back(cv::Point2d(pixel_loc_x, pixel_loc_y));
			final_blob_num++;
		}
	}
	
	delete[] logical_image;
	delete[] blob_image;
	delete[] isVisited;
	delete[] pixel_num_per_blob;
}

cv::Mat rotation_vector(3, 1, cv::DataType<double>::type);
cv::Mat translation_vector(3, 1, cv::DataType<double>::type);
cv::Mat camera_matrix(3, 3, cv::DataType<double>::type);
cv::Mat distortion_Coeffs(5, 1, cv::DataType<double>::type);
std::vector<cv::Point2d> image_points;
std::vector<double> image_x;
std::vector<double> image_y;
std::vector<cv::Point2d> spattern_pos;
std::vector<std::tuple<double, double, int>> apattern_pos;
std::vector<std::pair<double, int>> diff_from_center;

void Lens_Init(void) {
	// var. init 
	image_points.clear();
	image_x.clear();
	image_y.clear();
	spattern_pos.clear();
	apattern_pos.clear();
	diff_from_center.clear();
}

bool LENS_PatternMatching2Mask(LENS_CAL_OBJECT* lens_obj) {
	//Matching between Physical LED Patten Point and measure image Point
	int num_of_detected_point = lens_obj->spot_detector.pixel_pos.size();
	
	if (num_of_detected_point != lens_obj->min_led_num) {
		fprintf(stderr, "LENS_PatternMatching2Mask : Detected LED Pattern Number less than minimum\n");
		fprintf(stderr, "-- Current Setting Led Num should be detected <%d> but detected <%d> \n",lens_obj->min_led_num, num_of_detected_point);
		//return LGIT_ERROR;
	}

	// Make Matching Information Physical LED Points & Detected LED Point each Point
	rotation_vector.at<double>(0) = 0.0;
	rotation_vector.at<double>(1) = 0.0;
	rotation_vector.at<double>(2) = 0.0;
	
	translation_vector.at<double>(0) = 0.0;
	translation_vector.at<double>(1) = 0.0;
	translation_vector.at<double>(2) = 0.0;
	
	camera_matrix.at<double>(0, 0) = lens_obj->lens_params[0];
	camera_matrix.at<double>(1, 0) = 0.0;
	camera_matrix.at<double>(2, 0) = 0.0;

	camera_matrix.at<double>(0, 1) = 0.0;
	camera_matrix.at<double>(1, 1) = lens_obj->lens_params[1];
	camera_matrix.at<double>(2, 1) = 0.0;

	camera_matrix.at<double>(0, 2) = LSI_WID - 0.5;
	camera_matrix.at<double>(1, 2) = LSI_HGT - 0.5;
	camera_matrix.at<double>(2, 2) = 1.0;
	
	distortion_Coeffs.at<double>(0) = lens_obj->lens_params[4];
	distortion_Coeffs.at<double>(1) = lens_obj->lens_params[5];
	distortion_Coeffs.at<double>(2) = lens_obj->lens_params[6];
	distortion_Coeffs.at<double>(3) = lens_obj->lens_params[7];
	distortion_Coeffs.at<double>(4) = lens_obj->lens_params[8];

	// LED Pattern Object Point Projection
	// cv::projectPoints(lens_obj->pattern, rotation_vector, translation_vector, camera_matrix, distortion_Coeffs, image_points);

	for (int idx = 0; idx < num_of_detected_point; idx++) {
		image_x.push_back(lens_obj->spot_detector.pixel_pos[idx].x);
		image_y.push_back(lens_obj->spot_detector.pixel_pos[idx].y);
	}
	int cx = LENS_subfunction_Find_CxCy_rulebase(image_x, 20, lens_obj->cx_lim[0], lens_obj->cx_lim[1]);
	int cy = LENS_subfunction_Find_CxCy_rulebase(image_y, 20, lens_obj->cy_lim[0], lens_obj->cy_lim[1]);

	bool cx_cond = (cx < lens_obj->cx_lim[0]) || (cx > lens_obj->cx_lim[1]);
	bool cy_cond = (cy < lens_obj->cy_lim[0]) || (cy > lens_obj->cy_lim[1]);

	if (cx_cond || cy_cond) {
		fprintf(stderr, "LENS_PatternMatching2Mask : cx & cy matching Fail\n");
		return LGIT_ERROR;
	}

	camera_matrix.at<double>(0, 2) = cx * 1.0;
	camera_matrix.at<double>(1, 2) = cy * 1.0;

	cv::projectPoints(lens_obj->pattern, rotation_vector, translation_vector, camera_matrix, distortion_Coeffs, image_points);


	int image_point_num = image_points.size();

	for (int idx = 0; idx < image_point_num; idx++) {
		// projected image point with index vector
		apattern_pos.push_back(std::make_tuple(image_points[idx].x, image_points[idx].y, idx));
		// initialization
		spattern_pos.push_back(cv::Point2d(-1.0, -1.0)); 
	}
	
	for (int idx = 0; idx < num_of_detected_point; idx++) {
		double x_pos = lens_obj->spot_detector.pixel_pos[idx].x - cx;
		double y_pos = lens_obj->spot_detector.pixel_pos[idx].y - cy;
		diff_from_center.push_back(std::make_pair(sqrt((x_pos * x_pos) + (y_pos * y_pos)), idx));
	}

	// sorting with index
	sort(diff_from_center.begin(), diff_from_center.end());

	for (int idx = 0; idx < num_of_detected_point; idx++) {
		double min_val = 100000.0;
		int min_idx = 0;
		// Calc Distance All Points
		int sorted_led_idx = diff_from_center[idx].second;
		int remain_pos_size = apattern_pos.size();
		
		double x_pos = lens_obj->spot_detector.pixel_pos[sorted_led_idx].x;
		double y_pos = lens_obj->spot_detector.pixel_pos[sorted_led_idx].y;

		for (int led_idx = 0; led_idx < remain_pos_size; led_idx++) {
			double dx = x_pos - std::get<0>(apattern_pos[led_idx]);
			double dy = y_pos - std::get<1>(apattern_pos[led_idx]);
			double dR = sqrt((dx * dx) + (dy * dy));

			if (led_idx == 0) {
				min_val = dR;
				min_idx = 0;
			}
			else if (dR < min_val) {
				min_val = dR;
				min_idx = led_idx;
			}
		}

		int led_idx_org = std::get<2>(apattern_pos[min_idx]);
		spattern_pos[led_idx_org].x = x_pos;
		spattern_pos[led_idx_org].y = y_pos;

		apattern_pos.erase(apattern_pos.begin() + min_idx);
	}

	lens_obj->spot_detector.pixel_pos.assign(spattern_pos.begin(), spattern_pos.end());

	return LGIT_OK;
}

void LENS_FitLensParams(LENS_CAL_OBJECT* lens_obj) {
	// Calculate Lens Parameter using nlopt optimization library
	PROJECTION_DATA* d = new PROJECTION_DATA;
	if (d == NULL) {
		fprintf(stderr, "LENS_FitLensParams : Memory Alloc Error\n");
		return;
	}

	double min_projetion_error;
	std::vector<double> x;

	x.push_back(lens_obj->lens_params[0]); // fx
	x.push_back(lens_obj->lens_params[2]); // cx
	x.push_back(lens_obj->lens_params[3]); // cy
	x.push_back(lens_obj->lens_params[4]); // k1
	x.push_back(lens_obj->lens_params[5]); // k2
	x.push_back(lens_obj->lens_params[6]); // p1
	x.push_back(lens_obj->lens_params[7]); // p2
	x.push_back(0.0); // rotation vector 0
	x.push_back(0.0); // rotation vector 1
	x.push_back(0.0); // rotation vector 2
	
	// valid image point insert, except < -1, -1 >
	d->imagePoints.clear();
	d->pr_imagePoints.clear();
	d->objectPoints.clear();

	for (int idx = 0; idx < lens_obj->spot_detector.pixel_pos.size(); idx++) {
		double x = lens_obj->spot_detector.pixel_pos[idx].x;
		double y = lens_obj->spot_detector.pixel_pos[idx].y;
		if (x > 0) {
			d->imagePoints.push_back(cv::Point2d(x, y));
			d->objectPoints.push_back(cv::Point3d(lens_obj->pattern[idx].x, lens_obj->pattern[idx].y, lens_obj->pattern[idx].z));
		}
	}
	
	// Run optimization
	nlopt::opt opt(nlopt::LN_NELDERMEAD, 10);
	opt.set_min_objective(LENS_subfunction_Calc_Error_Polynomial, d);
	try {
		nlopt::result opt_result = opt.optimize(x, min_projetion_error);
	}
	catch (std::exception &e) {
		fprintf(stderr, "NLopt Error : %s \n", e.what());
	}

	// Set Optimzed Params
	lens_obj->lens_params[0] = x[0];
	for (int idx = 1; idx < 8; idx++) {
		lens_obj->lens_params[idx] = x[idx - 1];
	}
	for (int idx = 0; idx < 3; idx++) {
		lens_obj->rVec[idx] = x[idx + 7];
	}
	lens_obj->projection_error = min_projetion_error;
	lens_obj->pr_imagePoints.assign(d->pr_imagePoints.begin(), d->pr_imagePoints.end());

	// Calc Min Projection Error
	lens_obj->reprojection_error = LENS_subfunction_Calc_Reprojection_Error(x, d);

	delete d;
}

void LENS_MakeGT(LENS_CAL_OBJECT* lens_obj, float* gt) {
	// make target plane image using lens parmas and physical inforatiom of led pattern for fppn
	double* x_correct = new double[LSI_SIZE];
	double* y_correct = new double[LSI_SIZE];
	double* x_global = new double[LSI_SIZE];
	double* y_global = new double[LSI_SIZE];
	double* z_global = new double[LSI_SIZE];
	double* RLensZ = new double[LSI_SIZE];

	if (x_correct == NULL || y_correct == NULL || x_global == NULL || y_global == NULL || z_global == NULL || RLensZ == NULL) {
		fprintf(stderr, "LENS_MakeGT : Memory Alloc Error\n");
		return;
	}

	float target_dist = lens_obj->pattern_center[2];

	cv::Mat rotation_vector(3, 1, cv::DataType<double>::type);
	rotation_vector.at<double>(0) = lens_obj->rVec[0] * -1.0;
	rotation_vector.at<double>(1) = lens_obj->rVec[1] * -1.0;
	rotation_vector.at<double>(2) = lens_obj->rVec[2] * -1.0;

	cv::Mat R(3, 3, cv::DataType<double>::type);
	cv::Rodrigues(rotation_vector, R);

	LENS_Calc_Undistortion_Points(lens_obj->lens_params, x_correct, y_correct);

	for (int row = 0; row < LSI_HGT; row++) {
		for (int col = 0; col < LSI_WID; col++) {
			int idx = row * LSI_WID + col;
			double len = sqrt(x_correct[idx] * x_correct[idx] + y_correct[idx] * y_correct[idx] + 1);
			x_global[idx] = x_correct[idx] / len;
			y_global[idx] = y_correct[idx] / len;
			z_global[idx] = sqrt(1 - (x_global[idx] * x_global[idx] + y_global[idx] * y_global[idx]));
			RLensZ[idx] = (R.at<double>(2, 0) * x_global[idx]) + (R.at<double>(2, 1) * y_global[idx]) + (R.at<double>(2, 2) * z_global[idx]);
			gt[idx] = target_dist / RLensZ[idx];
		}
	}

	delete[] x_correct;
	delete[] y_correct;

	delete[] x_global;
	delete[] y_global;
	delete[] z_global;
	delete[] RLensZ;
}

void LENS_Calc_Undistortion_Points(double* lens_params, double* x_correct, double* y_correct) {
	// make distortion information map using Lens parameter
	double fx = lens_params[0];
	double fy = lens_params[1];
	double ifx = 1.0 / fx;
	double ify = 1.0 / fy;
	double cx = lens_params[2];
	double cy = lens_params[3];
	double error_th = 0.00000001;

	double* x_org = new double[LSI_WID];
	double* y_org = new double[LSI_HGT];

	if (x_org == NULL || y_org == NULL) {
		fprintf(stderr, "LENS_Calc_Undistortion_Points : Memory Alloc Error\n");
		return;
	}

	double k[5] = { 0.0, lens_params[4], lens_params[5], lens_params[6],lens_params[7] };

	for (int idx = 0; idx < LSI_WID; idx++) {
		x_org[idx] = (idx - cx) * ifx;
		if (idx < LSI_HGT) {
			y_org[idx] = (idx - cy) * ify;
		}
	}
	for (int row = 0; row < LSI_HGT; row++) {
		std::copy(x_org, x_org + LSI_WID, &x_correct[row * LSI_WID]);
	}
	for (int row = 0; row < LSI_HGT; row++) {
		std::fill_n(y_correct + (row * LSI_WID), LSI_WID, y_org[row]);
	}

	for (int iter = 0; iter < 100; iter++) {
		for (int row = 0; row < LSI_HGT; row++) {
			for (int col = 0; col < LSI_WID; col++) {

				int idx = row * LSI_WID + col;
				double xx = x_correct[col] * x_correct[col];
				double xy = x_correct[col] * y_correct[row];
				double yy = y_correct[row] * y_correct[row];

				double r2 = xx + yy;
				double r4 = r2 * r2;

				double icdist = 1 / (1 + k[2] * r4 + k[1] * r2); // Radial 
				double deltaX = 2 * k[3] * xy + k[4] * (r2 + 2 * xx);  // Tangential
				double deltaY = k[3] * (r2 + 2 * yy) + 2 * k[4] * xy;

				x_correct[idx] = (x_org[col] - deltaX) * icdist;
				y_correct[idx] = (y_org[row] - deltaY) * icdist;
			}
		}
	}
	delete[] x_org;
	delete[] y_org;
}


// NLopt optimization object function, should be of the form : (const std::vector`<double>` &x, std::vector`<double>` &grad, void* f_data);
double LENS_subfunction_Calc_Error_Polynomial(const std::vector<double> &x, std::vector<double> &grad, void* my_func_data){
	// NLopt Lib. Optimization Target Function
	PROJECTION_DATA* d = (PROJECTION_DATA *)my_func_data;
	
	cv::Mat camera_matrix(3, 3, cv::DataType<double>::type);
	camera_matrix.at<double>(0, 0) = x[0]; // fx
	camera_matrix.at<double>(1, 0) = 0.0;
	camera_matrix.at<double>(2, 0) = 0.0;

	camera_matrix.at<double>(0, 1) = 0.0;
	camera_matrix.at<double>(1, 1) = x[0]; // fy == fx
	camera_matrix.at<double>(2, 1) = 0.0;

	camera_matrix.at<double>(0, 2) = x[1]; // cx
	camera_matrix.at<double>(1, 2) = x[2]; // cy
	camera_matrix.at<double>(2, 2) = 1.0;

	cv::Mat distortion_Coeffs(5, 1, cv::DataType<double>::type);
	distortion_Coeffs.at<double>(0) = x[3];
	distortion_Coeffs.at<double>(1) = x[4];
	distortion_Coeffs.at<double>(2) = x[5];
	distortion_Coeffs.at<double>(3) = x[6];
	distortion_Coeffs.at<double>(4) = 0.0;
	
	cv::Mat rotation_vector(3, 1, cv::DataType<double>::type);
	rotation_vector.at<double>(0) = x[7];
	rotation_vector.at<double>(1) = x[8];
	rotation_vector.at<double>(2) = x[9];

	cv::Mat translation_vector(3, 1, cv::DataType<double>::type);
	translation_vector.at<double>(0) = 0.0;
	translation_vector.at<double>(1) = 0.0;
	translation_vector.at<double>(2) = 0.0;
	// Projection
	cv::projectPoints(d->objectPoints, rotation_vector, translation_vector, camera_matrix, distortion_Coeffs, d->pr_imagePoints);
	
	// Calc Projection Error 
	double projection_error = 0.0;
	double sum = 0.0;
	int num_of_points = d->pr_imagePoints.size();

	for (int idx = 0; idx < num_of_points; idx++) {
		double x_diff = d->pr_imagePoints[idx].x - d->imagePoints[idx].x;
		double y_diff = d->pr_imagePoints[idx].y - d->imagePoints[idx].y;
		sum += (x_diff * x_diff) + (y_diff * y_diff);
	}
	sum = sum / num_of_points;
	projection_error = sqrt(sum);
	return projection_error;
}

double LENS_subfunction_Calc_Reprojection_Error(const std::vector<double>& x, PROJECTION_DATA* d) {
	// Calcluate min reprojection error using final lens parameters
	cv::Mat camera_matrix(3, 3, cv::DataType<double>::type);
	camera_matrix.at<double>(0, 0) = x[0]; // fx
	camera_matrix.at<double>(1, 0) = 0.0;
	camera_matrix.at<double>(2, 0) = 0.0;

	camera_matrix.at<double>(0, 1) = 0.0;
	camera_matrix.at<double>(1, 1) = x[0]; // fy == fx
	camera_matrix.at<double>(2, 1) = 0.0;

	camera_matrix.at<double>(0, 2) = x[1]; // cx
	camera_matrix.at<double>(1, 2) = x[2]; // cy
	camera_matrix.at<double>(2, 2) = 1.0;

	cv::Mat distortion_Coeffs(5, 1, cv::DataType<double>::type);
	distortion_Coeffs.at<double>(0) = x[3];
	distortion_Coeffs.at<double>(1) = x[4];
	distortion_Coeffs.at<double>(2) = x[5];
	distortion_Coeffs.at<double>(3) = x[6];
	distortion_Coeffs.at<double>(4) = 0.0;
	
	cv::Mat rotation_vector(3, 1, cv::DataType<double>::type);
	cv::Mat translation_vector(3, 1, cv::DataType<double>::type);

	// Re Projection
	cv::solvePnP(d->objectPoints, d->imagePoints, camera_matrix, distortion_Coeffs, rotation_vector, translation_vector);
	cv::projectPoints(d->objectPoints, rotation_vector, translation_vector, camera_matrix, distortion_Coeffs, d->pr_imagePoints);

	// Calc Projection Error 
	double re_projection_error = 0.0;
	double sum = 0.0;
	int num_of_points = d->pr_imagePoints.size();

	for (int idx = 0; idx < num_of_points; idx++) {
		double x_diff = d->pr_imagePoints[idx].x - d->imagePoints[idx].x;
		double y_diff = d->pr_imagePoints[idx].y - d->imagePoints[idx].y;
		sum += (x_diff * x_diff) + (y_diff * y_diff);
	}
	sum = sum / num_of_points;
	re_projection_error = sqrt(sum); 
	return re_projection_error;
}


std::vector<std::pair<int, int>> LENS_subfuntion_Blob_floodFill(
	bool *logical_image, short *blob_image, short* pixel_num_blob, bool *isVisited, int arg_h, int arg_w, int* blob_count) {
	// binary image to blob image using BFS algorithm
	std::queue<std::pair<int, int>> q;
	std::vector<std::pair<int, int>> v_temp;

	int dx[4] = {-1, 0, 1, 0};
	int dy[4] = { 0,-1, 0, 1};
	int blob_number = *blob_count;
	pixel_num_blob[blob_number] = 0;

	q.push(std::make_pair(arg_h, arg_w));
	v_temp.push_back(std::make_pair(arg_h, arg_w));
	isVisited[arg_h * LSI_WID + arg_w] = true;
	
	// flood fill Algortihm using BFS
	while (!q.empty()) {
		
		std::pair<int, int> pt = q.front();
		q.pop();

		int y = pt.first;
		int x = pt.second;

		for (int i = 0; i < 4; i++) {
			int ny = dy[i] + y;
			int nx = dx[i] + x;
			
			bool location_condition = (nx >= 0) && (ny >= 0) && (nx < LSI_WID) && (ny < LSI_HGT);
				
			if (location_condition && (!isVisited[ny * LSI_WID + nx])) {
				if (logical_image[ny * LSI_WID + nx]) {
					isVisited[ny * LSI_WID + nx] = true;
					pixel_num_blob[blob_number]++;
					q.push(std::make_pair(ny, nx));
					v_temp.push_back(std::make_pair(ny, nx));
					blob_image[ny * LSI_WID + nx] = blob_number;
				}
			}
		}
	}

	// eraze blob when the blob size too small
	if (v_temp.size() < MIN_BLOB_SIZE) {
		for (int idx = 0; idx < v_temp.size(); idx++) {

			std::pair<int, int> pt = v_temp[idx];

			int y = pt.first;
			int x = pt.second;
			
			blob_image[y * LSI_WID + x] = 0;
			logical_image[y * LSI_WID + x] = false;
		}
	}

	return v_temp;
}

int LENS_subfunction_Find_CxCy_rulebase(std::vector<double> image_points1d, int th, int limit_low, int limit_high) {
	// temporary cx cy for calculate led pattern matching
	cv::Point2i max_point;
	int points_num = image_points1d.size();
	
	std::vector<double> points;
	std::vector<std::pair<int, double>> bin;
	
	points.assign(image_points1d.begin(), image_points1d.end());
	sort(points.begin(), points.end());
	
	bin.push_back(std::make_pair(1, points[0]));

	int idx_x = 0;
	
	for (int idx = 1; idx < points_num; idx++) {
		if ((points[idx] - points[idx - 1]) <= th) {
			bin[idx_x].second += points[idx];
			bin[idx_x].first++;
		}
		else {
			bin[idx_x].second /= bin[idx_x].first;
			idx_x++;
			bin.push_back(std::make_pair(1, points[idx]));
		}
	}

	sort(bin.begin(), bin.end());
	int result = -1;

	for (int idx = 0; idx < bin.size(); idx++) {
		double c_temp = bin[idx].second;
		if ( (limit_low < c_temp) && (limit_high > c_temp)) {
			result = (int)round(bin[idx].second);
			break;
		}
	}
	return result;
}

void ReadJson_LensCalSet(cv::String path_config, LENS_CAL_OBJECT* lens_obj, int **ptr)
{
	cv::FileStorage fsRead(path_config, cv::FileStorage::READ);
	
	if (fsRead.isOpened())
	{
		vector<float> data_float; 
		vector<int> data_int;
		vector<double> data_double;
		FileNode val = fsRead["LensCal_Setting_Params"];

		data_int.clear(); val["pattern_len"] >> data_int;
		lens_obj->pattern_len[0] = data_int[0];
		lens_obj->pattern_len[1] = data_int[1];
		
		data_int.clear();  val["pattern_mask"] >> data_int;
		*ptr = new int[data_int.size()];
		for (int idx = 0; idx < data_int.size(); idx++) {
			lens_obj->pattern_mask[idx] = data_int[idx];
		}
		
		data_float.clear();  val["pattern_spacing"] >> data_float;
		lens_obj->pattern_spacing = data_float[0];
		
		data_float.clear();  val["pattern_center"] >> data_float;
		for (int idx = 0; idx < data_float.size(); idx++) {
			lens_obj->pattern_center[idx] = data_float[idx];
		}

		data_int.clear();  val["noise_threshold"] >> data_int;
		lens_obj->spot_detector.noise_threshold = data_int[0];
		
		data_float.clear();  val["lens_parameter"] >> data_float;
		for (int idx = 0; idx < data_float.size(); idx++) {
			lens_obj->lens_params[idx] = data_float[idx];
		}
		
		data_int.clear();  val["cx_limits"] >> data_int;
		for (int idx = 0; idx < data_int.size(); idx++) {
			lens_obj->cx_lim[idx] = data_int[idx];
		}
		
		data_int.clear();  val["cy_limits"] >> data_int;
		for (int idx = 0; idx < data_int.size(); idx++) {
			lens_obj->cy_lim[idx] = data_int[idx];
		}

		data_int.clear();  val["Fov_H"] >> data_int;
		lens_obj->fov_h = data_int[0];

		data_int.clear();  val["Fov_V"] >> data_int;
		lens_obj->fov_v = data_int[0];

		data_int.clear();  val["MIN_LED_NUM"] >> data_int;
		lens_obj->min_led_num = data_int[0];

		data_double.clear();  val["reprojection_max"] >> data_double;
		lens_obj->reprojection_error_max = data_double[0];
	}

	fsRead.release();
}

void Lens_eeprom(int* fov, double* lensParams, PARAM_LENS* lenscal)
{
	// Lens Params write to eeprom
	UINT32 sum = 0;
	lenscal->lut_size = 15;

	for (int idx = 0; idx < 14; idx++)
		lenscal->padding1[idx] = 0xff;

	lenscal->FOV_hor = fov[0];
	lenscal->FOV_ver = fov[1];
	lenscal->lens_cc_x = (INT32)(lensParams[2] * 100000);
	lenscal->lens_cc_y = (INT32)(lensParams[3] * 100000);
	lenscal->lens_f_length_x = (INT32)(lensParams[0] * 100000);
	lenscal->lens_f_length_y = (INT32)(lensParams[1] * 100000);
	lenscal->lens_kc[0] = (INT32)(lensParams[4] * 100000000);
	lenscal->lens_kc[1] = (INT32)(lensParams[5] * 100000000);
	lenscal->lens_kc[2] = (INT32)(lensParams[6] * 100000000);
	lenscal->lens_kc[3] = (INT32)(lensParams[7] * 100000000);
	lenscal->lens_kc[4] = (INT32)(lensParams[8] * 100000000);
	lenscal->new_lens_f_length_x = (INT32)(lensParams[2] * 100000);
	lenscal->new_lens_f_length_y = (INT32)(lensParams[3] * 100000);
	lenscal->new_lens_cc_x = (INT32)(lensParams[0] * 100000);
	lenscal->new_lens_cc_y = (INT32)(lensParams[1] * 100000);

	sum = sumParams(lenscal->FOV_hor, sum);
	sum = sumParams(lenscal->FOV_ver, sum);
	sum = sumParams(lenscal->lens_cc_x, sum);
	sum = sumParams(lenscal->lens_cc_y, sum);
	sum = sumParams(lenscal->lens_f_length_x, sum);
	sum = sumParams(lenscal->lens_f_length_y, sum);
	sum = sumParams(lenscal->lens_kc[0], sum);
	sum = sumParams(lenscal->lens_kc[1], sum);
	sum = sumParams(lenscal->lens_kc[2], sum);
	sum = sumParams(lenscal->lens_kc[3], sum);
	sum = sumParams(lenscal->lens_kc[4], sum);
	sum = sumParams(lenscal->new_lens_f_length_x, sum);
	sum = sumParams(lenscal->new_lens_f_length_y, sum);
	sum = sumParams(lenscal->new_lens_cc_x, sum);
	sum = sumParams(lenscal->new_lens_cc_y, sum);

	for (int idx = 0; idx < 179; idx++)
		lenscal->padding2[idx] = 0xff;

	lenscal->lensChecksum = (UINT8)((sum % 255) + 1);
}

UINT32 sumParams(INT32 data, INT32 sum)
{	// Checksum Calculated function
	UINT8 temp[4];
	temp[0] = data >> 24;
	temp[1] = data >> 16;
	temp[2] = data >> 8;
	temp[3] = data >> 0;

	for (int i = 0; i < 4; i++) {
		sum += temp[i];

		if (sum > 255) {
			sum -= 256;
		}
	}

	return sum;
}
