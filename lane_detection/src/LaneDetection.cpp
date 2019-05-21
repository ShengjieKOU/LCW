#pragma warning(disable: 4819)
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <algorithm>

#include "LaneDetection.h"
// Lane marking definition
#define MAX_LANE_MARKING 2000
int MAX_LW_N = 32;		 // Max lane width, nearest
int MAX_LW_F = 5;		 // Max lane width, farest
#define MAX_LW_D 0		 //20/2		// Max lane width, delta, lane marking can be wider if it is too left or right
int MIN_LW_N = 15;		 // Min lane width, nearest
int MIN_LW_F = 1;		 // Min lane width, farest
int tempLW_F;
int tempLW_N;
#define SCAN_INTERVAL1 1 //lower
#define SCAN_INTERVAL2 1 //upper

#define KALMAN_THRES 50
//#define KALMAN_THRES 25    //identification threshold for kalman filter

//VP DEFAULT @chenqi & jxy
#define VP_X 384		   //VP_X
#define VP_Y 235		   //VP_Y
#define VP_X_DELTA_NUM 12  //originally VP_X_DELTA, 6*24, cabin number by x. TODO: find a more robust method
#define VP_Y_DELTA_NUM 3   //originally VP_Y_DELTA, 3*24, cabin number by y.
#define VP_WINDOW 3		   //half of the vp vote cabin size
#define WINDOW_EXPANSION 2 //expand small voting rectangle by this

// Lane Marking Grouping
#define MAX_LANE_SEED 200
#define SEED_MARKING_DIST_THRES 10
#define VALID_SEED_MARKING_NUMBER_THRES 6 // by jxy as a test, default is 6
#define LOW_LEVEL_ASS_THRES 1.95		   // by @wentuopu change threshold for low association

#define DASH_SCORE_THRES 0.4		   // by @chenqi validate the score of dash

int image_num = 1;
bool flag_initial = 0; //by jxy kalman initial flag
cv::VideoWriter kalman_video;
int edge_set_flag = 1;

//------------------@xiebo--2018.04.27---------------------
struct LANE_COEFF {
	std::vector<float> now;
	std::vector<float> minus1;
	std::vector<float> minus2; //meaning see Matlab. coeff of lanes in current and past images.
};

//int image_num = 131;
int image_num_initial = image_num;
cv::Mat Xm;//输出车道线结果（卡尔曼估计值）
cv::Mat R;
cv::Mat Q;
cv::Mat Phi;
cv::Mat H;
cv::Mat Pm;//各矩阵意义均见标准卡尔曼滤波
int num_Xm; //当前车道线数量
std::vector<int> bbb;
cv::Mat numempty;
cv::Mat lane_g_id;

cv::Mat lane_g_style;

LANE_COEFF aaa;
std::vector<int> aaamarking;

cv::Mat vp_candidate_x(VP_Y_DELTA_NUM * 2, VP_X_DELTA_NUM * 2, CV_32FC1);
cv::Mat vp_candidate_y(VP_Y_DELTA_NUM * 2, VP_X_DELTA_NUM * 2, CV_32FC1);
cv::Mat vp_countlines(VP_Y_DELTA_NUM * 2, VP_X_DELTA_NUM * 2, CV_32FC1);
float VP_X_DELTA = VP_X_DELTA_NUM * VP_WINDOW * 2; //big rectangle
float VP_Y_DELTA = VP_Y_DELTA_NUM * VP_WINDOW * 2;
cv::Point2f last_vp;

void LaneDetection::setcount(int maincount){
	image_num = maincount;
}

// mat calc(mat A, mat b)
// {
// 	mat x = zeros<mat>(2, 1);
// 	solve(x, A, b);
// 	return x;
// }

void  LaneDetection::init_id_arr() {
	for (int i = 1; i < 25; i++) {
		id_arr.push_back(i);
	}
}
int  LaneDetection::get_id() {
	if (id_arr.size() > 0) {
		int id = id_arr[0];
		id_arr.erase(id_arr.begin());
		return id;
	}else{ 
		std::cout << "array not long enough" << std::endl;
		return -1;
	}
}

void LaneDetection::show_id_arr() {
	std::cout << "show id_arr " << std::endl;
	for (int ii = 0; ii < id_arr.size()-1; ii++) {
		std::cout << " "<<id_arr[ii];
	}

	std::cout << std::endl;

}

void  LaneDetection::check_id(int id) {
	id_arr.push_back(id);
}



float LaneDetection::dist_pt(cv::Point2f pt_1, cv::Point2f pt_2) {
	
	return sqrtf(pow(pt_1.x - pt_2.x,2)+pow(pt_1.y - pt_2.y,2));
}
// Initializing variables depending on the resolution of the input image.
double valueAt(std::vector<float>& f, float x) {
	float ans = 0.f;
	for (int i = (int)f.size() - 1; i >= 0; --i)
		ans = ans * x + f[i];
	return ans;
}

//----------------------2018.03.05-----------------------------

LaneDetection::LaneDetection()
{
	vp_pt.y = VP_Y;
	vp_pt.x = VP_X;
	//------------------------IPM----------------by chenqi--------------------------------------------
	// f_image_scale = 1;
	f_image_scale = 2.5;

	//f_world_scale = 3 * f_image_scale;
	f_world_scale =1;
	f_visual_offset = 384;
	//id generation init
	init_id_arr();
	// left y + right y- 
	// front x+ 
	// The 4-points at the input image	
	std::vector<cv::Point2f> origPoints;

	// origPoints.push_back(cv::Point2f(207 / f_image_scale, 1074 / f_image_scale));
	// origPoints.push_back(cv::Point2f(1676 / f_image_scale, 1074 / f_image_scale));
	// origPoints.push_back(cv::Point2f(728 / f_image_scale, 754 / f_image_scale));
	// origPoints.push_back(cv::Point2f(1325 / f_image_scale, 692 / f_image_scale));

	origPoints.push_back(cv::Point2f(754 / f_image_scale, 777 / f_image_scale));
	origPoints.push_back(cv::Point2f(1055 / f_image_scale, 787 / f_image_scale));
	//cal 1
	//origPoints.push_back(cv::Point2f(583 / f_image_scale, 369 / f_image_scale));
	//origPoints.push_back(cv::Point2f(101 / f_image_scale, 400 / f_image_scale));
	//cal
	origPoints.push_back(cv::Point2f(1245 / f_image_scale, 1074 / f_image_scale));
	origPoints.push_back(cv::Point2f(533 / f_image_scale, 1055 / f_image_scale));

	// The 4-points correspondences in the destination image
	std::vector<cv::Point2f> dstPoints;
	std::vector<cv::Point2f> dstPoints_visual;

	dstPoints_visual.push_back(cv::Point2f(121 / f_world_scale + f_visual_offset, (1.812+3.485)*100 / f_world_scale));
	dstPoints_visual.push_back(cv::Point2f(-119 / f_world_scale + f_visual_offset, (3.48+1.824)*100 / f_world_scale));
	dstPoints_visual.push_back(cv::Point2f(-221/ f_world_scale + f_visual_offset, 31.5 / f_world_scale));
	dstPoints_visual.push_back(cv::Point2f(216 / f_world_scale + f_visual_offset, 63 / f_world_scale));

	// dstPoints.push_back(cv::Point2f(2.2 / f_world_scale, 1.0 / f_world_scale));
	// dstPoints.push_back(cv::Point2f(-2.2 / f_world_scale , 1.0 / f_world_scale));
	// dstPoints.push_back(cv::Point2f(0.9 / f_world_scale , 5.4 / f_world_scale));
	// dstPoints.push_back(cv::Point2f(-3.5 / f_world_scale , 6.4 / f_world_scale));

	dstPoints.push_back(cv::Point2f(1.253 / f_world_scale, 6.72 / f_world_scale));
	dstPoints.push_back(cv::Point2f(-1.22 / f_world_scale, 6.72 / f_world_scale));
	//dstPoints.push_back(cv::Point2f(-2.36 / f_world_scale ,1.828/ f_world_scale)); // cal1
	//dstPoints.push_back(cv::Point2f(2.61 / f_world_scale, 1.815 / f_world_scale)); // cal1

	dstPoints.push_back(cv::Point2f(-1.22 / f_world_scale, 1.515 / f_world_scale));   // cal2
	dstPoints.push_back(cv::Point2f(1.253 / f_world_scale, 1.515 / f_world_scale)); // cal2

	//dstPoints.push_back(cv::Point2f(y , x));

	// IPM object

	ipm.init(cv::Size(768, 480), cv::Size(768, 480), origPoints, dstPoints);
	ipm_visual.init(cv::Size(768, 480), cv::Size(768, 480), origPoints, dstPoints_visual);
}

bool LaneDetection::initialize_variable(cv::Mat image) {

	// Image variable setting
	cv::Mat img_src = image.clone();
	if (img_src.empty()) {
		std::cout << "Err: Cannot find an input image for initialization: " << image << std::endl;
		return false;
	}

	img_size = img_src.size();    
	img_height = img_src.rows - 10; 
	img_width = img_src.cols;       
//	img_roi_height = (int)(img_size.height*4.4 / 12);
	img_roi_height = (int)(img_size.height*6.25/12);
	img_depth = img_src.depth();            

	max_lw.resize(img_height);
	min_lw.resize(img_height);
	max_lw_d.resize(img_width);

	// Estimated Lane Width
	for (int hh = img_roi_height; hh < img_height; ++hh) {
		max_lw[hh] = (int)((MAX_LW_N - MAX_LW_F)*(hh - img_roi_height) / (img_size.height - img_roi_height) + MAX_LW_F);
		
		min_lw[hh] = (int)((MIN_LW_N - MIN_LW_F)*(hh - img_roi_height) / (img_size.height - img_roi_height) + MIN_LW_F);
		
	}

	int w = img_width - 1;     //640-1
	while (img_width - 1 - w < w) {
		max_lw_d[w] = (int)(MAX_LW_D*(fabs(w - (img_width - 1) / 2.0)) / ((img_width - 1) / 2.0));
		                   //10*(|(w-319.5)|)/319.5
		max_lw_d[img_width - 1 - w] = (int)(MAX_LW_D*(fabs(w - (img_width - 1) / 2.0)) / ((img_width - 1) / 2.0));
		w--;
	} 
	//int vanish_point_y = img_roi_height ;//-40 000038
    //int vanish_point_x = img_width / 2 ;//- 71.5 00038.avi TEST2compress + 30, test_data - 10

	for (int hh = 0; hh < 3; ++hh) {
		bbb.push_back(0);
	}
	aaa.now.push_back(0);
	aaa.minus1.push_back(0);
	aaa.minus2.push_back(0);

	//final_video.open("result/final_video.avi", CV_FOURCC('M', 'J', 'P', 'G'), 20, img_size, true);
//	kalman_video.open("/home/chenqi/CRF5.5/result/kalman/kalman.avi", CV_FOURCC('D','I','V','X'), 20, img_size, true);

	for (int m = 0; m < VP_Y_DELTA_NUM * 2; m++)
	{
		for (int n = 0; n < VP_X_DELTA_NUM * 2; n++)
		{
			vp_candidate_x.at<float>(m, n) = VP_X - VP_X_DELTA + n * VP_WINDOW * 2 + VP_WINDOW;
			vp_candidate_y.at<float>(m, n) = VP_Y - VP_Y_DELTA + m * VP_WINDOW * 2 + VP_WINDOW;
			vp_countlines.at<float>(m, n) = 0;
		}
	}

	return true;
}


//----------------------2018.03.05--------------------------------
bool LaneDetection::initialize_Img(cv::Mat image) {

	// Loading an input image
	cv::Mat img_src = image.clone();
	img_clr = image.clone(); // by @wentuopu for save original image

	if (img_src.empty()) {
		std::cout << "Err: Cannot find the input image: " << image << std::endl;
		return false;
	}

	img_gray = cv::Mat(img_size, img_depth);
	//cv::equalizeHist(img_gray, img_gray);

	if (img_src.channels() == 1) 
	{
		img_src.copyTo(img_gray);                     
	}
	else {
		cv::cvtColor(img_src, img_gray, CV_BGR2GRAY);
	}

	// Variable initialization 
	lm.clear();
	marking_seed.resize(0);
	nodes.resize(0);
	edges.resize(0);
	return true;
}



void LaneDetection::lane_marking_detection(bool verbose) {

	//std::cout << image_num << std::endl;

	cv::Mat img_hsv;
	if(0){
	std::vector<cv::Mat> channel_bgr(3);
	cv::split(img_clr,channel_bgr);
	
	cv::equalizeHist(channel_bgr[0],channel_bgr[0]);
	cv::equalizeHist(channel_bgr[1],channel_bgr[1]);
	cv::equalizeHist(channel_bgr[2],channel_bgr[2]);

	cv::merge(channel_bgr,img_clr);
	}
	cv::cvtColor(img_clr, img_hsv, CV_BGR2HSV);
	// cv::Mat img_test_pixel = cv::Mat(img_size, CV_8UC3, cv::Scalar(0, 0, 0));
	// cv::imshow("Lane marking detection", img_test_pixel);
	// cv::waitKey(0);

	//------------------------@xiebo------gradient difference-------------
	for (int h = img_roi_height; h < img_height;) {
	// for (int h = 281; h < img_height;){
		//std::cout << "h: " << h << std::endl;

		// half size of the filter
        // by @wentuopu change the slope to adapt lane width
		int hf_size = 2 + 13 * (h - img_roi_height + 1) / (img_height - img_roi_height);

		std::vector<int> scan_line(img_width);

		for (int w = hf_size + 1; w < img_width - hf_size - 1; w++) {

			// left edge value, right edge value    
			int l_val = 0;
			int r_val = 0;

			for (int i = -hf_size; i<0; i++) {
				l_val = l_val + img_gray.at<uchar>(h, w + i);
			}
			for (int i = 1; i <= hf_size; i++) {
				r_val = r_val + img_gray.at<uchar>(h, w + i);
				// int temp = img_gray.at<uchar>(h, w + i);
				// std::cout << "img_gray " << h << " " << w << " pixel: " << temp << std::endl;
			}
			
			if (((float)(r_val - l_val) / (float)hf_size)*2 >marking_thres((float)l_val / (float)hf_size)) scan_line[w] = 1; // right edge = 1, gray level: white is small;
			if (((float)(l_val - r_val) / (float)hf_size)*2 >marking_thres((float)r_val / (float)hf_size)) scan_line[w] = -1; // left edge = -1;
		    
		}

		//The notorius bug of 282 lies here! jxy 0718
		int e_flag = 0; // edge flag
		for (int w = hf_size + 1; w < img_width - hf_size - 2; w++) {
			//std::cout << "w = " << w << " e_flag = " << e_flag << " scan_line = " << scan_line[w] << std::endl;
			if (scan_line[w] == 1) {
				if (e_flag >= 0) {
					e_flag++; //this w is a right climber.
				}
				else { //this w is a right cliff
					scan_line[w + (int)(e_flag / 2.0)+1] = -10; //the left pixels are climbing, but stopped now. so the center of the climbers should be recorded as left edge.
					//jxy 0718: e_flag means the number of climbing pixels. /2.0 means, regard the center of the climbers as the edge pixel.
					//std::cout << "right. memory execute:" << w - (int)(e_flag / 2.0) << std::endl;
					e_flag = 0;
				}
			}
			else if (scan_line[w] == -1) {
				if (e_flag <= 0) {
					e_flag--;
				}
				else {
					scan_line[w - (int)(e_flag / 2.0)+1] = 10; //e_flag is positive, so this memory execute is smaller than w, thus is safe.
					//std::cout << "left. memory execute:" << w + (int)(e_flag / 2.0) << std::endl;
					e_flag = 0;
				}
			}
			else {
				if (e_flag > 0) {
					scan_line[w - (int)(e_flag / 2.0)+1] = 10;
					//std::cout << "right. memory execute:" << w - (int)(e_flag / 2.0) << std::endl;
					e_flag = 0;
				}
				else if (e_flag < 0) {
					scan_line[w + (int)(e_flag / 2.0)+1] = -10;
					//std::cout << "left. memory execute:" << w + (int)(e_flag / 2.0) << std::endl;
					e_flag = 0;
				}
			}
		}

		// cv::imshow("Lane marking detection", img_test_pixel);
		// cv::waitKey(0);

		// Extracting Lane Markings - marking flag
		cv::Point2i l_pt, r_pt;
		int m_flag = 0;
		
		for (int w = hf_size + 1; w < img_width - hf_size - 1; w++) {
			
			// std::cout << "scan_line[" << w << "] = " << scan_line[w] << std::endl;
			// std::cout << "m_flag = " << m_flag << std::endl;
			if (scan_line[w] == 10) {
				m_flag = 1;
				l_pt.x = w;
				l_pt.y = h;
			}
			if (m_flag == 1) {
				if (scan_line[w] == -10) {
					m_flag = 2;
					r_pt.x = w;
					r_pt.y = h;
				}
			}
			if (m_flag == 2) {
				if (((r_pt.x - l_pt.x) >= min_lw[h]) && ((r_pt.x - l_pt.x) <= (max_lw[h] + max_lw_d[w]))) {

					// lane update
					LANE_MARKING lm_new;
					int Rsum=0,Gsum=0,Bsum=0;
					int Hsum=0,Ssum=0,Vsum=0;
					
					for(int x=l_pt.x;x<r_pt.x+1;x++){
						int Btemp = img_clr.at<cv::Vec3b>(h, x)[0];
						int Gtemp = img_clr.at<cv::Vec3b>(h, x)[1];
						int Rtemp = img_clr.at<cv::Vec3b>(h, x)[2];
						Bsum += Btemp;
						Gsum += Gtemp;
						Rsum += Rtemp;
						int Htemp = img_hsv.at<cv::Vec3b>(h, x)[0];
						int Stemp = img_hsv.at<cv::Vec3b>(h, x)[1];
						int Vtemp = img_hsv.at<cv::Vec3b>(h, x)[2];
						Hsum += Htemp;
						Ssum += Stemp;
						Vsum += Vtemp;
						//std::cout << "pixel " <<h<<" "<<x<<" BGR: " << Btemp << " " << Gtemp << " " << Rtemp << std::endl;
					}

					lm_new.B = Bsum / (r_pt.x - l_pt.x+1);
					lm_new.G = Gsum / (r_pt.x - l_pt.x+1);
					lm_new.R = Rsum / (r_pt.x - l_pt.x+1);
					lm_new.H = Hsum / (r_pt.x - l_pt.x+1);
					lm_new.S = Ssum / (r_pt.x - l_pt.x+1);
					lm_new.V = Vsum / (r_pt.x - l_pt.x+1);
					//std::cout << "lm average BGR: " << lm_new.B <<" "<< lm_new.G <<" "<< lm_new.R << std::endl;

					lm_new.str_p = l_pt;                                      //start piont
					lm_new.end_p = r_pt;                                      //end point
					lm_new.cnt_p.x = (int)((l_pt.x + r_pt.x) / 2.0);          //center point x
					lm_new.cnt_p.y = r_pt.y;                                  //center point y
					if (lm_new.cnt_p.x > (int)(img_size.width / 2)) {
						lm_new.inn_p = l_pt;
					}
					else {
						lm_new.inn_p = r_pt;
					}
					lm_new.size = r_pt.x - l_pt.x;
					//if(lm_new.S<70){

					lm.push_back(lm_new); //lm are detected lane markings

					// cv::imshow("Lane marking detection", img_test_pixel);
					// cv::waitKey(0);
					//}
					// if (lm_new.S < 60)
					// {
					// 	cv::line(img_test_pixel, lm_new.str_p, lm_new.end_p, CV_RGB(lm_new.R, lm_new.G, lm_new.B), 1, 8, 0); //all lines width=1, this is still not supermarking. the shown picture is consist of many many row lines.
					// }
					// else if (lm_new.S < 70)
					// {
					// 	cv::line(img_test_pixel, lm_new.str_p, lm_new.end_p, CV_RGB(255, 255, 0), 1, 8, 0);
					// }
					// else
					// {
					// 	cv::line(img_test_pixel, lm_new.str_p, lm_new.end_p, CV_RGB(0, 0, 255), 1, 8, 0);
					// }
					w = r_pt.x + 5;
					m_flag = 0;
					if (lm.size() >= MAX_LANE_MARKING - 1)
					{
						std::cout << "lm break!!!" << std::endl;
						break;
					}
					// cv::imshow("Lane marking detection", img_test_pixel);
					// std::cout << "lm color: " << lm_new.B << " " << lm_new.G << " " << lm_new.R << std::endl;
					// std::cout << "lm HSV: " << lm_new.H << " " << lm_new.S << " " << lm_new.V << std::endl;
					// cv::waitKey(0);
				}
				m_flag = 0;
			}
		}
		//std::cout << "before break" << std::endl;
		if (lm.size() >= MAX_LANE_MARKING - 1) {
			break;
		}
		
		//if (h < 120) {
		//	h += SCAN_INTERVAL1;
		//}
		//else {
		//	h += SCAN_INTERVAL2;
		//}
		h += SCAN_INTERVAL1;
		// std::cout << "lm.size(): " << lm.size() << std::endl;
		// std::cout << "h= " << h << std::endl;
	}
	//std::cout << "Lane marking detection finished!" << std::endl;

	if (verbose) {
		//cv::imshow("img_clr", img_clr);
        //std::cout << img_size.width << " " << img_size.height << std::endl;
		cv::Mat img_test = cv::Mat(img_size, CV_8UC3, cv::Scalar(0,0,0));      //\D0½\A8һ\B8\F6\BA\DAɫͼ\CF\F1

		for (int n = 0; n < lm.size(); n++) {
			if(lm[n].S<80 || 10<lm[n].H<44){
				// cv::line(img_test, lm[n].str_p, lm[n].end_p, CV_RGB(lm[n].R, lm[n].G, lm[n].B), 1, 8, 0);//all lines width=1, this is still not supermarking. the shown picture is consist of many many row lines.
				cv::line(img_test, lm[n].str_p, lm[n].end_p, CV_RGB(255, 255, 0), 1, 8, 0);//all lines width=1, this is still not supermarking. the shown picture is consist of many many row lines.
		
			}
			// else if(lm[n].S<70){
			// 	cv::line(img_test, lm[n].str_p, lm[n].end_p, CV_RGB(255, 255, 0), 1, 8, 0);
			// }
			else{
				cv::line(img_test, lm[n].str_p, lm[n].end_p, CV_RGB(0, 0, 255), 1, 8, 0);
			}
			// for(int m=lm[n].str_p.x;m<lm[n].end_p.x+1;m++){
			// 	int h=lm[n].str_p.y;
			// 	int Btemp = img_clr.at<cv::Vec3b>(h, m)[0];
			// 	int Gtemp = img_clr.at<cv::Vec3b>(h, m)[1];
			// 	int Rtemp = img_clr.at<cv::Vec3b>(h, m)[2];
			// 	cv::Point tempPoint;
			// 	tempPoint.x=m;
			// 	tempPoint.y=h;
			// 	img_test.at<cv::Vec3b>(h, m)[0] = Btemp;
			// 	img_test.at<cv::Vec3b>(h, m)[1] = Gtemp;
			// 	img_test.at<cv::Vec3b>(h, m)[2] = Rtemp;
			// 	//cv::circle(img_test, tempPoint, 1, CV_RGB(lm[n].R, lm[n].G, lm[n].B), -1);
			// 	std::cout << "pixel BGR: "<< Btemp << " " << Gtemp << " " << Rtemp << std::endl;
			// }
			//cv::imshow("Lane marking detection", img_test);
			//std::cout << "lm color: " << lm[n].B << " " << lm[n].G << " " << lm[n].R << std::endl;
			//std::cout << "lm HSV: " << lm[n].H << " " << lm[n].S << " " << lm[n].V << std::endl;
			//cv::waitKey(0);
		}
		// cv::imshow("Src hsv ", img_hsv);

		//cv::imshow("Lane marking detection", img_test);
        std::stringstream path;
        path << "/home/chenqi/logging/result_" << image_num << ".jpg";
        //cv::imwrite(path.str(), img_test);
  		img_test.release();
  		// std::cout << "Lane marking detection finished!" << std::endl;
	}
}

void LaneDetection::seed_generation(bool verbose) {

	//std::cout << "seed_generation start!" << std::endl;
	cv::Mat img_marking_seed_pixel = cv::Mat(img_size, CV_8UC3, cv::Scalar(0, 0, 0));
	// Initialization

	// STEP 1-1. Generating Seeds: Making a bunch of seeds consisting of lane markings near each others.
	int flag_group = 0;
	int flag_dist = 0;
	int marking_seed_index; //record the seed where the grouped lm goes
	for (int ii = lm.size() - 1; ii >= 0; ii--) {          
		flag_group = 0;
		for (int jj = marking_seed.size() - 1; jj >= 0; jj--) {

			bool marking_long=0;
			if(marking_seed[jj].index.size()>10){
				marking_long = 1;
			}
			
			flag_dist = dist_ftn1(ii, marking_seed[jj].index[marking_seed[jj].index.size() - 1], marking_seed[jj].cnt_dir, marking_seed[jj].coeff, marking_long);

			if (flag_dist == 1) {
				flag_group = 1;
				marking_seed[jj].index.push_back(ii);
				if (marking_seed[jj].cnt_dir < -99) {
					marking_seed[jj].cnt_dir = slope_ftn(lm[ii].cnt_p, marking_seed[jj].cnt_p);
				}
				else {
					marking_seed[jj].cnt_dir = 0.8*marking_seed[jj].cnt_dir + 0.2*slope_ftn(lm[ii].cnt_p, marking_seed[jj].cnt_p);//TODO: fitting will be better.
				}
				marking_seed[jj].cnt_p = lm[ii].cnt_p;//ii-- makes marking_seed[jj].cnt_p the highest cnt_p of its lanemarks(many row lines). str and end are not defined. (y is minimized)
				
				// polynomial fitting
				std::vector<cv::Point2f> pts;
				std::vector<float> coeff(3);
				
				for (int pp = 0; pp < marking_seed[jj].index.size(); pp++) {
					int idx_lm = marking_seed[jj].index[pp];
					pts.push_back(lm[idx_lm].cnt_p);
				}
				// by @wentuopu if poly3 is singular, turn to poly2 fit
				float length = marking_seed[jj].index.size();
				bool isolated_short = (length < 200);
				float err;
				if (isolated_short) {
					coeff.resize(2);
					err=poly2(pts, pts.size(), coeff);
					//std::cout << "coeff of final lines:" << coeff.size() <<" " <<coeff[0] << " " << coeff[1] << std::endl;
				}
				else {
					err=poly3(pts, pts.size(), coeff);
					//std::cout << "coeff of final lines:" << coeff.size() <<" " <<coeff[0] << " " << coeff[1] << " " << coeff[2] << std::endl; 
				}
				marking_seed[jj].coeff = coeff;

				break;
			}
		}
		if (flag_group == 0) {
			//std::cout << "unassigned" << std::endl << std::endl;
			MARKING_SEED seed_new;
			seed_new.flag = 0;
			seed_new.index.resize(0);
			seed_new.index.push_back(ii);
			seed_new.cnt_dir = -100;
			seed_new.cnt_p = lm[ii].cnt_p;
			marking_seed.push_back(seed_new);
		}
		// else{
		// 	std::cout << "assigned" << std::endl << std::endl;
		// }
		cv::line(img_marking_seed_pixel, lm[ii].str_p, lm[ii].end_p, CV_RGB(0, 255*(1-flag_group), 255*flag_group), 1, 8, 0);
		// cv::imshow("???", img_marking_seed_pixel);
		// cv::waitKey(0);
	}

    for (int i = 0; i < marking_seed.size(); i++) {
        reverse(marking_seed[i].index.begin(), marking_seed[i].index.end());
    }
    //reverse(marking_seed.begin(), marking_seed.end());

	if (verbose) {
		cv::Mat img_test_marking_seed = cv::Mat(img_size, CV_8UC3, cv::Scalar(0,0,0));//initial black picture
		//std::cout << "marking_seed.size:" << marking_seed.size() << std::endl;
		int seedSsum;
		float seedS;
		int seedHsum;
		float seedH;
		int seedVsum;
		float seedV;
		for (int ii = 0; ii < marking_seed.size(); ++ii) {
			seedSsum = 0;
			seedHsum = 0;
			for (int jj = 0; jj < marking_seed[ii].index.size(); ++jj) {
				int idx = marking_seed[ii].index[jj];
				//std::cout << "lm S: " << lm[idx].S << std::endl;
				seedSsum += lm[idx].S;
				seedHsum += lm[idx].H;
				seedVsum += lm[idx].V;
			}
			seedS = (float)seedSsum/marking_seed[ii].index.size();
			seedH = (float)seedHsum / marking_seed[ii].index.size();
			seedV = (float)seedVsum / marking_seed[ii].index.size();
			int	r = rand() % 200 + 50;
			int	g = rand() % 200 + 50;
			int b = rand() % 200 + 50;
			// int r = (int)(seedS)*2;
			// int g = (int)(seedS)*2;
			// int b = (int)(seedS)*2;
			for (int jj = 0; jj < marking_seed[ii].index.size(); ++jj) {
				int idx = marking_seed[ii].index[jj];
				if((seedS<80) || (10<seedH && seedH<44)){
					cv::line(img_test_marking_seed, lm[idx].str_p, lm[idx].end_p, CV_RGB(0,0,255), 1, 8, 0);
				}
				// else if(seedS>60){
					// cv::line(img_test_marking_seed, lm[idx].str_p, lm[idx].end_p, CV_RGB(0,255,0), 1, 8, 0);
				// }
				else{
					// cv::line(img_test_marking_seed, lm[idx].str_p, lm[idx].end_p, CV_RGB(r, g, b), 1, 8, 0);
					cv::line(img_test_marking_seed, lm[idx].str_p, lm[idx].end_p, CV_RGB(255, 0, 0), 1, 8, 0);
				}
				// cv::line(img_test_marking_seed, lm[idx].str_p, lm[idx].end_p, CV_RGB(r, g, b), 1, 8, 0);
			}
			//cv::imshow("Raw marking seeds", img_test_marking_seed);
			//std::cout << "marking_seed S: " << seedS << std::endl;
			//cv::waitKey(0);
            int idx = marking_seed[ii].index.size() - 1;
            idx = marking_seed[ii].index[0];
            //cv::line(img_test_marking_seed, lm[idx].cnt_p, marking_seed[ii].cnt_p, cv::Scalar(0,255,0), 1, 8, 0);
			//cv::circle(img_test_marking_seed, marking_seed[ii].cnt_p, 3, cv::Scalar(255, 255, 255), 2, 8, 0);//draw each maeking_seed's top, white
			//std::cout << 
			//std::cout << marking_seed[ii].str_p << marking_seed[ii].end_p << std::endl;//now still 0!
		}

		// cv::imshow("Raw marking seeds", img_test_marking_seed);
	

        std::stringstream path;
        path << "/home/chenqi/logging/seed_" << image_num << ".jpg";
        //cv::imwrite(path.str(), img_test_marking_seed);
		//cv::imshow("???", img_marking_seed_pixel);
		std::stringstream pathpixel;
		pathpixel << "/home/chenqi/logging/seed_pixel_" << image_num << ".jpg";
		//cv::imwrite(pathpixel.str(), img_marking_seed_pixel);
		// std::cout << "Step 1-1 finished!" << std::endl;
	}
	// STEP 1-2. Seed Validation
	int count_i, count_j;
	float var;
	int seedSsum;
	int seedHsum;
	int seedVsum;
	float seedV;
	float seedS;
	float seedH;
	cv::Mat img_test_valid_seed_pixel = cv::Mat(img_size, CV_8UC3, cv::Scalar(0, 0, 0)); //jxy: 0713temp
	for (int ii = 0; ii < marking_seed.size(); ii++) {
		count_i = marking_seed[ii].index.size();

		// if contained lane marking is less then a certain number
        //std::cout << marking_seed[ii].cnt_p.x - img_width / 2 << std::endl;
        int lm_idx = marking_seed[ii].index[0];

        // by @wentuopu keep markings near the vanishing point//jxy: distance to the center of the top of roi, e.g. detected vp.
        //can verify use vp
		int dist_center_x = (marking_seed[ii].cnt_p.x - img_width / 2);
        int dist_center_y = (marking_seed[ii].cnt_p.y - img_roi_height);
        float dist = sqrt(dist_center_x * dist_center_x + dist_center_y * dist_center_y);
		//jxy: add seedS validation
		seedSsum = 0;
		seedHsum = 0;
		seedVsum = 0;
		for (int jj = 0; jj < marking_seed[ii].index.size(); ++jj)
		{
			int idx = marking_seed[ii].index[jj];
			//std::cout << "lm S: " << lm[idx].S << std::endl;
			seedSsum += lm[idx].S;
			seedHsum += lm[idx].H;
			seedVsum += lm[idx].V;
		}
		seedS = (float)seedSsum / marking_seed[ii].index.size();
		seedH = (float)seedHsum / marking_seed[ii].index.size();
		seedV = (float)seedVsum / marking_seed[ii].index.size();

		//std::cout << "seedHSV: " << seedH << " " << seedS << " " << seedV << std::endl;

		// if(!(seedS<110 && 10<seedH<44 && seedV>170)){
		// 	marking_seed[ii].flag = -2;
		// 	//std::cout << "saturation reject!" << std::endl;
		// 	//continue;
		// }
		if(!((seedS<80) || (10<seedH && seedH<44))){
			marking_seed[ii].flag = -2;
			//std::cout << "saturation reject!" << std::endl;
			//continue;
		}
		else if (count_i < VALID_SEED_MARKING_NUMBER_THRES && dist > 70) {//TODO: a more robust solution, now number threshold is 6, dist threshold is 20*2^0.5
			//those near to vp have few lm, but if far from vp, count_i will be big.
			marking_seed[ii].flag = -1;
			//std::cout << "few lm: "<< count_i <<" & far from vp: " << dist << std::endl;
			//continue;
		}
		else if (count_i < VALID_SEED_MARKING_NUMBER_THRES) {
			float mean = 0.f;
			for (int jj = 0; jj < count_i; jj++) {
				int idx_i = marking_seed[ii].index[jj];
				mean = mean + lm[idx_i].size;
			}
			mean = (float)mean / (float)count_i;
			float var = 0.f;
			for (int jj = 0; jj < count_i; jj++) {
				int idx_i = marking_seed[ii].index[jj];
				var = var + (lm[idx_i].size - mean)*(lm[idx_i].size - mean);
			}
			var = var / (float)count_i;

			// if variance is higher, it regarded as invalid. Those near to vp should not have a big lm width variance.
			if (var > 6.0) {
				marking_seed[ii].flag = -1;
				// std::cout << "reject for var: " << var << std::endl;
			}
		}

		//Rectify: save those long seeds
		if(count_i > 6*VALID_SEED_MARKING_NUMBER_THRES){
			marking_seed[ii].flag = 1;
		}

		if (marking_seed[ii].flag == -1){
			for (int jj = 0; jj < marking_seed[ii].index.size(); ++jj)
			{
				int idx = marking_seed[ii].index[jj];
				cv::line(img_test_valid_seed_pixel, lm[idx].str_p, lm[idx].end_p, CV_RGB(0, 255, 0), 1, 8, 0);
			}
		}
		else if (marking_seed[ii].flag == -2){
			for (int jj = 0; jj < marking_seed[ii].index.size(); ++jj)
			{
				int idx = marking_seed[ii].index[jj];
				cv::line(img_test_valid_seed_pixel, lm[idx].str_p, lm[idx].end_p, CV_RGB(255, 0, 0), 1, 8, 0);
			}
		}
		else{
			for (int jj = 0; jj < marking_seed[ii].index.size(); ++jj)
			{
				int idx = marking_seed[ii].index[jj];
				cv::line(img_test_valid_seed_pixel, lm[idx].str_p, lm[idx].end_p, CV_RGB(0, 0, 255), 1, 8, 0);
			}
		}
		//int idx = seed.index[0];
		// cv::imshow("val marking seeds", img_test_valid_seed_pixel);
		// cv::waitKey(0);
	}

	// STEP 1-3. Seed specification: Getting information of each seeds, position & direction
	std::vector<int> val_seed;

	srand((unsigned)time(NULL));
	int r, g, b;
	for (int ii = 0; ii < marking_seed.size(); ii++) {
		if (marking_seed[ii].flag < 0) {
			continue;
		}
		seed_specification(marking_seed[ii], 1);
		val_seed.push_back(ii);//jxy: record the indexes of valid seeds
	}

	if (verbose) {
		cv::Mat img_test_valid_seed = cv::Mat(img_size, CV_8UC3, cv::Scalar(0,0,0));
		for (int ii = 0; ii < val_seed.size(); ++ii) {
			int	r = rand() % 200 + 50;
			int	g = rand() % 200 + 50;
			int b = rand() % 200 + 50;

			MARKING_SEED seed = marking_seed[val_seed[ii]];
			for (int jj = 0; jj < seed.index.size(); ++jj) {
				int idx = seed.index[jj];
				cv::line(img_test_valid_seed, lm[idx].str_p, lm[idx].end_p, CV_RGB(r, g, b), 1, 8, 0);
			}
            //int idx = seed.index[0];
			//cv::circle(img_test_valid_seed, lm[idx].cnt_p, 3, cv::Scalar(255, 255, 255), 2, 8, 0);
            //std::cout << " [" << ii << "]" << std::endl;
			//std::cout << marking_seed[val_seed[ii]].str_p << "  " << marking_seed[val_seed[ii]].cnt_p << "  " << marking_seed[val_seed[ii]].end_p << std::endl;
			//std::cout << marking_seed[val_seed[ii]].str_dir << "  " << marking_seed[val_seed[ii]].cnt_dir << "  " << marking_seed[val_seed[ii]].end_dir << " " << marking_seed[val_seed[ii]].index.size() << std::endl;
			//cv::imshow("val marking seeds", img_test_valid_seed);
			//cv::waitKey(0);
            //std::stringstream path;
            //path << "result/val_" << image_num++ << ".jpg";
            //cv::imwrite(path.str(), img_test_valid_seed);
		}
		//cv::imshow("val marking seeds", img_test_valid_seed_pixel);
        std::stringstream path;
        path << "/home/chenqi/logging/val_pixel_" << image_num << ".jpg";;
        cv::imwrite(path.str(), img_test_valid_seed_pixel);
		// std::cout << "Step 1-3 finished!" << std::endl;
	}

	// STEP 2. Seed Growing - Dist_mat Generation
	int n_of_valid_seeds = val_seed.size();
	std::vector<int> trns_stats;
	trns_stats.resize(n_of_valid_seeds, -1);
	cv::Mat dist_mat = cv::Mat(n_of_valid_seeds, n_of_valid_seeds, CV_32FC1);

    // std::cout << "valid seeds:";
	for (int ii = 0; ii < n_of_valid_seeds; ++ii) {
		dist_mat.at<float>(ii, ii) = -1.f;
		for (int jj = ii + 1; jj < n_of_valid_seeds; ++jj) {
			dist_mat.at<float>(ii, jj) = dist_ftn2(val_seed[jj], val_seed[ii]);
			dist_mat.at<float>(jj, ii) = dist_mat.at<float>(ii, jj);
		}
		// std::cout << val_seed[ii] << " ";
	}
    
	/*
	for (int ii = 0; ii < n_of_valid_seeds; ++ii) {
		for (int jj = 0; jj < n_of_valid_seeds; ++jj) {
			std::cout << dist_mat.at<float>(ii, jj) << " ";
		}
		std::cout << std::endl;
	}
	std::cout << "Step 2-0 finished!" << std::endl;
    */
	// STEP 2-1. Low Level Association Process #1 - Head -> Tail
    // by @wentuopu
    // change the algorithm, every time the marking will only search the best marking in front whose dist is larger than threshold. The global optimum is not guaranteed. 

	for (int ii = 0; ii < n_of_valid_seeds; ++ii) {
		int cnct_count = 0;
		int cnct_idx = -1;
        float temp_dist_mat = -2;
		int valid_flag = 0;
		for (int jj = 0; jj < ii; ++jj) {
			if (dist_mat.at<float>(ii, jj) > LOW_LEVEL_ASS_THRES) {
				cnct_count++;
                //cnct_idx = jj;
		        float temp_max = LOW_LEVEL_ASS_THRES;
		        int max_id = -1;
                for (int kk = jj; kk < n_of_valid_seeds; kk++) {
                    if (dist_mat.at<float>(jj, kk) > temp_max) {
                        temp_max = dist_mat.at<float>(jj, kk);
                        max_id = kk;
                    }
                }
                if (max_id == ii) {
                   valid_flag = 1;
                   if (dist_mat.at<float>(ii, jj) > temp_dist_mat) {
                       cnct_idx = jj;
                       temp_dist_mat = dist_mat.at<float>(ii, jj);
                   }
                }
			}
		}
        /*
		int valid_flag = 0;
		float temp_max = 0;
		int max_id = -1;

		if (cnct_count >= 1) {
			for (int kk = cnct_idx; kk<n_of_valid_seeds; kk++) {
				if (dist_mat.at<float>(cnct_idx, kk) > temp_max) {
					temp_max = dist_mat.at<float>(cnct_idx, kk);
					max_id = kk;
				}
			}
			if (max_id == ii) {
				valid_flag = 1;
			}
		}
        */
		if (valid_flag == 1) {
            // std::cout << ii << "-->" << cnct_idx << std::endl;
			//	The only seed which comes down to 'cnct_idx' is 'ii'. Thus, 'cnct_idx' has to be connected to 'ii'.
			MARKING_SEED* seed_dst = &marking_seed[val_seed[cnct_idx]];
			MARKING_SEED* seed_connect = &marking_seed[val_seed[ii]];
			count_j = seed_connect->index.size();
			for (int kk = 0; kk < count_j; kk++) {
				seed_dst->index.push_back(seed_connect->index[kk]);
			}
			seed_connect->index.resize(0);
			seed_dst->flag = 1;
			seed_connect->flag = -1;	// seed # which become included in i
			seed_specification(*seed_dst, 0);
			seed_dst->str_dir = seed_connect->str_dir;
			seed_dst->str_p = seed_connect->str_p;
			seed_dst->length = seed_dst->length + seed_connect->length;
			for (int ll = cnct_idx; ll < n_of_valid_seeds; ll++) {
				//dist_mat.at<float>(cnct_idx, ll) = 0;
			}
            for (int ll = 0; ll < ii; ll++) {
                //dist_mat.at<float>(ll, ii) = -1;
            }
            for (int ll = 0; ll < n_of_valid_seeds; ++ll)
                dist_mat.at<float>(ll, ii) = -1;
			// remember where the transition happened
			trns_stats[cnct_idx] = ii;
		}
	}
    /*
	int temp_val = 0;
	int last_idx = 0;
	// STEP 2-2. Low Level Association Process #2 - Head <- Tail
	for (int ii = n_of_valid_seeds - 1; ii >= 0; ii--) {
		int cnct_count = 0;
		int cnct_idx = -1;
		int valid_flag = 0;
        float temp_dist_mat = -2;
		for (int jj = ii + 1; jj < n_of_valid_seeds; jj++) {
			if (dist_mat.at<float>(ii, jj) > LOW_LEVEL_ASS_THRES) {
				cnct_count++;
				//cnct_idx = jj;
		        int temp_max = LOW_LEVEL_ASS_THRES;
		        int max_id = -1;
			    for (int kk = 0; kk < jj; kk++) {
				    if (dist_mat.at<float>(kk, jj) > temp_max) {
					    temp_max = dist_mat.at<float>(kk, jj);
					    max_id = kk;
				    }
			        if (max_id == ii) {
				        valid_flag = 1;
			        }
		        }
                if (dist_mat.at<float>(ii, jj) > temp_dist_mat) {
                    temp_dist_mat = dist_mat.at<float>(ii, jj);
                    cnct_idx = jj;
                    
                }
			}
		}
		if (valid_flag == 1) {
			// remember where the transition happened
            std::cout << ii << "<--" << cnct_idx << std::endl;
			last_idx = cnct_idx;
			temp_val = trns_stats[last_idx];
			while (temp_val != -1) {
				last_idx = temp_val;
				temp_val = trns_stats[last_idx];
			}
			cnct_idx = last_idx;
			// the only seed coming upto 'cnct_idx' is 'i'.
			MARKING_SEED* seed_dst = &marking_seed[val_seed[ii]];
			MARKING_SEED* seed_connect = &marking_seed[val_seed[cnct_idx]];
			count_j = seed_connect->index.size();
			for (int kk = 0; kk < count_j; kk++) {
				seed_dst->index.push_back(seed_connect->index[kk]);
			}
			seed_connect->index.resize(0);
			seed_dst->flag = 1;
			seed_connect->flag = -1;
			seed_specification(*seed_dst, 0);
			seed_dst->end_dir = seed_connect->end_dir;
			seed_dst->end_p = seed_connect->end_p;
			seed_dst->length = seed_dst->length + seed_connect->length;
			for (int ll = 0; ll < cnct_idx; ll++) {
				dist_mat.at<float>(ll, cnct_idx) = 0;
			}
		}
	}*/

	if (verbose) {
		cv::Mat img_test_raw_level_assoc = cv::Mat(img_size, CV_8UC3, cv::Scalar(0,0,0));
		for (int ii = 0; ii < marking_seed.size(); ++ii) {
			if (marking_seed[ii].flag < 0) {
				continue;
			}
			int	r = rand() % 200 + 50;
			int	g = rand() % 200 + 50;
			int b = rand() % 200 + 50;

			MARKING_SEED seed = marking_seed[ii];
			for (int jj = 0; jj < seed.index.size(); ++jj) {
				int idx = seed.index[jj];

				cv::line(img_test_raw_level_assoc, lm[idx].str_p, lm[idx].end_p, CV_RGB(r, g, b), 2, 8, 0);//only draw those that are valid;
			}

            if (ii != -1) {
                //cv::circle(img_test_raw_level_assoc, marking_seed[ii].str_p, 3, cv::Scalar(0,255,255), 2,8,0);
                //cv::circle(img_test_raw_level_assoc, marking_seed[ii].cnt_p, 3, cv::Scalar(255,255,255), 2,8,0);
                //cv::circle(img_test_raw_level_assoc, marking_seed[ii].end_p, 3, cv::Scalar(255,255,0), 2,8,0);
            }
            //std::stringstream path;
            //path << "result/association_" << image_num++ << ".jpg";
            //cv::imwrite(path.str(), img_test_raw_level_assoc);
		}
		//cv::imshow("Low Level Association", img_test_raw_level_assoc);
        std::stringstream path;
		path << "/home/chenqi/logging/association_" << image_num << ".jpg";
		cv::imwrite(path.str(), img_test_raw_level_assoc);
	}
}

void LaneDetection::seed_specification(MARKING_SEED& marking_seed_curr, int mode) {

	float temp_x = 0;
	float temp_y = 0;

	std::vector<float> coeff2;
	std::vector<cv::Point2f> points;
	coeff2.resize(2);
	int n_of_lm = marking_seed_curr.index.size();

	for (int ii = 0; ii < n_of_lm; ii++) {
		int idx_lm = marking_seed_curr.index[ii];
		temp_x += (float)lm[idx_lm].cnt_p.x;
		temp_y += (float)lm[idx_lm].cnt_p.y;
		points.push_back(lm[idx_lm].cnt_p);
	}
	poly2(points, points.size(), coeff2);
	marking_seed_curr.cnt_dir = CV_PI / 2 - atan(coeff2[1]);
	marking_seed_curr.cnt_p.x = (int)(temp_x / n_of_lm);
	marking_seed_curr.cnt_p.y = (int)(temp_y / n_of_lm);

	if (mode == 1) {	// initial seed
		marking_seed_curr.str_p = lm[marking_seed_curr.index[0]].cnt_p;
		marking_seed_curr.end_p = lm[marking_seed_curr.index[n_of_lm - 1]].cnt_p;
		marking_seed_curr.length = length_ftn(marking_seed_curr.str_p, marking_seed_curr.end_p);
		if (n_of_lm < VALID_SEED_MARKING_NUMBER_THRES) {
			marking_seed_curr.end_dir = marking_seed_curr.cnt_dir;
			marking_seed_curr.str_dir = marking_seed_curr.cnt_dir;
		}
		else {
			int n_samples = std::max(5, (int)(0.3f*n_of_lm));
			poly2(points, n_samples, coeff2);
			marking_seed_curr.str_dir = (float)(CV_PI / 2 - atan(coeff2[1]));
			points.resize(0);
			for (int ii = n_of_lm - 1; ii >= n_of_lm - n_samples; ii--) {
				int idx_i = marking_seed_curr.index[ii];
				points.push_back(lm[idx_i].cnt_p);
			}
			poly2(points, n_samples, coeff2);
			marking_seed_curr.end_dir = (float)(CV_PI / 2 - atan(coeff2[1]));
		}
	}

	//printf("%d %d / %d %d\n", marking_seed[idx].str_p.x, marking_seed[idx].str_p.y, marking_seed[idx].end_p.x, marking_seed[idx].end_p.y);
	// the lowest point(in human frame) is the start point, vice versa

}

void LaneDetection::graph_generation(bool verbose) {

	srand((unsigned)time(NULL));

	cv::Mat img_test_graph = cv::Mat(img_size, CV_8UC3);

	// STEP 1. Graph Formulation
	std::vector<int> grp_seed;

	for (int ii = 0; ii < marking_seed.size(); ii++) {
		if (marking_seed[ii].flag < 0) {
			continue;
		}
		if (marking_seed[ii].index.size() < VALID_SEED_MARKING_NUMBER_THRES) {
			continue;
		}
		grp_seed.push_back(ii);
	}


	// STEP 2-1. Node Generation - Generating valid node 
	int n_of_grp_seeds = grp_seed.size();
	cv::Mat vert_mat = cv::Mat(n_of_grp_seeds, n_of_grp_seeds, CV_32SC1);
	std::vector<int> row_sum(n_of_grp_seeds);
	std::vector<int> col_sum(n_of_grp_seeds);
	std::vector<int> ele_sum(n_of_grp_seeds);

	for (int ii = 0; ii < n_of_grp_seeds; ii++) {
		for (int jj = 0; jj < n_of_grp_seeds; jj++) {
			vert_mat.at<int>(ii, jj) = dist_ftn3(grp_seed[ii], grp_seed[jj], ii, jj);
		}
		vert_mat.at<int>(ii, ii) = -1;
	}

	//for (int hh = 0; hh < n_of_grp_seeds; ++hh) {
	//	for (int ww = 0; ww < n_of_grp_seeds; ++ww) {
	//		std::cout << vert_mat.at<int>(hh, ww) << " ";
	//	}
	//	std::cout << std::endl;
	//}

	// STEP 2-2. Separating nodes to each groups
	int n_of_node_grps = 0;
	for (int ii = 0; ii < n_of_grp_seeds; ii++) {
		for (int jj = 0; jj < n_of_grp_seeds; jj++) {
			if (vert_mat.at<int>(ii, jj) == 1) {
				vert_mat.at<int>(ii, jj) = n_of_node_grps + 100;
				node_grouping(vert_mat, n_of_grp_seeds, 0, ii, n_of_node_grps + 100);
				node_grouping(vert_mat, n_of_grp_seeds, 0, jj, n_of_node_grps + 100);
				node_grouping(vert_mat, n_of_grp_seeds, 1, jj, n_of_node_grps + 100);
				node_grouping(vert_mat, n_of_grp_seeds, 1, ii, n_of_node_grps + 100);
				n_of_node_grps++;
			}
		}
	}

	//for (int hh = 0; hh < n_of_grp_seeds; ++hh) {
	//	for (int ww = 0; ww < n_of_grp_seeds; ++ww) {
	//		std::cout << vert_mat.at<int>(hh, ww) << " ";
	//	}
	//	std::cout << std::endl;
	//}

	// STEP 2-3. Node indexing & initialization
	nodes.resize(0);
	for (int ii = 0; ii < n_of_grp_seeds; ii++) {
		for (int jj = 0; jj < n_of_grp_seeds; jj++) {
			if (vert_mat.at<int>(ii, jj) >= 100) {
				NODE_CRF node_new;
				node_new.vert_idx1 = ii;
				node_new.vert_idx2 = jj;
				node_new.idx = vert_mat.at<int>(ii, jj) - 100;
				
				// Node initialization - Unary Term
				node_new.unary = unary_ftn(grp_seed[ii], grp_seed[jj]);   // always be 0
				nodes.push_back(node_new);
			}
		}
	}
	
	// STEP 2-4. Node Grouping
	std::vector<NODE_GRP> node_grp(n_of_node_grps);
	for (int ii = 0; ii < nodes.size(); ii++) {
		int node_grp_idx = nodes[ii].idx;
		node_grp[node_grp_idx].idx.push_back(ii);
	}
	
	//// Grouping result display
	//for (int i = 0; i < n_of_node_grps; i++) {
	//	for (int j = 0; j < node_grp[i].idx.size(); j++) {
	//		printf("%d ", node_grp[i].idx[j]);
	//	}
	//	printf("\n");
	//}

	// Hungarian Method
		// 1) Sorting! in the order of Unary term - Unary term:Logistic function, Sorting - bubble sort
		// 2) Labling using the Constraint - with clear rules! with 4)
		// 3) Calculating the pairwise term with finding Edges - Nodes which are in the same group have the same edges
		// 4) iteration back to the 1) - clear rules, with 2)


	// STEP 3. Hungarian Methos, Edge Indexing, Initialization	

	for (int nn = 0; nn < n_of_node_grps; nn++) {
		// STEP 3-1. Sorting! in the order of Unary term - Unary term:Logistic function, Sorting - bubble sort
		for (int ii = node_grp[nn].idx.size() - 1; ii > 0; ii--) {
			for (int jj = 0; jj < ii; jj++) {
				if (nodes[node_grp[nn].idx[jj]].unary < nodes[node_grp[nn].idx[jj + 1]].unary) {
					int temp_val = node_grp[nn].idx[jj + 1];
					node_grp[nn].idx[jj + 1] = node_grp[nn].idx[jj];
					node_grp[nn].idx[jj] = temp_val;
				}
			}
		}
		//// debugging
		//printf(" > Sorting node grp [%d]:", nn);
		for (int i = 0; i < node_grp[nn].idx.size(); i++) {
			printf(" %d", node_grp[nn].idx[i]);
	    }	
		printf("\n");

		if (node_grp[nn].idx.size() == 1) {	// trivial case which doesn't need the inference 
			continue;
		}
		for (int n_iter = 0; n_iter < node_grp[nn].idx.size(); n_iter++) {
			// STEP 3-2. For each iteration in Hungarian Methods, Find the possible edges
			for (int ii = 0; ii < n_of_grp_seeds; ii++) {
				row_sum[ii] = 0;
				col_sum[ii] = 0;
				ele_sum[ii] = 0;
			}
			nodes[node_grp[nn].idx[n_iter]].label = 1;
			int n_of_labels = 1;
			row_sum[nodes[node_grp[nn].idx[n_iter]].vert_idx1]++;
			col_sum[nodes[node_grp[nn].idx[n_iter]].vert_idx2]++;
			ele_sum[nodes[node_grp[nn].idx[n_iter]].vert_idx1]++;
			ele_sum[nodes[node_grp[nn].idx[n_iter]].vert_idx2]++;

			for (int ii = 0; ii < node_grp[nn].idx.size(); ++ii) {
				if (ii == n_iter) {
					continue;
				}
				if (row_sum[nodes[node_grp[nn].idx[ii]].vert_idx1]>0) {
					nodes[node_grp[nn].idx[ii]].label = 0;
					continue;
				}
				if (col_sum[nodes[node_grp[nn].idx[ii]].vert_idx2]>0) {
					nodes[node_grp[nn].idx[ii]].label = 0;
					continue;
				}
				nodes[node_grp[nn].idx[ii]].label = 1;
				n_of_labels++;
				row_sum[nodes[node_grp[nn].idx[ii]].vert_idx1]++;
				col_sum[nodes[node_grp[nn].idx[ii]].vert_idx2]++;
				ele_sum[nodes[node_grp[nn].idx[ii]].vert_idx1]++;
				ele_sum[nodes[node_grp[nn].idx[ii]].vert_idx2]++;
			}
			// Discarding those which cannot construct an edge
			if (n_of_labels <= 1) {
				continue;
			}
			// Indexing nodes consisting of the edge
			EDGE_CRF edge_new;
			for (int ii = 0; ii < node_grp[nn].idx.size(); ++ii) {
				if (nodes[node_grp[nn].idx[ii]].label == 1) {
					edge_new.node_idx.push_back(node_grp[nn].idx[ii]);
				}
			}
			int iden_flag = 0;	// 0 if different, 1 if identical
			for (int ii = 0; ii < edges.size(); ii++) {
				if (edges[ii].node_idx.size() != edge_new.node_idx.size()) {
					//printf("  >> disciarding <-> %d - count\n", i);
					iden_flag = 0;
					continue;
				}
				for (int jj = 0; jj < edge_new.node_idx.size(); jj++) {
					if (edges[ii].node_idx[jj] != edge_new.node_idx[jj]) {
						//printf("  >> disciarding <-> %d - index %d\n", i, j);
						iden_flag = 0;
						break;
					}
					iden_flag = 1;
				}
				if (iden_flag == 1) {
					break;
				}
			}

			if ((edges.size() != 0) && (iden_flag == 1)) {
				continue;	// this edges is already included
			}

			// STEP 3-3. Pairwise cost calculation
			int n_of_pts = 0;
			std::vector<cv::Point2f> pts;
			for (int ii = 0; ii < n_of_grp_seeds; ii++) {
				if (ele_sum[ii] > 0) {
					int count_i = marking_seed[grp_seed[ii]].index.size();
					for (int jj = 0; jj < count_i; jj++) {
						if (count_i > 15) {
							if (jj % (count_i / 14) != 0) {
								continue;
							}
						}
						cv::Point2f pts_new;
						pts_new.x = (float)lm[marking_seed[grp_seed[ii]].index[jj]].cnt_p.x;
						pts_new.y = (float)lm[marking_seed[grp_seed[ii]].index[jj]].cnt_p.y;
						pts.push_back(pts_new);
					}
				}
			}

			edge_new.pairwise = pairwise_ftn(pts);
			edge_new.grp_idx = nn;
			edges.push_back(edge_new);

			//std::cout << "  > pairwise [" << edges.size() << "] : " << std::endl;
			//for (int tt = 0; tt < edges[edges.size() - 1].node_idx.size(); tt++) {
			//	std::cout << "    " << edges[edges.size() - 1].node_idx[tt] << " " << edges[edges.size() - 1].pairwise << " " << edges[edges.size() - 1].grp_idx << std::endl;
			//}
			//std::cout << std::endl;
		}
	}

	// CRF Formulation, Hungarian Method	
	//printf("\n ==== Hungarian Method ==== \n");
	std::vector<int> final_label;
	final_label.resize(nodes.size(), -1);

	double energy = 0;
	double min_energy = 0;
	int expt_flag = 0;
	for (int nn = 0; nn < n_of_node_grps; nn++) {

		min_energy = 0;
		//printf(" > grp #%d\n\n", nn);
		for (int n_iter = 0; n_iter<node_grp[nn].idx.size(); n_iter++) {
			//printf(" >> iter #%d\n", n_iter);
			// Exception # 1
			if (node_grp[nn].idx.size() == 1) {
				if (nodes[node_grp[nn].idx[0]].unary > 0.5) {
					final_label[node_grp[nn].idx[0]] = 1;
				}
				else {
					final_label[node_grp[nn].idx[0]] = 0;
				}
				continue;
			}
			// Exception # 2
			expt_flag = 0;
			for (int ii = 0; ii<node_grp[nn].idx.size(); ii++) {
				if (nodes[node_grp[nn].idx[ii]].unary > 0.5) {
					break;
				}
				if (ii == node_grp[nn].idx.size() - 1) {
					expt_flag = 1;
				}
			}
			if (expt_flag == 1) {
				continue;
			}

			// 2) Labling using the Constraint - with clear rules! with 4)
			for (int ii = 0; ii < n_of_grp_seeds; ii++) {
				row_sum[ii] = 0;
				col_sum[ii] = 0;
				ele_sum[ii] = 0;
			}
			nodes[node_grp[nn].idx[n_iter]].label = 1;
			int n_of_labels = 1;
			row_sum[nodes[node_grp[nn].idx[n_iter]].vert_idx1]++;
			col_sum[nodes[node_grp[nn].idx[n_iter]].vert_idx2]++;
			ele_sum[nodes[node_grp[nn].idx[n_iter]].vert_idx1]++;
			ele_sum[nodes[node_grp[nn].idx[n_iter]].vert_idx2]++;
			for (int ii = 0; ii < node_grp[nn].idx.size(); ++ii) {
				if (ii == n_iter) {
					continue;
				}
				if (row_sum[nodes[node_grp[nn].idx[ii]].vert_idx1]>0) {
					nodes[node_grp[nn].idx[ii]].label = 0;
					continue;
				}
				if (col_sum[nodes[node_grp[nn].idx[ii]].vert_idx2]>0) {
					nodes[node_grp[nn].idx[ii]].label = 0;
					continue;
				}
				nodes[node_grp[nn].idx[ii]].label = 1;
				n_of_labels++;
				row_sum[nodes[node_grp[nn].idx[ii]].vert_idx1]++;
				col_sum[nodes[node_grp[nn].idx[ii]].vert_idx2]++;
				ele_sum[nodes[node_grp[nn].idx[ii]].vert_idx1]++;
				ele_sum[nodes[node_grp[nn].idx[ii]].vert_idx2]++;
			}

			//printf("  >> # of labels: %d\n", n_of_labels);
			//printf("  >> nodes: ");
			//for(int ii=0;ii<node_grp[nn].idx.size();ii++){
			//	printf("[%d] %d / ", node_grp[nn].idx[ii], nodes[node_grp[nn].idx[ii]].label);
			//}
			//printf("\n");

			// 3) Calculating the pairwise term after finding Edges - Nodes which are in the same group have the common edges
			//printf("  >> edges: ");
			for (int ii = 0; ii < edges.size(); ii++) {
				if (edges[ii].grp_idx != nn) {
					continue;
				}
				if (edges[ii].node_idx.size() != n_of_labels) {
					edges[ii].label = 0;
					continue;
				}
				for (int jj = 0; jj < edges[ii].node_idx.size(); jj++) {
					if (nodes[edges[ii].node_idx[jj]].label != 1) {
						edges[ii].label = 0;
						break;
					}
					edges[ii].label = 1;
				}
				//printf("[%d] %d / ",ii,edges[ii].label);
			}
			//printf("\n");

			////  Calculating Energy & Updating labels
			energy = 0;
			for (int ii = 0; ii < node_grp[nn].idx.size(); ++ii) {
				if (nodes[node_grp[nn].idx[ii]].label == 1) {
					energy = energy - nodes[node_grp[nn].idx[ii]].unary;
				}
				else {
					energy = energy - (1 - nodes[node_grp[nn].idx[ii]].unary);
				}
			}
			expt_flag = 0;
			for (int ii = 0; ii < edges.size(); ++ii) {
				if (edges[ii].grp_idx != nn) {
					continue;
				}
				if (edges[ii].label == 1) {
					energy = energy - edges[ii].pairwise;
					if (edges[ii].pairwise < 0.5) {
						expt_flag = 1;
					}
				}
				else {
					energy = energy - (1 - edges[ii].pairwise);
				}
			}
			//printf(" >>> energy = %.3f, min = %.3f\n", energy, min_energy);				

			if (energy < min_energy) {
				min_energy = energy;
				for (int ii = 0; ii < node_grp[nn].idx.size(); ++ii) {
					final_label[node_grp[nn].idx[ii]] = nodes[node_grp[nn].idx[ii]].label;
					if (expt_flag == 1) {
						final_label[node_grp[nn].idx[ii]] = 0;
					}
					//printf(" >>> [%d] %d\n", node_grp[nn].idx[ii], nodes[node_grp[nn].idx[ii]].label);
				}
			}
			//printf("\n");
			//cvShowImage("Pairwise", testImg);
			//cvWaitKey(0);
		}         
	}

	//// Final Result Printing
	//printf(" >> Labels : ");
	//for (int ii = 0; ii < nodes.size(); ++ii) {
	//	nodes[ii].label = final_label[ii];
	//	printf("%d ", nodes[ii].label);
	//}
	//printf("\n");

	//// Graph Print
	////printf(" > after graph\n");
	////for(int i=0;i<n_of_grp_seeds;i++){
	////	for(int j=0;j<n_of_grp_seeds;j++){
	////		printf("%d ", vert_mat[i][j]);
	////	}
	////	printf("\n");
	////}
	////

	// Seeds association according to the nodes those having been labeled by 1 : s_i -> s_j ( s_i is absorbed into s_j )
	int s_i, s_j;
	int count_j;
	for (int ii = 0; ii < nodes.size(); ++ii) {
		if (nodes[ii].label != 1) {
			continue;
		}
		s_i = grp_seed[nodes[ii].vert_idx1];
		s_j = grp_seed[nodes[ii].vert_idx2];
		count_j = marking_seed[s_j].index.size();
		for (int jj = 0; jj < marking_seed[s_i].index.size(); ++jj) {
			marking_seed[s_j].index.push_back(marking_seed[s_i].index[jj]);
		}

		marking_seed[s_j].flag = 1;
		marking_seed[s_i].flag = -1;
		seed_specification(marking_seed[s_j], 0);
		marking_seed[s_j].str_dir = marking_seed[s_i].str_dir;
		marking_seed[s_j].str_p = marking_seed[s_i].str_p;
		marking_seed[s_j].length = marking_seed[s_i].length + marking_seed[s_j].length;
	}

	// Test displaying
	if (verbose) {
		cv::Mat img_test_crf = cv::Mat(img_size, CV_8UC3, cv::Scalar(0,0,0));
		for (int ii = 0; ii < marking_seed.size(); ++ii) {
			if (marking_seed[ii].flag < 0) {
				continue;
			}
			int r = rand() % 230 + 20;
			int g = rand() % 230 + 20;
			int b = rand() % 230 + 20;

			float sum_dist_marking = 0;
			std::vector<float> vec_dist_mark_y;

			float sum_marking_index = 0;
			for (int jj = 0; jj < marking_seed[ii].index.size(); ++jj) {
				int temp_i = marking_seed[ii].index[jj];
				/*if (jj<marking_seed[ii].index.size() - 1) {
					sum_dist_marking += dist_pt(lm[marking_seed[ii].index[jj]].cnt_p, lm[marking_seed[ii].index[jj+1]].cnt_p);
				}*/
				//calculate score of dash line
				vec_dist_mark_y.push_back(lm[marking_seed[ii].index[jj]].cnt_p.y);
				//vec_dist_mark_y.push_back( ipm.applyHomography(lm[marking_seed[ii].index[jj]].cnt_p).y);

				cv::line(img_test_crf, lm[temp_i].str_p, lm[temp_i].end_p, cv::Scalar(b, g, r), 2, 8, 0);

			}
			//calculate score of dash line
			std::sort(vec_dist_mark_y.begin(), vec_dist_mark_y.end());

			int count_i = 0;
			for (int jj = 0; jj <vec_dist_mark_y.size()-1; ++jj) {
				//sum_dist_marking += fabs(vec_dist_mark_y[jj] - vec_dist_mark_y[jj + 1])  *((1-  (vec_dist_mark_y[jj + 1]-vp_pt.y)/ (480 - vp_pt.y))+0.5);
				sum_dist_marking += fabs(vec_dist_mark_y[jj] - vec_dist_mark_y[jj + 1]);
				if (fabs(vec_dist_mark_y[jj] - vec_dist_mark_y[jj + 1]) < 2 )
				{
					sum_marking_index += fabs(vec_dist_mark_y[jj] - vec_dist_mark_y[jj + 1]);
				}else {
					count_i++;
				}

			}
			marking_seed[ii].dash_score = (sum_dist_marking - (float)marking_seed[ii].index.size()) / (float)marking_seed[ii].index.size();
			marking_seed[ii].dash_score = (sum_dist_marking - sum_marking_index)/  fabs(vec_dist_mark_y[vec_dist_mark_y.size()-1] - vec_dist_mark_y[0]);
		 }
		//cv::imshow("CRF", img_test_crf);
        std::stringstream path;
        path << "/home/chenqi/logging/CRF_" << image_num << ".jpg";
        cv::imwrite(path.str(), img_test_crf);
	}
}

cv::Mat LaneDetection::validating_final_seeds(bool verbose) {

	//cv::Mat img_test_val = cv::Mat(img_size, CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat img_test_val = img_clr.clone();//cv::Mat(img_size, CV_8UC3, cv::Scalar(0,0,0));
	cv::Mat img_test_val_pixel = cv::Mat(img_size, CV_8UC3, cv::Scalar(0,0,0)); //pixel level lanes

//------------------------------------motive vanishing point---------------------------------
	//Replaced by jxy on 0712
	//直接搬进来的
	
// 	cv::Mat frame = img_test_val.clone();
	
// 	cv::Mat image = cv::Mat(cv::Size(img_test_val.rows, img_test_val.cols), CV_8UC1, 0.0);
// 	minlength = image.cols * image.cols * 0.05;

// 	int flag = 0;
// 	clock_t begin, end;

// 	begin = clock(); //耗时计算，对程序运行没有影响

// 	cv::cvtColor(frame, image, cv::COLOR_BGR2GRAY);
// 	cv::Rect r(0,0,768,180);
// 	image(r).setTo(255);
// //	cv::resize(image, image, cv::Size(640, 480));

// 	cv::equalizeHist(image, image); //均衡直方图

// 	init(image, prevRes);
// 	makeLines(flag);
// 	//approximate vanishing point
// 	eval();
// 	//to calculate fps
// 	end = clock();
// 	cout << "old vp time: " << double(end - begin) / CLOCKS_PER_SEC << endl;
	
//----------------------------------------------------------------------------------------------------
//-------------------------------Vector v_PolyfitParam------------------------------------------------

//-------------------------------------static vanishing point------------------------------------------

	//Clear PolyfitParam
	v_PolyfitParam.v_ld_coeff.clear();
	v_PolyfitParam.v_WholeLane_World.clear();
	v_PolyfitParam.v_WholeLane_Pixel.clear();
	//std::cout << "marking_seed.size:" << marking_seed.size() << std::endl;  //without validation, totally 38, however the following codes reject according to flags
	
//-----------@-------------------------	
	int count_coeff = 0;
	for (int jj = 1; jj >= 0; --jj) { //stack, date back to 3 images
		bbb[jj + 1] = bbb[jj]; //number of lines date back to 3 images
	}

	aaa.minus2 = aaa.minus1;
	aaa.minus1 = aaa.now;
	aaa.now.resize(0);
	aaamarking.resize(0);

//------------@jxy vanish vp 0521------------

	std::vector<std::vector<float> > coeff_pic; //coeffs of all lines in the picture
	std::vector<int> marking_id_pic; //marking seed index matching coeff_pic
	float vote_vp_x;
	float vote_vp_y;
	cv::Point2f dot_p;

	clock_t begin2, end2;
	begin2 = clock(); //耗时计算，对程序运行没有影响

	std::vector<int> tmp_lane_style;
	std::vector<float> tmp_lane_style_data;

	g_lane_style.clear();
	g_lane_style_data.clear();
	for (int m = 0; m < VP_Y_DELTA_NUM * 2; m++)
	{
		for (int n = 0; n < VP_X_DELTA_NUM * 2; n++)
		{
			dot_p.x = vp_candidate_x.at<float>(m, n);
			dot_p.y = vp_candidate_y.at<float>(m, n);
			cv::circle(img_test_val, dot_p, 1, cv::Scalar(0, 255, 0), 1, 8, 0);
		}
	}

	for (int ii = 0; ii < marking_seed.size(); ii++) 
	{
		float length = length_ftn(marking_seed[ii].end_p, marking_seed[ii].str_p);
		//std::cout << "length = " << length << std::endl;
		//std::cout << "size = " << marking_seed[ii].index.size() <<std::endl;

		if ((marking_seed[ii].flag == 0) && (marking_seed[ii].index.size() > VALID_SEED_MARKING_NUMBER_THRES)) { //the number of lm
			marking_seed[ii].flag = 1;
            // isolated marker which is large enough
		}
		if (marking_seed[ii].flag < 1) {
			continue;
		}
		//float length = length_ftn(marking_seed[ii].end_p, marking_seed[ii].str_p);
		if (length < 20) {
			//std::cout << "length too short." << std::endl;
			continue;
		}
		if (marking_seed[ii].length < 50) {
			marking_seed[ii].flag = 0;
			//std::cout << "occupied length too short: " << length <<std::endl;
			//continue;
		}
        // by @wentuopu change threshold
		if (marking_seed[ii].length / length < 0.20) {
			marking_seed[ii].flag = 0;
			//std::cout << "too disconnected." <<std::endl;
			//continue;
		}
		if ((length == marking_seed[ii].length) && (length < 62)) {
			marking_seed[ii].flag = 0;
			//std::cout << "small single." <<std::endl;
			//continue;
		}

		// supermarking displaying
		// int r = rand() % 230 + 20;
		// int g = rand() % 230 + 20;
		// int b = rand() % 230 + 20;

		// for (int jj = 0; jj < marking_seed[ii].index.size(); ++jj) {
		// 	int temp_i = marking_seed[ii].index[jj];
		// 	cv::line(img_test_val, lm[temp_i].str_p, lm[temp_i].end_p, cv::Scalar(255, 0, 0), 2, 8, 0);
		// }
		std::vector<float> coeff(3);
		// polynomial fitting
		float err=0;
		std::vector<cv::Point2f> pts;
		
		for (int pp = 0; pp < marking_seed[ii].index.size(); pp++) {
			int idx_lm = marking_seed[ii].index[pp];
			pts.push_back(lm[idx_lm].cnt_p);
		}
		// by @wentuopu if poly3 is singular, turn to poly2 fit
		bool isolated_short = (length < 200);
		
		if (isolated_short) {
			coeff.resize(2);
			err=poly2(pts, pts.size(), coeff);
			//std::cout << "coeff of final lines:" << coeff.size() <<" " <<coeff[0] << " " << coeff[1] << std::endl;
		}
		else {
			err=poly3(pts, pts.size(), coeff);
			//std::cout << "coeff of final lines:" << coeff.size() <<" " <<coeff[0] << " " << coeff[1] << " " << coeff[2] << std::endl; 
		}
		
		coeff_pic.push_back(coeff);
		marking_id_pic.push_back(ii);
		int i_lane_style = 0;
		// cout <<"marking_seed[ii].dash_score "<<ii<<" :"<<marking_seed[ii].dash_score<<endl;

		if (marking_seed[ii].dash_score > DASH_SCORE_THRES )
		{
			i_lane_style = 1;
		// std::cout << "i_lane_style: " << i_lane_style << " : " << marking_seed[ii].dash_score << std::endl;
		}
		tmp_lane_style.push_back(i_lane_style);
		tmp_lane_style_data.push_back(marking_seed[ii].dash_score);

		//std::cout << "marking_id: " << ii << " error: " << err << std::endl;



		//std::cout << ii << " is drawn!" << std::endl;

		//vote vp
		for (int m = 0; m < VP_Y_DELTA_NUM * 2; m++)
		{
			for (int n = 0; n < VP_X_DELTA_NUM * 2; n++)
			{
				if (pass_rectangle(coeff, vp_candidate_x.at<float>(m, n), vp_candidate_y.at<float>(m, n), img_test_val, VP_WINDOW, WINDOW_EXPANSION)){
					vp_countlines.at<float>(m, n) = vp_countlines.at<float>(m, n) + 1;
				}
			}
		}
		
		//std::cout << "vote result: " << vp_countlines << std::endl;

		double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
		cv::minMaxLoc(vp_countlines, &minVal, &maxVal, &minLoc, &maxLoc);
		std::vector<int> max_x_list;
		std::vector<int> max_y_list;
		float min_dist_to_last = 1000; //big enough

		if (image_num == image_num_initial)
		{
			last_vp.x = VP_X;
			last_vp.y = VP_Y;
		}

		for (int m = 0; m < VP_Y_DELTA_NUM * 2; m++)
		{
			for (int n = 0; n < VP_X_DELTA_NUM * 2; n++)
			{
				if (vp_countlines.at<float>(m, n) == maxVal){
					max_x_list.push_back(m);
					max_y_list.push_back(n);
					float temp_distance = sqrt(pow((vp_candidate_x.at<float>(m,n)-last_vp.y),2)+pow(vp_candidate_x.at<float>(m,n)-last_vp.x,2));
					if(temp_distance<min_dist_to_last){
						vote_vp_x = vp_candidate_x.at<float>(m, n);
						vote_vp_y = vp_candidate_y.at<float>(m, n);
					}
				}
			}
		}

		vp_pt.y = vote_vp_y;
		vp_pt.x = vote_vp_x;

		last_vp.y = vote_vp_y;
		last_vp.x = vote_vp_x; //next image will use this

		//cv::circle(img_test_val, vp_pt, 25, cv::Scalar(0, 0, 50), 1, 8, 0);
		//cv::circle(img_test_val, vp_pt, 2, cv::Scalar(0, 50, 50), 2, 8, 0);
		// cv::Mat ipm_show;
		// ipm_visual.applyHomography(img_clr,ipm_show);
		//cv::imshow("ipm_visual", ipm_show);
		// cv::imshow("ipm", img_test_val);

		//cv::waitKey(0);
	}
	end2 = clock();
	// cout << "new vp detection fps: " << 1 / (double(end2 - begin2) / CLOCKS_PER_SEC) << endl;

	//------------@-----------------------
	for (int ii = 0; ii < coeff_pic.size(); ii++)
	{
    //    int vanish_point_x = img_width / 2 + 10;//TEST2compress + 30;
	// 	int vanish_point_y = img_roi_height - 80;
    //    //int vanish_point_y = img_roi_height - 25;
    //    int vanish_point_x = img_width / 2 + 10;//TEST2compress + 30;
	/*	dot_p.y =204/2;
		vp_pt.x = 299/2;
	*/
		std::vector<float> coeff=coeff_pic[ii];
		int marking_id=marking_id_pic[ii];

		//消失点门限，参数在最开始的宏上调，或者lines_std（边缘）的模仿？或者将车道线模仿成它？
		cv::Point2f vp_pt_1;
		cv::rectangle(img_test_val, cv::Rect(VP_X - VP_X_DELTA, VP_Y - VP_Y_DELTA, 2 * VP_X_DELTA, 2 * VP_Y_DELTA), cv::Scalar(0, 0, 0), 1, 8, 0);
		// vp_pt_1.y = soln(1, 0);
		// vp_pt_1.x = soln(0, 0);
		// // vp_pt.y = soln(1, 0);
		// // vp_pt.x = soln(0, 0);
		// cv::circle(img_test_val, vp_pt_1, 25, cv::Scalar(255, 0, 0), 1, 8, 0); //element 1: vp
		//cv::circle(img_test_val, vp_pt, 2, cv::Scalar(255, 0, 0), 2, 8, 0);

		vp_pt.y = vote_vp_y;
		vp_pt.x = vote_vp_x;
		//cv::circle(img_test_val, vp_pt, 25, cv::Scalar(0, 255, 255), 3, 8, 0);//element 1: vp
		cv::circle(img_test_val, vp_pt, 2, cv::Scalar(0, 255, 255), 2, 8, 0);
		//img_roi_height = vp_pt.y;

		float vanish_point_x = vp_pt.x;
		float vanish_point_y = vp_pt.y;

		std::vector<float> dist_to_vp(img_height-1);
		//std::cout << img_height - 1 << std::endl;
		int mindist = 1;
		float xx;

		for (int yy = 1; yy < img_height; ++yy) { //1 to 379, img_height = 380
			xx = valueAt(coeff, yy);
			dist_to_vp[yy - 1] = 0;
		}
		for (int yy = 1; yy < img_height; ++yy) {
			xx = valueAt(coeff, yy);
			dist_to_vp[yy - 1] = sqrt((float)(xx - vanish_point_x)*(xx - vanish_point_x) + (float)(yy - vanish_point_y)*(yy - vanish_point_y));
		}
		for (int yy = 1; yy < img_height; ++yy) {
			if (dist_to_vp[yy - 1] < dist_to_vp[mindist - 1]){
				mindist = yy - 1;
			}
		}

		if (dist_to_vp[mindist] > VP_WINDOW*WINDOW_EXPANSION*6) {
			// std::cout << ii << " is far from vp." << std::endl;
			continue; //those far from vp are abandoned
		}
		else if(dist_to_vp[mindist] > VP_WINDOW*WINDOW_EXPANSION*4 && marking_seed[marking_id].index.size()<50){
			//those long marking_seed with slightly bigger dist_to_vp are kept
			continue; //those far from vp are abandoned
		}
//-----------@-----------------------
		count_coeff = count_coeff + 1;
		bbb[0] = count_coeff;
		//global variable lane style note
		for (int yy = img_roi_height + 10; yy < img_height; ++yy)
		{
			dot_p.y = yy;
			dot_p.x = valueAt(coeff, dot_p.y);

			cv::circle(img_test_val, dot_p, 1, cv::Scalar(0, 50, 0), 1, 8, 0); //element 2: detected lines! valueAt is the fitted result
		}
		g_lane_style_data.push_back(tmp_lane_style_data[ii]);
		g_lane_style.push_back(tmp_lane_style[ii]);
		// std::cout << "lane_style: " << tmp_lane_style[ii] << std::endl;

		if (coeff.size() == 3) {
			aaa.now.push_back(coeff[2]);
			aaa.now.push_back(coeff[1]);
			aaa.now.push_back(coeff[0]); //顺序：二次项在前
		}
		else {
			aaa.now.push_back(0);
			aaa.now.push_back(coeff[1]);
			aaa.now.push_back(coeff[0]); //the current line
		}

		aaamarking.push_back(marking_id); //record marking seed id along with aaa
		//std::cout << "final pixel marking_id: " << marking_id << " with size: " << marking_seed[marking_id].index.size() << std::endl;
		for (int jj = 0; jj < marking_seed[marking_id].index.size(); ++jj) {
			int temp_i = marking_seed[marking_id].index[jj];
			//std::cout << "temp_i: " << temp_i << std::endl;
			cv::line(img_test_val_pixel, lm[temp_i].str_p, lm[temp_i].end_p, cv::Scalar(255, 0, 0), 2, 8, 0);
		}
//------------------------------------
		//Varible Clear by chenqi
		// v_SingleLane_Pixel.clear();
		// v_SingleLane_World.clear();
		// LANE_Point p_tmp;
		
		// for (int yy = img_roi_height + 10; yy < img_height; ++yy) {
		// 	dot_p.y = yy;
		// 	dot_p.x = valueAt(coeff, dot_p.y);
		//    //Lane_Pixel 
		// 	p_tmp.x=dot_p.x;
		// 	p_tmp.y=dot_p.y;
		// 	v_SingleLane_Pixel.push_back(p_tmp);
		// 	p_tmp.x=ipm.applyHomography(dot_p).x;
		// //	std::cout << " p_tmp.x ="<<p_tmp.x << std::endl;
		// //	std::cout << " applyHomography(dot_p).x ="<<ipm.applyHomography(dot_p).x << std::endl;

		// 	p_tmp.y=ipm.applyHomography(dot_p).y;
		// 	v_SingleLane_World.push_back(p_tmp); 
		// 	cv::circle(img_test_val, dot_p, 1, cv::Scalar(0, 255, 0), 1, 8, 0);//element 2: detected lines! valueAt is the fitted result
		// }

		// //std::cout << ii << " is drawn!" << std::endl;//this proved that the markers are clustered to the smallest flag!
		// v_PolyfitParam.v_WholeLane_Pixel.push_back(v_SingleLane_Pixel);
		// v_PolyfitParam.v_WholeLane_World.push_back(v_SingleLane_World);
		// // cv::imshow("ipm", img_test_val);
		// cv::imshow("Lane pixel", img_test_val_pixel);
		// std::cout << "marking_id: " << marking_id << std::endl;
		// cv::waitKey(0);
	}

	if(coeff_pic.size()==0){
		bbb[0] = 0;
	}

	//cv::imshow("vp", img_test_val);
	// cv::imshow("Lane pixel", img_test_val_pixel);
	//cv::waitKey(0);
    
    std::stringstream path;
    path << "/home/chenqi/logging/final_" << image_num << ".jpg";
    cv::imwrite(path.str(), img_test_val);
	std::stringstream path_pixel;
	path_pixel << "/home/chenqi/logging/final_pixel_" << image_num << ".jpg";
	cv::imwrite(path_pixel.str(), img_test_val_pixel);
	return img_test_val;
}

bool LaneDetection::judgepixel(float u, float v, cv::Mat procImg){
	int Btemp = procImg.at<cv::Vec3b>(u, v)[0];
	if(Btemp == 255){
		return 1;
	}
	else{
		return 0;
	}
}

cv::Mat LaneDetection::Xmoutput(){
	return Xm;
}

//-------@--kalman--------------
cv::Mat LaneDetection::kalman(bool verbose) { //jxy: 0417 makes kalman independent, (already assigned in the previous one)
	cv::Mat img_kalman = img_clr.clone();
	cv::Mat img_test_kalman_pixel = img_clr.clone();
	//cv::Mat img_test_kalman_pixel = cv::Mat(img_size, CV_8UC3, cv::Scalar(0, 0, 0));
	//by jxy add kalman
//----lane_data_pub---@xiebo--06.04----------

	//When no lines are detected, each numempty should add by one.
	if (bbb[0] == 0)
	{
		std::cout << "No lines!" << std::endl;
		image_num = image_num + 1;
		for (int jj = 0; jj < num_Xm; ++jj)
		{
			if(numempty.at<float>(jj,0)<0){
				numempty.at<float>(jj, 0)=0;//however many the track is, everything will be back to zero.
			}
			numempty.at<float>(jj, 0) = numempty.at<float>(jj, 0) + 1;
			if (numempty.at<float>(jj, 0) > 5)
			{
				Xm = del_num(Xm, jj * 3 + 2);
				Xm = del_num(Xm, jj * 3 + 1);
				Xm = del_num(Xm, jj * 3);
				numempty = del_num(numempty, jj);
				Pm = del_row(Pm, jj * 3 + 2);
				Pm = del_row(Pm, jj * 3 + 1);
				Pm = del_row(Pm, jj * 3);
				Pm = Pm.t(); //转置后重复上述操作，删除列
				Pm = del_row(Pm, jj * 3 + 2);
				Pm = del_row(Pm, jj * 3 + 1);
				Pm = del_row(Pm, jj * 3);
				Pm = Pm.t(); //转置回来
				num_Xm = num_Xm - 1;

			}
		}
		// cv::imshow("img_kalman", img_kalman);
		cv::imshow("img_kalman_pixel", img_test_kalman_pixel);
		return img_test_kalman_pixel;
	}

	//Clear PolyfitParam
	v_PolyfitParam.v_ld_coeff.clear();
	v_PolyfitParam.v_WholeLane_World.clear();
	v_PolyfitParam.v_WholeLane_Pixel.clear();
	cv::Mat Yaaa(bbb[0] * 3, 1, CV_32FC1); //新息，由于有aaa在储存，因此新息不必设为全局变量
	cv::Mat Yaaa_1(bbb[1] * 3, 1, CV_32FC1); //上一帧
	cv::Mat Yaaa_2(bbb[2] * 3, 1, CV_32FC1); //再上一帧所有LANE COEFF
	cv::Mat marking_ids(bbb[0], 1, CV_32FC1);//与aaa对应的marking_seed的序号
	cv::Mat	lane_style_mat = cv::Mat::zeros(bbb[0], 1, CV_8U);

	//lane_g_style = cv::Mat::zeros(bbb[0], 1, CV_8U);
//	cout<<"bbb[0].size(): "<<bbb[0]<<endl;

//	cout<<"lane_g_style.size(): "<<lane_g_style.size()<<endl;

	for (int jj = 0; jj<bbb[0]; ++jj) {  //current coeff
		Yaaa.at<float>(jj * 3, 0) = aaa.now[jj * 3];
		Yaaa.at<float>(jj * 3 + 1, 0) = aaa.now[jj * 3 + 1];
		Yaaa.at<float>(jj * 3 + 2, 0) = aaa.now[jj * 3 + 2];
		marking_ids.at<float>(jj,0) = aaamarking[jj];
		lane_style_mat.at<uchar>(jj, 0) = (uchar)g_lane_style[jj];
		// cout<<"lane style data: "<<jj <<" "<<g_lane_style_data[jj]<<endl;
	}
	for (int jj = 0; jj<bbb[1]; ++jj) { //-1 coeff 上一帧
		Yaaa_1.at<float>(jj * 3, 0) = aaa.minus1[jj * 3];
		Yaaa_1.at<float>(jj * 3 + 1, 0) = aaa.minus1[jj * 3 + 1];
		Yaaa_1.at<float>(jj * 3 + 2, 0) = aaa.minus1[jj * 3 + 2];
	}
	for (int jj = 0; jj<bbb[2]; ++jj) {//-2 coeff 再上一帧
		Yaaa_2.at<float>(jj * 3, 0) = aaa.minus2[jj * 3];
		Yaaa_2.at<float>(jj * 3 + 1, 0) = aaa.minus2[jj * 3 + 1];
		Yaaa_2.at<float>(jj * 3 + 2, 0) = aaa.minus2[jj * 3 + 2];
	}
	// std::cout << "Yaaa=" << std::endl << Yaaa << std::endl;

	 //ONLY ONCE
	if (image_num == image_num_initial || flag_initial == 1) { //kalman variable initialize   ONLY ONCE
		Xm = Yaaa;
		num_Xm = bbb[0];
		cv::Mat sigma(bbb[0] * 3, 1, CV_32FC1);
		for (int jj = 0; jj<bbb[0]; ++jj) {
			sigma.at<float>(jj * 3, 0) = 0.0006; //先验信息，其实差一些也无所谓，应该很快就会被滤波修正好的
			sigma.at<float>(jj * 3 + 1, 0) = 2.4;
			sigma.at<float>(jj * 3 + 2, 0) = 500;
		}
		cv::Mat V = cv::Mat::eye(bbb[0] * 3, bbb[0] * 3, CV_32FC1);
		for (int jj = 0; jj<bbb[0] * 3; ++jj) {
			V.at<float>(jj, jj) = sigma.at<float>(0, jj);
		}
		R = V*V; //Phi，H，R，Q和Pm参与滤波，因此为全局变量
				 //std::cout << "initial R=" << std::endl << R << std::endl;
		cv::Mat W = 3 * V;
		Q = W*W;
		Phi = cv::Mat::eye(bbb[0] * 3, bbb[0] * 3, CV_32FC1);
		H = cv::Mat::eye(bbb[0] * 3, bbb[0] * 3, CV_32FC1);
		Pm = cv::Mat::eye(bbb[0] * 3, bbb[0] * 3, CV_32FC1);
		numempty = cv::Mat::zeros(bbb[0], 1, CV_32FC1);
		lane_g_id = cv::Mat::zeros(bbb[0],1, CV_8U);
		for (int jj = 0; jj<bbb[0]; ++jj) {
			lane_g_id.at<uchar>(jj, 0) = (uchar)get_id(); //存放global_id
		}

		flag_initial = 0;
	}
	// std::cout << "number of lines: " << bbb[0] << std::endl;

	//合并距离过近的舱 将过近的参数存放到rubbish中，然后剔除XM
	int count_cabin = 0;
	cv::Mat distmat_Xm = cv::Mat::zeros(num_Xm, num_Xm, CV_32FC1);
	for (int jj = 0; jj<num_Xm; ++jj) {
		for (int kk = 0; kk<num_Xm; ++kk) {
			distmat_Xm.at<float>(jj, kk) = dist_line(Xm.at<float>(jj * 3), Xm.at<float>(jj * 3 + 1), Xm.at<float>(jj * 3 + 2), Xm.at<float>(kk * 3), Xm.at<float>(kk * 3 + 1), Xm.at<float>(kk * 3 + 2));
		}
	}
	//std::cout << "distmat_Xm=" << std::endl << distmat_Xm << std::endl;
	std::vector<float> rubbish;
	for (int jj = 0; jj<num_Xm; ++jj) {
		for (int kk = jj + 1; kk<num_Xm; ++kk) {	//**只存了一组数据，数据会被替换，只需删除一次即可，删除序号大的那个，如1：2 删除2
			if ((distmat_Xm.at<float>(jj, kk)<KALMAN_THRES) && !(isinside(rubbish, kk))) {
				rubbish.push_back(kk); //rubbish记载着Xm中过近需要删除的线
				count_cabin = count_cabin + 1;
			}
		}
	}

	std::sort(rubbish.begin(), rubbish.end());
	for (int jj = count_cabin - 1; jj >= 0; --jj) {
		std::cout << rubbish[jj] << " is rubbish." << std::endl;
	}
	num_Xm = num_Xm - count_cabin;
	//std::cout <<  count_cabin << std::endl;
	//std::cout << "Pm=" << std::endl << Pm <<std::endl;
	for (int jj = count_cabin - 1; jj >= 0; --jj) {  
		std::cout << "delete " << rubbish[jj] << std::endl;
		Xm = del_num(Xm, rubbish[jj] * 3 + 2);
		Xm = del_num(Xm, rubbish[jj] * 3 + 1);
		Xm = del_num(Xm, rubbish[jj] * 3);
		numempty = del_num(numempty, rubbish[jj]);
		Pm = del_row(Pm, rubbish[jj] * 3 + 2);
		Pm = del_row(Pm, rubbish[jj] * 3 + 1);
		Pm = del_row(Pm, rubbish[jj] * 3);
		Pm = Pm.t(); //转置后重复上述操作，删除列
		Pm = del_row(Pm, rubbish[jj] * 3 + 2);
		Pm = del_row(Pm, rubbish[jj] * 3 + 1);
		Pm = del_row(Pm, rubbish[jj] * 3);
		Pm = Pm.t(); //转置回来
		std::cout << "delete  id " << (int)lane_g_id.at<uchar>((int)rubbish[jj], 0) << std::endl;

		check_id((int)lane_g_id.at<uchar>((int)rubbish[jj],0));
	//	lane_g_style = del_row_8u(lane_g_style, rubbish[jj]);
		lane_g_id = del_row_8u(lane_g_id, rubbish[jj]); //删掉垃圾id
		// std::cout << "After delete:" << std::endl;
		// std::cout << "Xm=" << std::endl << Xm <<std::endl;
		// std::cout << "Pm=" << std::endl << Pm <<std::endl;

		// std::cout << "lane_g_id=" << std::endl << lane_g_id <<std::endl;

	}
	

	int num_new = bbb[0];

	//求解新息和原始线的对应性
	cv::Mat distmat = cv::Mat::zeros(num_Xm, num_new, CV_32FC1);
	cv::Mat identity = cv::Mat::zeros(num_Xm, 1, CV_32FC1);
	for (int jj = 0; jj<num_Xm; ++jj) {
		for (int kk = 0; kk<num_new; ++kk) {
			distmat.at<float>(jj, kk) = dist_line(Xm.at<float>(jj * 3), Xm.at<float>(jj * 3 + 1), Xm.at<float>(jj * 3 + 2), Yaaa.at<float>(kk * 3), Yaaa.at<float>(kk * 3 + 1), Yaaa.at<float>(kk * 3 + 2));
		}
	}
	//std::cout << "distmat=" << std::endl << distmat << std::endl;
	for (int jj = 0; jj<num_Xm; ++jj) {
		cv::Mat temp = distmat.row(jj).clone(); //clone可以创建新矩阵，否则就是原矩阵部分的指针
		double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
		cv::minMaxLoc(temp, &minVal, &maxVal, &minLoc, &maxLoc);
		//std::cout << "dists of " << jj << " = " << std::endl << temp << std::endl;
		//std::cout << "minimum=" << minVal << " " << "minimumplace=" << minLoc.x << std::endl;
		if (minVal<KALMAN_THRES) {
			identity.at<float>(jj, 0) = minLoc.x;
			if(numempty.at<float>(jj,0)>0){
				numempty.at<float>(jj, 0)=0;//however many the track lost, everything will be back to zero.
			}
			numempty.at<float>(jj, 0) = numempty.at<float>(jj, 0)-1; //numempty是jj的漏检次数，为负则为跟踪次数
		}
		else {
			identity.at<float>(jj, 0) = -1; //matlab中为0，但是c++从0开始，只能将异常记为-1
			if(numempty.at<float>(jj,0)<0){
				numempty.at<float>(jj, 0)=0;//however many the track is, everything will be back to zero.
			}
			numempty.at<float>(jj, 0) = numempty.at<float>(jj, 0) + 1;
		}
	}

	//去除漏检5次的线及numempty，改变协方差矩阵，更改identity阵
	for (int jj = num_Xm - 1; jj >= 0; --jj) {
		if (numempty.at<float>(jj, 0) >= 5) {
			std::cout << "Killing..." << std::endl;
			Xm = del_num(Xm, jj * 3 + 2);
			Xm = del_num(Xm, jj * 3 + 1);
			Xm = del_num(Xm, jj * 3);
			numempty = del_num(numempty, jj);
			Pm = del_row(Pm, jj * 3 + 2);
			Pm = del_row(Pm, jj * 3 + 1);
			Pm = del_row(Pm, jj * 3);
			Pm = Pm.t(); //转置后重复上述操作，删除列
			Pm = del_row(Pm, jj * 3 + 2);
			Pm = del_row(Pm, jj * 3 + 1);
			Pm = del_row(Pm, jj * 3);
			Pm = Pm.t(); //转置回来
			num_Xm = num_Xm - 1;
			identity = del_num(identity, jj);

			std::cout << "Killing  id " << (int)lane_g_id.at<uchar>(jj, 0) << std::endl;

			check_id((int)lane_g_id.at<uchar>(jj, 0));
		//	lane_g_style = del_row_8u(lane_g_style, jj); //删掉垃圾线型

			lane_g_id = del_row_8u(lane_g_id, jj); //删掉垃圾id
		}
	}

	//增舱
	int temp_trace;
	if (image_num - image_num_initial>3) {
		float num_old = (bbb[0] + bbb[1] + bbb[2]) / 3.0;
		// std::cout << "bbb = " << bbb[0] << " " << bbb[1] << " " << bbb[2] << std::endl;
		// std::cout << "num_old = " << num_old << std::endl;
		// std::cout << "num_Xm = " << num_Xm << std::endl;
		cv::Mat flag = cv::Mat::zeros(num_new, 1, CV_32FC1);
		//if (num_old>num_Xm && num_Xm<4) { //这个方法太粗暴，应该设计一套完备的增舱审查制度
		if (num_old>num_Xm) {
			for (int kk = 0; kk<num_new; ++kk) {
				if (ismember(identity, kk) == 0) {
					//std::cout << kk << " is not inside identity." << std::endl; 
					//当前帧与前一帧的方程系数 欧拉距离，满足要求后 继续判断。 //如存在多个满足的样本，只选择了最后一个
					for (int mm = 0; mm<bbb[1]; ++mm) {
						//std::cout << kk << "'s distance to previous" << mm << " is " << dist_line(Yaaa.at<float>(kk * 3), Yaaa.at<float>(kk * 3 + 1), Yaaa.at<float>(kk * 3 + 2), Yaaa_1.at<float>(mm * 3), Yaaa_1.at<float>(mm * 3 + 1), Yaaa_1.at<float>(mm * 3 + 2)) << std::endl;
						if (dist_line(Yaaa.at<float>(kk * 3), Yaaa.at<float>(kk * 3 + 1), Yaaa.at<float>(kk * 3 + 2), Yaaa_1.at<float>(mm * 3), Yaaa_1.at<float>(mm * 3 + 1), Yaaa_1.at<float>(mm * 3 + 2))<KALMAN_THRES) {
							flag.at<float>(kk, 0) = 0.5;
							temp_trace = mm;
							//std::cout << "Ready 0.5 for add." << std::endl;
							break;
						}
						flag.at<float>(kk, 0) = -1;
					}
					//如果满足要求，则判断前一帧与前前帧之间的方程系数 欧拉距离，判断是否 三帧都是这条车道线
					if (flag.at<float>(kk, 0) == 0.5) {
						for (int mm = 0; mm<bbb[2]; ++mm) {
							if (dist_line(Yaaa_1.at<float>(temp_trace * 3), Yaaa_1.at<float>(temp_trace * 3 + 1), Yaaa_1.at<float>(temp_trace * 3 + 2), Yaaa_2.at<float>(mm * 3), Yaaa_2.at<float>(mm * 3 + 1), Yaaa_2.at<float>(mm * 3 + 2))<KALMAN_THRES) {
								flag.at<float>(kk, 0) = 1;
								//std::cout << "Ready 1 for add." << std::endl;
								break;
							}
							flag.at<float>(kk, 0) = -1;
						}
					}
					if(flag.at<float>(kk,0) == 1){
						for (int mm = 0; mm < num_Xm; ++mm)
						{
							float tempdist = dist_line(Xm.at<float>(mm * 3), Xm.at<float>(mm * 3 + 1), Xm.at<float>(mm * 3 + 2), Yaaa.at<float>(kk * 3), Yaaa.at<float>(kk * 3 + 1), Yaaa.at<float>(kk * 3 + 2));
							if (tempdist<KALMAN_THRES){
								flag.at<float>(kk,0) = -1;
								break; //too near to existing line. delete by too near.
							}
						}
					}
				}
			}
			for (int kk = 0; kk<num_new; ++kk) {
				if (flag.at<float>(kk, 0) == 1) {
					Xm.push_back(Yaaa.at<float>(kk * 3));
					Xm.push_back(Yaaa.at<float>(kk * 3 + 1));
					Xm.push_back(Yaaa.at<float>(kk * 3 + 2));
					cv::Mat Pm_temp = cv::Mat::zeros(num_Xm * 3 + 3, num_Xm * 3 + 3, CV_32FC1);
					for (int mm = 0; mm<num_Xm * 3; ++mm) {
						Pm_temp.at<float>(mm, mm) = Pm.at<float>(mm, mm);
					}
					for (int mm = 0; mm<3; ++mm) {
						Pm_temp.at<float>(mm + num_Xm * 3, mm + num_Xm * 3) = 1;
					}
					Pm = Pm_temp;
					numempty.push_back((float)0);
					identity.push_back((float)kk);
					num_Xm = num_Xm + 1;
					int tmp_id = get_id();
					lane_g_id.push_back((uchar)tmp_id);
				}
			}
		}
	}
	if (num_Xm == 0) {
		std::cout << "Fail!!" << std::endl;
	}

	if (num_Xm != 0) {

	//改变舱数后，detect出对应新息
	// std::cout << "identity=" << std::endl << identity << std::endl;
	cv::Mat Y(num_Xm * 3, 1, CV_32FC1);
	
	//Y assignment and pixel output
	// std::cout << "MAX_LW_N: " << MAX_LW_N << std::endl;
	// std::cout << "MIN_LW_N: " << MIN_LW_N << std::endl;
	// std::cout << "MAX_LW_F: " << MAX_LW_F << std::endl;
	// std::cout << "MIN_LW_F: " << MIN_LW_F << std::endl;
	////std::cout << "minmax280: " << min_lw[280] << " " << max_lw[280] << std::endl;
	////std::cout << "minmax400: " << min_lw[400] << " " << max_lw[400] << std::endl;
	////std::cout << "img_roi_height: " << img_roi_height << std::endl;
	float near280sum=0;
	float near400sum=0;
	float near280;
	float near400;
	int count280=0;
	int count400=0;

	lane_g_style = cv::Mat::zeros(num_Xm, 1, CV_8U);

	for (int jj = 0; jj<num_Xm; ++jj) {
		if (identity.at<float>(jj, 0) != -1) {
			// std::cout << "tracking times: " << numempty.at<float>(jj,0) << std::endl;
			Y.at<float>(jj * 3, 0) = Yaaa.at<float>(identity.at<float>(jj, 0) * 3, 0);
			Y.at<float>(jj * 3 + 1, 0) = Yaaa.at<float>(identity.at<float>(jj, 0) * 3 + 1, 0);
			Y.at<float>(jj * 3 + 2, 0) = Yaaa.at<float>(identity.at<float>(jj, 0) * 3 + 2, 0);
			//marking_ids.at<float>(identity.at<float>(jj, 0), 0) is a valid seed tested by kalman
			lane_g_style.at<uchar>(jj,0) = (uchar) lane_style_mat.at<uchar>(identity.at<float>(jj, 0),0);
			
			int marking_id = marking_ids.at<float>(identity.at<float>(jj, 0), 0);
			for (int kk = 0; kk < marking_seed[marking_id].index.size(); kk++) {
				int temp_i = marking_seed[marking_id].index[kk];
				//cv::line(img_test_kalman_pixel, lm[temp_i].str_p, lm[temp_i].end_p, cv::Scalar(255, 0, 0), 2, 8, 0);
				if(lm[temp_i].str_p.y > 275 && lm[temp_i].str_p.y < 285 && numempty.at<float>(jj,0)<-20){ //TODO:find a more robust method
					//std::cout << "near 280: y: " << lm[temp_i].str_p.y << " x: " << lm[temp_i].cnt_p.x << " size: " << lm[temp_i].size << std::endl;
					count280++;
					near280sum+=lm[temp_i].size;
				}
			}
			for (int kk = 0; kk < marking_seed[marking_id].index.size(); kk++) {
				int temp_i = marking_seed[marking_id].index[kk];
				if(lm[temp_i].str_p.y > 395 && lm[temp_i].str_p.y < 405 && numempty.at<float>(jj,0)<-20){
					//std::cout << "near 400: y: " << lm[temp_i].str_p.y << " x: " << lm[temp_i].cnt_p.x << " size: " << lm[temp_i].size << std::endl;
					count400++;
					near400sum += lm[temp_i].size;
				}
			}
		}
		
		else {
			Y.at<float>(jj * 3, 0) = Xm.at<float>(jj * 3, 0);
			Y.at<float>(jj * 3 + 1, 0) = Xm.at<float>(jj * 3 + 1, 0);
			Y.at<float>(jj * 3 + 2, 0) = Xm.at<float>(jj * 3 + 2, 0); //漏检，自估
			lane_g_style.at<uchar>(jj,0) = (uchar)lane_style_mat.at<uchar>(jj,0);

		}
	}

	if (count280 != 0 && count400 != 0 && edge_set_flag == 1){
		near280 = near280sum / count280;
		near400 = near400sum / count400;
		//std::cout << "near280: " << near280 << std::endl;
		//std::cout << "near400: " << near400 << std::endl;
		//std::cout << "280 average and count: " << near280 << " " << count280 << std::endl;
		//std::cout << "400 average and count: " << near400 << " " << count400 << std::endl;
		tempLW_F = (int)(near280 - (280 - 240) / (float)(400 - 280) * (near400 - near280));
		tempLW_N = (int)(near400 + (img_height - 400) / (float)(400 - 280) * (near400 - near280));
		//std::cout << "tempLW_F: " << tempLW_F << std::endl;
		//std::cout << "tempLW_N: " << tempLW_N << std::endl;
		// MAX_LW_N = (int)(tempLW_N*1.2);
		// MIN_LW_N = (int)(tempLW_N*0.8);
		// MAX_LW_F = (int)(tempLW_F*1.4);
		// MIN_LW_F = (int)(tempLW_F*0.6); //far, not accurate enough.
		//std::cout << "MAX_LW_N: " << MAX_LW_N << std::endl;
		//std::cout << "MIN_LW_N: " << MIN_LW_N << std::endl;
		//std::cout << "MAX_LW_F: " << MAX_LW_F << std::endl;
		//std::cout << "MIN_LW_F: " << MIN_LW_F << std::endl;
		edge_set_flag = 0;
	}

	// std::cout << "Y=" << std::endl << Y <<std::endl;

	//改变舱数后相应调整各矩阵
	cv::Mat sigma(num_Xm * 3, 1, CV_32FC1);
	for (int jj = 0; jj<num_Xm; ++jj) {
		sigma.at<float>(jj * 3, 0) = 0.0006; //先验信息，其实差一些也无所谓，应该很快就会被滤波修正好的
		sigma.at<float>(jj * 3 + 1, 0) = 2.4;
		sigma.at<float>(jj * 3 + 2, 0) = 500;
	}
	cv::Mat V = cv::Mat::eye(num_Xm * 3, num_Xm * 3, CV_32FC1);
	for (int jj = 0; jj<num_Xm * 3; ++jj) {
		V.at<float>(jj, jj) = sigma.at<float>(jj, 0);
	}
	R = V*V; //Phi，H，R，Q和Pm参与滤波，因此为全局变量
			 //std::cout << "initial R=" << std::endl << R << std::endl;
	cv::Mat W = 3 * V;
	Q = W*W;
	Phi = cv::Mat::eye(num_Xm * 3, num_Xm * 3, CV_32FC1);
	H = cv::Mat::eye(num_Xm * 3, num_Xm * 3, CV_32FC1);

	//卡尔曼滤波
	cv::Mat Xw = Phi*Xm;
	cv::Mat Pw = Phi*Pm*Phi.t() + Q;
	cv::Mat K = Pw*H.t() / (H*Pw*H.t() + R);
	Pm = (cv::Mat::eye(num_Xm * 3, num_Xm * 3, CV_32FC1) - K*H)*Pw;
	Xm = Xw + K*(Y - Xw);
	//std::cout << "kalman Xm=" << std::endl << Xm.t() << std::endl;

	//画图
	cv::Point2f dot_p;
	std::vector<float> coeff_temp;

	//合并路沿：将较近且靠外的参数存放到rubbish中，然后显示为其他颜色，但是不删除Xm
	int count_cabin2 = 0;
	cv::Mat distmat_Xm2 = cv::Mat::zeros(num_Xm, num_Xm, CV_32FC1);
	for (int jj = 0; jj < num_Xm; ++jj)
	{
		for (int kk = 0; kk < num_Xm; ++kk)
		{
			distmat_Xm2.at<float>(jj, kk) = dist_line(Xm.at<float>(jj * 3), Xm.at<float>(jj * 3 + 1), Xm.at<float>(jj * 3 + 2), Xm.at<float>(kk * 3), Xm.at<float>(kk * 3 + 1), Xm.at<float>(kk * 3 + 2));
		}
	}
	//std::cout << "distmat_Xm=" << std::endl << distmat_Xm << std::endl;
	std::vector<float> rubbish2;
	for (int jj = 0; jj < num_Xm; ++jj)
	{
		for (int kk = jj + 1; kk < num_Xm; ++kk)
		{
			if ((distmat_Xm2.at<float>(jj, kk) < 3*KALMAN_THRES)) //3倍距离阈值，因为路沿与真实线较远
			{
				// std::cout << "dist too near: " << distmat_Xm2.at<float>(jj, kk) << std::endl;
				coeff_temp.resize(0);
				coeff_temp.push_back(Xm.at<float>(0, jj * 3 + 2));
				coeff_temp.push_back(Xm.at<float>(0, jj * 3 + 1));
				coeff_temp.push_back(Xm.at<float>(0, jj * 3)); //current lane, coeff index is equal to power, which is in reverse order of Xm

				double jj_bottom = valueAt(coeff_temp, 470);

				coeff_temp.resize(0);
				coeff_temp.push_back(Xm.at<float>(0, kk * 3 + 2));
				coeff_temp.push_back(Xm.at<float>(0, kk * 3 + 1));
				coeff_temp.push_back(Xm.at<float>(0, kk * 3)); //current lane, coeff index is equal to power, which is in reverse order of Xm

				double kk_bottom = valueAt(coeff_temp, 470);

				int to_del; //将删除jj与kk中的一个，分左车道线和右车道线讨论
				if(coeff_temp[1]<0){ //左车道线
					if(jj_bottom<kk_bottom){
						to_del=jj;
						// std::cout << "delete: " << jj_bottom << std::endl;
						// std::cout << "keep: " << kk_bottom << std::endl;
					}
					else{
						to_del=kk;
						// std::cout << "delete: " << kk_bottom << std::endl;
						// std::cout << "keep: " << jj_bottom << std::endl;
					}
				}
				else{
					if(jj_bottom<kk_bottom){
						to_del=kk;
						// std::cout << "delete: " << kk_bottom << std::endl;
						// std::cout << "keep: " << jj_bottom << std::endl;
					}
					else{
						to_del=jj;
						// std::cout << "delete: " << jj_bottom << std::endl;
						// std::cout << "keep: " << kk_bottom << std::endl;
					}
				}

				if(!(isinside(rubbish2, to_del))){
					//std::cout << "confirmed to_del num: " << to_del << std::endl;
					rubbish2.push_back(to_del); //rubbish2记载着Xm中过近需要删除的线
					count_cabin2 = count_cabin2 + 1;
				}
			}
		}
	}

	// for(int jj=0;jj<rubbish2.size();jj++){
	// 	std::cout << "rubbish2: " << rubbish2[jj] << std::endl;
	// }

	for (int jj = 0; jj<num_Xm; ++jj) { //each jj is a line
		coeff_temp.resize(0);
		coeff_temp.push_back(Xm.at<float>(0, jj * 3 + 2));
		coeff_temp.push_back(Xm.at<float>(0, jj * 3 + 1));
		coeff_temp.push_back(Xm.at<float>(0, jj * 3)); //current lane, coeff index is equal to power, which is in reverse order of Xm

		// std::cout << "coeff_temp=" << coeff_temp[0] << " " << coeff_temp[1] << " " << coeff_temp[2] << std::endl;
		double jj_bottom = valueAt(coeff_temp, 470);

		//----lane_data_pub---@chenqi&xiebo--08.20-----------

		v_SingleLane_Pixel.clear();
		v_SingleLane_World.clear();
		LANE_Point p_tmp;

		std::vector<cv::Point2f> v_pts;
		cv::Point2f tmp_pts;
		
		//可以选择是否按筛选画图
		if(isinside(rubbish2,jj)){
		//if(0){
			//std::cout << jj << " is not ok." << std::endl;
			//std::cout << "delete: " << jj_bottom << std::endl;
			for (int yy = img_roi_height; yy < img_height; ++yy) {
				dot_p.y = yy;
				dot_p.x = valueAt(coeff_temp, dot_p.y);

				p_tmp.x=dot_p.x;
				p_tmp.y=dot_p.y;
				// v_SingleLane_Pixel.push_back(p_tmp);

				p_tmp.x=ipm.applyHomography(dot_p).x;
				p_tmp.y=ipm.applyHomography(dot_p).y;
				// v_SingleLane_World.push_back(p_tmp);
				tmp_pts.x=p_tmp.x;
				tmp_pts.y=p_tmp.y;
				v_pts.push_back(tmp_pts);
		     	//cv::circle(img_kalman,  dot_p, 2, cv::Scalar(255, 0, 0), 1, 8, 0);//element 2: detected lines! valueAt is the fitted result		

				// cv::circle(img_kalman,  cv::Point(p_tmp.x+10,p_tmp.y), 2, cv::Scalar(255, 0, 0), 1, 8, 0);//element 2: detected lines! valueAt is the fitted result		
			}
			
		}
		else if (numempty.at<float>(jj, 0) <= 0) {
			//std::cout << jj << " is ok." << std::endl;
			//std::cout << jj_bottom << std::endl;
			for (int yy = img_roi_height; yy < img_height; ++yy) {
				dot_p.y = yy;
				dot_p.x = valueAt(coeff_temp, dot_p.y);
				char tmp_str[20] = "";
				sprintf(tmp_str, " %d", lane_g_id.at<uchar>(jj, 0));

				// cv::putText(img_kalman, tmp_str, cv::Point(valueAt(coeff_temp, img_height-180), img_height-180), cv::FONT_HERSHEY_DUPLEX, 0.8, CV_RGB(255, 120, 0), 1.9, 8, 0);

				p_tmp.x=dot_p.x;
				p_tmp.y=dot_p.y;
				// v_SingleLane_Pixel.push_back(p_tmp);

				p_tmp.x=ipm.applyHomography(dot_p).x;
				p_tmp.y=ipm.applyHomography(dot_p).y;
				tmp_pts.x=p_tmp.x;
				tmp_pts.y=p_tmp.y;
				v_pts.push_back(tmp_pts);
				// v_SingleLane_World.push_back(p_tmp);

				cv::circle(img_kalman, dot_p, 2, cv::Scalar(0, 255, 0), 1, 8, 0);//element 2: detected lines! valueAt is the fitted result		
			
			}
			//pixel画图
			if (identity.at<float>(jj, 0) != -1)
			{
				int marking_id = marking_ids.at<float>(identity.at<float>(jj, 0), 0);
				for (int kk = 0; kk < marking_seed[marking_id].index.size(); kk++)
				{
					int temp_i = marking_seed[marking_id].index[kk];
					cv::line(img_test_kalman_pixel, lm[temp_i].str_p, lm[temp_i].end_p, cv::Scalar(255, 255, 255), 2, 8, 0);
				}
			}
		}
		else {
			//预测的线根据历史宽度进行pixel画图
			//std::cout << jj << " is ok." << std::endl;
			std::cout << jj_bottom << std::endl;
			for (int yy = img_roi_height; yy < img_height; ++yy) {
				dot_p.y = yy;
				dot_p.x = valueAt(coeff_temp, dot_p.y);

				p_tmp.x=dot_p.x;
				p_tmp.y=dot_p.y;
				// v_SingleLane_Pixel.push_back(p_tmp);

				p_tmp.x=ipm.applyHomography(dot_p).x;
				p_tmp.y=ipm.applyHomography(dot_p).y;
				// v_SingleLane_World.push_back(p_tmp);
				tmp_pts.x=p_tmp.x;
				tmp_pts.y=p_tmp.y;
				v_pts.push_back(tmp_pts);

				//cv::circle(img_kalman,  cv::Point(p_tmp.x+10,p_tmp.y), 2, cv::Scalar(255, 0, 0), 1, 8, 0);//element 2: detected lines! valueAt is the fitted result
				double temp_width = (int)((tempLW_N - tempLW_F) * (yy - img_roi_height) / (img_size.height - img_roi_height) + tempLW_F);
				cv::Point2f start_p;
				cv::Point2f end_p;
				start_p.y = yy;
				start_p.x = valueAt(coeff_temp, dot_p.y) - temp_width / 2;

				end_p.y = yy;
				end_p.x = valueAt(coeff_temp, dot_p.y) + temp_width / 2;

				//cv::line(img_kalman, start_p, end_p, cv::Scalar(0, 0, 255), 2, 8, 0);
				//cv::line(img_test_kalman_pixel, start_p, end_p, cv::Scalar(0, 0, 255), 2, 8, 0);

				cv::circle(img_kalman, dot_p, 2, cv::Scalar(0, 255, 0), 1, 8, 0); //element 2: detected lines! valueAt is the fitted result
			}
		}

	//	cv::putText(img_kalman, tmp_str2, cv::Point(20, 20), cv::FONT_HERSHEY_DUPLEX, 0.8, CV_RGB(255, 120, 0), 1.9, 8, 0);

		std::vector<float> coeff(3);

		int err=poly3(v_pts, v_pts.size(), coeff);
		Polyfit_Param.a = 0;
		Polyfit_Param.b = coeff[2];
		Polyfit_Param.c = coeff[1];
		Polyfit_Param.d = coeff[0];
		Polyfit_Param.global_id =(int)lane_g_id.at<uchar>(jj, 0);
		v_PolyfitParam.v_ld_coeff.push_back(Polyfit_Param);

		v_PolyfitParam.v_WholeLane_Pixel.push_back(v_SingleLane_Pixel);
		v_PolyfitParam.v_WholeLane_World.push_back(v_SingleLane_World);
		cv::imshow("img_kalman", img_kalman);
	}
	}
	else {
		flag_initial = 1;
		std::cout << "reinitialize..." << std::endl;
	}
	//show_id_arr();
	// cv::imshow("img_kalman", img_kalman);
	cv::imshow("img_kalman_pixel", img_test_kalman_pixel);
	std::cout << img_roi_height << std::endl;

	std::stringstream path;
	path << "/home/chenqi/logging/kalman_" << image_num << ".jpg";
	cv::imwrite(path.str(), img_kalman);
	std::stringstream pathpixel;
	pathpixel << "/home/chenqi/logging/kalman_pixel_" << image_num << ".jpg";
	cv::imwrite(pathpixel.str(), img_test_kalman_pixel);
	//kalman_video << img_kalman;
	image_num = image_num + 1;
	std::cout << "num_Xm=" << num_Xm << std::endl;
	//cv::waitKey(0);
	return img_test_kalman_pixel;

}

bool LaneDetection::pass_rectangle(std::vector<float> coeff, float center_x, float center_y, cv::Mat img_test_val, float window, float window_expansion){
	//TODO: find a more robust method
	window=window*window_expansion;
	std::vector<float> corner_x;
	std::vector<float> corner_y;
	std::vector<bool> bigger_x;
	corner_x.resize(4);
	corner_y.resize(4);
	bigger_x.resize(4);
	corner_x[0] = center_x - window; //left up
	corner_y[0] = center_y - window;
	corner_x[1] = center_x + window; //right up
	corner_y[1] = center_y - window;
	corner_x[2] = center_x - window; //left down
	corner_y[2] = center_y + window;
	corner_x[3] = center_x + window; //right down
	corner_y[3] = center_y + window;
	float corner_x_temp;
	for(int ii=0;ii<4;ii++){
		corner_x_temp = valueAt(coeff,corner_y[ii]);
		if(corner_x_temp>corner_x[ii]){
			bigger_x[ii] = true;
		}
		else{
			bigger_x[ii] = false;
		}
	}
	
	if(bigger_x[0]==bigger_x[1] && bigger_x[1]==bigger_x[2] && bigger_x[2]==bigger_x[3]){
		return false;
	}
	//std::cout << "bigger_x: " << bigger_x[0] << " " << bigger_x[1] << " " << bigger_x[2] << " " << bigger_x[3] << std::endl;
	//cv::rectangle(img_test_val, cv::Rect(corner_x[0], corner_y[0], window * 2, window * 2), cv::Scalar(0, 0, 0), 1, 8, 0);
	//cv::imshow("ipm", img_test_val);
	//cv::waitKey(0);

	return true;
}

float LaneDetection::isinside(std::vector<float>rubbish, int kk) {
	int n = rubbish.size();
	for (int jj = 0; jj<n; ++jj) {
		if (rubbish[jj] == kk) {
			//std::cout << kk << " is inside." << std::endl;
			return 1;
		}
	}
	return 0;
}

float LaneDetection::ismember(cv::Mat identity, int kk) {
	int n = identity.rows;
	for (int jj = 0; jj<n; ++jj) {
		if (identity.at<float>(jj, 0) == kk) {
			//std::cout << kk << " is inside." << std::endl;
			return 1;
		}
	}
	return 0;
}

float LaneDetection::dist_line(float A1, float B1, float C1, float A2, float B2, float C2) {
	float dist = 0;
	int yy = 280;
	for (int yy = img_roi_height - 10; yy < img_height; ++yy) {
		dist = dist + fabs((yy*yy*A1 + yy*B1 + C1) - (yy*yy*A2 + yy*B2 + C2));
	}
	dist = dist / (img_height - (img_roi_height - 10));
	return dist;
}

cv::Mat LaneDetection::del_num(cv::Mat ori, int jj) {
	int n = ori.rows; //这是对列向量进行操作，因此按行操作
	cv::Mat temp(n - 1, 1, CV_32FC1);
	int count_temp = 0;
	for (int kk = 0; kk<n; ++kk) {
		if (kk != jj) {
			temp.at<float>(count_temp, 0) = ori.at<float>(kk, 0);
			count_temp = count_temp + 1;
		}
	}
	return temp;
}

cv::Mat LaneDetection::del_row(cv::Mat ori, int jj) {
	int n = ori.rows;
	int m = ori.cols;
	cv::Mat temp;
	int count_temp = 0;
	for (int kk = 0; kk<n; ++kk) {
		if (kk != jj) {
			cv::Mat dst = ori.row(kk);
			temp.push_back(dst);
			count_temp = count_temp + 1;
		}
	}
	return temp;
}

cv::Mat LaneDetection::del_row_8u(cv::Mat ori, int jj) {
	int n = ori.rows; //这是对列向量进行操作，因此按行操作
	cv::Mat temp(n - 1, 1, CV_8U);
	int count_temp = 0;
	for (int kk = 0; kk<n; ++kk) {
		if (kk != jj) {
			temp.at<int>(count_temp, 0) = ori.at<int>(kk, 0);
			count_temp = count_temp + 1;
		}
	}
	return temp;
}
//-------@--kalman---------------------

float LaneDetection::marking_thres(float input) {

	float thres = 0;

	/*if(input<50){
	thres = (int)(input/10);
	}else{
	thres = (int)(15+input/200*10);
	}*/
	//return thres;
    // by @wentuopu a small modification on threshold. Insensitive.
	return input / 5 + 1;
}
int LaneDetection::dist_ftn1(int s_i, int s_j, double slope, std::vector<float> coeff, bool marking_long) {

	// For Seed Generation

	double value = 0;
	double slope_new = slope_ftn(lm[s_i].cnt_p, lm[s_j].cnt_p);
	CvPoint i, j;
	i = lm[s_i].cnt_p;
	j = lm[s_j].cnt_p;
	value = sqrt((i.x - j.x)*(i.x - j.x) + (i.y - j.y)*(i.y - j.y));
	//int hf_size = 2 + 13 * (i.y - img_roi_height + 1) / (img_height - img_roi_height);
	int tolerance = max_lw[i.y] + max_lw_d[i.x];

	//this is unreasonable, for a low slope line, one or two lm miss will cause the following error.
	// if ((lm[s_i].str_p.x > lm[s_j].end_p.x) || (lm[s_i].end_p.x < lm[s_j].str_p.x)) {
	// 	//printf(">> location err (%d,%d) (%d,%d) \n", lm[s_i].str_p.x, lm[s_i].end_p.x,lm[s_j].str_p.x, lm[s_j].end_p.x);
	// 	return 0;
	// }

	//printf("  >> slope : %.3f, diff : %.3f, location : (%d,%d) (%d,%d)", slope, abs(slope-slope_new),  lm[s_i].str_p.x, lm[s_i].end_p.x,lm[s_j].str_p.x, lm[s_j].end_p.x);

	// if (value < SEED_MARKING_DIST_THRES) {
	// 	if (slope <= -99) {
	// 		printf(">> initial\n");
	// 		return 1;
	// 	}
	// 	if ((value>50) && (fabs(slope - slope_new) > 1.1)) {
	// 		return 0;
	// 	}
	// 	if (fabs(slope - slope_new) < 0.8) {
	// 		printf(">> slope old %.3f new %.3f\n",slope, slope_new);
	// 		return 1;
	// 	}
	// 	if ((lm[s_i].cnt_p.x <= lm[s_j].end_p.x) && (lm[s_i].cnt_p.x >= lm[s_j].str_p.x)) {
	// 		printf(">> location\n"); //this is unreasonable, huge slope difference indicates that this lm shouldn't be grouped.
	// 		return 1;
	// 	}
	// }
	// return 0;
	//std::cout << "value: " << value << std::endl;
	if (value < SEED_MARKING_DIST_THRES*(marking_long+1)) {
		//std::cout << "value: " << value << std::endl;
		if(marking_long){
			double xx = valueAt(coeff, i.y);
			if(fabs(i.x-xx)>tolerance/2.0){ //error more than half lane width, reject
				//printf("error: %.3f tolerance: %d\n", j.x-xx, tolerance);
				return 0;
			}
			else{
				//printf("error: %.3f tolerance: %d, accepted\n", i.x - xx, tolerance);
				return 1;
			}
		}
		else{ //for short lanes, polyfit may not be accurate enough
			if (slope <= -99) {
				//printf(">> initial\n");
				return 1;
			}
			if ((value>50) && (fabs(slope - slope_new) > 1.1)) {
				//printf("dist %f and slope", value);
				return 0;
			}
			if (fabs(slope - slope_new) < 0.8) {
				//printf(">> slope old %.3f new %.3f\n",slope, slope_new);
				return 1;
			}
			if ((lm[s_i].cnt_p.x <= lm[s_j].end_p.x) && (lm[s_i].cnt_p.x >= lm[s_j].str_p.x)) {
				//printf(">> location\n"); //this is unreasonable, huge slope difference indicates that this lm shouldn't be grouped.
				return 1;
			}
		}
	}
	return 0;
}
float LaneDetection::dist_ftn2(int i, int j) {

	// For Low level Association
	//std::cout << marking_seed[i].str_p << marking_seed[i].end_p << std::endl;
	if (marking_seed[i].end_p.y > marking_seed[j].str_p.y) {
		return 0;
	}

	// Rough Verification
	std::vector<float> slp;
	slp.resize(7);
	slp[0] = marking_seed[i].cnt_dir;
	slp[1] = marking_seed[j].cnt_dir;
	if ((abs(slp[0] - slp[1])>0.5) && (abs(abs(slp[0] - slp[1]) - 3.141592) < 2.641592)) {
		return 0;
	}
	slp[2] = slope_ftn(marking_seed[i].cnt_p, marking_seed[j].cnt_p);
	slp[3] = slope_ftn(marking_seed[i].str_p, marking_seed[j].str_p);
	slp[4] = slope_ftn(marking_seed[i].str_p, marking_seed[j].end_p);
	slp[5] = slope_ftn(marking_seed[i].end_p, marking_seed[j].str_p);
	slp[6] = slope_ftn(marking_seed[i].end_p, marking_seed[j].end_p);

	// slope variance check
	float slp_mean = (slp[0] + slp[1] + slp[2] + slp[3] + slp[4] + slp[5] + slp[6]) / 7;
	float temp = 0;
	for (int i = 0; i < 7; i++) {
		temp += (slp[i] - slp_mean)*(slp[i] - slp_mean);
	}
	float slp_var = temp / 7;
	if (slp_var > 0.1) {
		return 0;
	}

	// distance ftn between two seeds	
	float sig = 0.25;
	float diff1, diff2;
	diff1 = slp[0] - slp[5];
	diff2 = slp[1] - slp[5];
	// it should be that 1 < 3 < 2 or 2 < 3 < 1
	if (((abs(diff1) + abs(diff2)) > 0.2) && (diff1*diff2 > 0)) {
		return 0;
	}
    

	if (abs(diff1) > 1.570796) {
		diff1 = abs(diff1 - 3.141592);
	}
	if (abs(diff2) > 1.570796) {
		diff2 = abs(diff2 - 3.141592);
	}

	return (float)(exp(-(diff1)*(diff1) / sig*sig) + exp(-(diff2)*(diff2) / sig*sig));
}


int LaneDetection::dist_ftn3(int i, int j, int s_i, int s_j) {

	// Graph Validity of (i to j)

	// Location 1
	if (marking_seed[i].end_p.y >= marking_seed[j].str_p.y) {
		return 0;
	}

	//printf(" >> Node [%d] -> [%d]\n",s_i,s_j);

	// Location 2
	double diff1 = marking_seed[j].str_p.x - (tan(CV_PI / 2 - marking_seed[i].end_dir)*(marking_seed[j].str_p.y - marking_seed[i].end_p.y) + marking_seed[i].end_p.x);
	double diff2 = marking_seed[i].end_p.x - (tan(CV_PI / 2 - marking_seed[j].str_dir)*(marking_seed[i].end_p.y - marking_seed[j].str_p.y) + marking_seed[j].str_p.x);
	//printf("  >> location diff = \t%.3f\t%.3f\t  %.3f\n", abs(diff1), abs(diff2), abs(diff1)+abs(diff2));
	if (abs(diff1) + abs(diff2) > 65) {
		//printf("  >> location diff [%d] -> [%d] = %.3f\t%.3f\t  %.3f\n", s_i,s_j, abs(diff1), abs(diff2), abs(diff1)+abs(diff2));
		return 0;
	}

	// Slope
	double inter_dir = slope_ftn(marking_seed[i].end_p, marking_seed[j].str_p);
	double diff3 = (marking_seed[i].end_dir - inter_dir) / CV_PI * 180;
	double diff4 = (marking_seed[j].str_dir - inter_dir) / CV_PI * 180;
	//printf("  >> slope diff = \t%.3f\t%.3f\t%.3f\t\t%.3f\t%.3f\t%.3f\n", abs(diff3), abs(diff4), abs(diff3)+abs(diff4), inter_dir/CV_PI*180, marking_seed[i].end_dir/CV_PI*180, marking_seed[j].str_dir/CV_PI*180);


    // by @wentuopu reduce the threshold for a more strict rule.
	if (abs(diff3) + abs(diff4) > 30) {
		//printf("  >> slope diff [%d] -> [%d] = %.3f\t%.3f\t  %.3f\t\t%.3f\t%.3f\t%.3f\n", s_i,s_j, abs(diff3), abs(diff4), abs(diff3)+abs(diff4), inter_dir/CV_PI*180, marking_seed[i].end_dir/CV_PI*180, marking_seed[j].str_dir/CV_PI*180);
		return 0;
	}

	//printf(" >> [%d] -> [%d] \n", s_i,s_j);
	return 1;

	// possible to be resued for the Unary term
}

float LaneDetection::slope_ftn(cv::Point2f pos1, cv::Point2f pos2) {

	cv::Point2f temp_pos;
	if (pos1.y > pos2.y) {
		temp_pos = pos1;
		pos1 = pos2;
		pos2 = temp_pos;
	}
	return (float)(acos((double)((pos2.x - pos1.x) / sqrt((float)((pos1.x - pos2.x)*(pos1.x - pos2.x) + (pos1.y - pos2.y)*(pos1.y - pos2.y))))));
}
float LaneDetection::length_ftn(cv::Point2f str_p, cv::Point2f end_p) {

	return sqrt((float)(str_p.x - end_p.x)*(str_p.x - end_p.x) + (float)(str_p.y - end_p.y)*(str_p.y - end_p.y));

}

float LaneDetection::unary_ftn(int i, int j) {

	// Location diff
	double diff1 = marking_seed[j].str_p.x - (tan(CV_PI / 2 - marking_seed[i].end_dir)*(marking_seed[j].str_p.y - marking_seed[i].end_p.y) + marking_seed[i].end_p.x);
	double diff2 = marking_seed[i].end_p.x - (tan(CV_PI / 2 - marking_seed[j].str_dir)*(marking_seed[i].end_p.y - marking_seed[j].str_p.y) + marking_seed[j].str_p.x);

	// Slope diff
	double inter_dir = slope_ftn(marking_seed[i].end_p, marking_seed[j].str_p);
	double diff3 = (marking_seed[i].end_dir - inter_dir) / CV_PI * 180;
	double diff4 = (marking_seed[j].str_dir - inter_dir) / CV_PI * 180;

	double x = abs(diff1) + abs(diff2);
	double y = abs(diff3) + abs(diff4);
	double unary = 0;
	//printf(" %.3f %.3f %.3f\n", term1,term2, unary);

	double a, b, c, d, e, f;
	a = 0.000140047;
	b = 0.001069285;
	c = -0.000263005;
	d = -0.283444141; 
	e = -0.255786389;
	f = 24.86101278;
    //std::cout << x << " " << y << std::endl;
	double fx = a*x*x + b*x*y + c*y*y + d*x + e*y + f;


    // by @wentuopu change unary to zero.(TODO):more suitable coefficients need to be tuned.
	unary = 0;//1 / (1 + exp(-fx));
	return (float)unary;
}

float LaneDetection::pairwise_ftn(std::vector<cv::Point2f>& pts) {

	cv::Point2f dots; 
	//std::vector<float> coeff(4);
	float error = 0;
	//poly4(pts,pts.size(),coeff);
    std::vector<float> coeff(2);
    poly2(pts, pts.size(), coeff);
	for(int ii=0;ii<pts.size();++ii){
		dots.y = (int)pts[ii].y;
		dots.x = (int)(coeff[0]+coeff[1]*dots.y);//+coeff[2]*dots.y*dots.y+coeff[3]*dots.y*dots.y*dots.y);
		error = error + (float)((pts[ii].x-dots.x)*(pts[ii].x-dots.x));
	}
    //std::cout << error / pts.size() << std::endl;	
	double sig = 50;
    double ferr = error / pts.size();
	double pairwise = exp(-(ferr*ferr)/sig/sig);
	
	return (float)pairwise;

}
void LaneDetection::node_grouping(cv::Mat& mat_in, int size, int type, int n, int label) {

	if (type == 0) {
		for (int ii = 0; ii < size; ii++) {
			if (mat_in.at<int>(n, ii) == 1) {
				mat_in.at<int>(n, ii) = label;
				node_grouping(mat_in, size, 0, ii, label);
				node_grouping(mat_in, size, 1, ii, label);
			}
		}
	}

	if (type == 1) {
		for (int ii = 0; ii < size; ii++) {
			if (mat_in.at<int>(ii, n) == 1) {
				mat_in.at<int>(ii, n) = label;
				node_grouping(mat_in, size, 0, ii, label);
				node_grouping(mat_in, size, 1, ii, label);
			}
		}
	}
}


float LaneDetection::poly2(std::vector<cv::Point2f> points, int n, std::vector<float>& coeff) {

	float norm_f = 1.f;
	float temp;
	float err=0;
	cv::Mat a = cv::Mat(2, 2, CV_32FC1);
	cv::Mat b = cv::Mat(2, 1, CV_32FC1);
	cv::Mat c = cv::Mat(2, 2, CV_32FC1);
	cv::Mat d = cv::Mat(2, 1, CV_32FC1);
	cv::Mat e = cv::Mat(2, 1, CV_32FC1);

	for (int ii = 0; ii < n; ii++) {
		points[ii].x = points[ii].x / norm_f;
		points[ii].y = points[ii].y / norm_f;
	}

	// configuring matrix 'a'
	a.at<float>(0, 0) = (float)n;
	temp = 0;
	for (int ii = 0; ii < n; ii++) {
		temp += points[ii].y;
	}
	a.at<float>(0, 1) = (float)temp;
	a.at<float>(1, 0) = (float)temp;
	temp = 0;
	for (int ii = 0; ii < n; ii++) {
		temp += points[ii].y * points[ii].y;
	}
	a.at<float>(1, 1) = (float)temp;
	//temp = 0;
	//for (int ii = 0; ii < n; ii++) {
	//	temp += points[ii].y * points[ii].y * points[ii].y;
	//}

	// configuring matrix 'b'
	temp = 0;
	for (int ii = 0; ii < n; ii++) {
		temp += points[ii].x;
	}
	b.at<float>(0, 0) = (float)temp;
	temp = 0;
	for (int ii = 0; ii < n; ii++) {
		temp += points[ii].x * points[ii].y;
	}
	b.at<float>(1, 0) = (float)temp;

	// matrix operation
	c = a.inv();
	d = c*b;
	coeff[0] = d.at<float>(0, 0)*norm_f;
	coeff[1] = d.at<float>(1, 0)*norm_f;



	//printf("%f %f %f\n", coeff[0], coeff[1], coeff[2]); 
	float xx,yy;
	for (int ii = 0; ii < n; ii++) {
		yy = points[ii].y;
		xx = valueAt(coeff, yy);
		err = err + fabs(xx-points[ii].x);
		//std::cout << "error at " << yy << " : " << xx-points[ii].x << std::endl;
		//cv::waitKey(0);
	}
	err=err/n;
	

	// e = a*d;
	// err = abs(e.at<float>(0, 0) - b.at<float>(0, 0)) + abs(e.at<float>(1, 0) - b.at<float>(1, 0));

	return err;
}
// by @wentuopu modify fitting function by add a scale and use SVD trick for a more stable solution.
float LaneDetection::poly3(std::vector<cv::Point2f> points, int n, std::vector<float>& coeff) {
    
	float norm_f = 255.f;
	float temp;
	float err = 0;
	cv::Mat a = cv::Mat(3, 3, CV_32FC1);
	cv::Mat b = cv::Mat(3, 1, CV_32FC1);
	cv::Mat c = cv::Mat(3, 3, CV_32FC1);
	cv::Mat d = cv::Mat(3, 1, CV_32FC1);
	cv::Mat e = cv::Mat(3, 1, CV_32FC1);

	for (int ii = 0; ii < n; ii++) {
		points[ii].x = points[ii].x / norm_f;
		points[ii].y = points[ii].y / norm_f;
	}
	// configuring matrix 'a'
	a.at<float>(0, 0) = (float)n;
	temp = 0;
	for (int i = 0; i < n; i++) {
		temp += points[i].y;
	}
	a.at<float>(0, 1) = (float)temp;
	a.at<float>(1, 0) = (float)temp;
	temp = 0;
	for (int i = 0; i < n; i++) {
		temp += points[i].y * points[i].y;
	}
	a.at<float>(0, 2) = (float)temp;
	a.at<float>(1, 1) = (float)temp;
	a.at<float>(2, 0) = (float)temp;
	temp = 0;
	for (int i = 0; i < n; i++) {
		temp += points[i].y * points[i].y * points[i].y;
	}
	a.at<float>(1, 2) = (float)temp;
	a.at<float>(2, 1) = (float)temp;
	temp = 0;
	for (int i = 0; i < n; i++) {
		temp += points[i].y * points[i].y * points[i].y * points[i].y;
	}
	a.at<float>(2, 2) = (float)temp;

	// configuring matrix 'b'
	temp = 0;
	for (int i = 0; i < n; i++) {
		temp += points[i].x;
	}
	b.at<float>(0, 0) = (float)temp;
	temp = 0;
	for (int i = 0; i < n; i++) {
		temp += points[i].x * points[i].y;
	}
	b.at<float>(1, 0) = (float)temp;
	temp = 0;
	for (int i = 0; i < n; i++) {
		temp += points[i].y * points[i].y * points[i].x;
	}
	b.at<float>(2, 0) = (float)temp;
    // SVD
    cv::Mat w, u, vt;
    //a = a * 0.01;
    //b = b * 0.01;
    cv::SVD::compute(a, w, u, vt);
    cv::transpose(u, u);
    cv::transpose(vt, vt);
    cv::Mat W = cv::Mat::zeros(3,3,CV_32FC1);
    for (int i = 0; i < 3; i++) {
        W.at<float>(i,i) = 1.0 / (w.at<float>(i));
    }
    //std::cout << u <<std::endl;
    //std::cout << w << std::endl;
    //std::cout << vt << std::endl;
    d = u * b;
    d = W * d;
    d = vt * d;
	// matrix operation
	//cv::Mat aT = cv::Mat(3, 3, CV_32FC1);
    //cv::transpose(a, aT);
    //c = aT * a;
	//c = a.inv();
    //b = aT * b;
	//d = c*b;
	coeff[0] = d.at<float>(0, 0)*norm_f;
	coeff[1] = d.at<float>(1, 0);
	coeff[2] = d.at<float>(2, 0) / norm_f;

	for (int ii = 0; ii < n; ii++) {
		points[ii].x = points[ii].x * norm_f;
		points[ii].y = points[ii].y * norm_f;
	}

	float xx,yy;
	for (int ii = 0; ii < n; ii++) {
		yy = points[ii].y;
		xx = valueAt(coeff, yy);
		err = err + fabs(xx-points[ii].x);
		//std::cout << "error at " << yy << " : " << xx-points[ii].x << std::endl;
		//cv::waitKey(0);
	}
	err=err/n;

	// e = a*d;
	// err = fabs(e.at<float>(0, 0) - b.at<float>(0, 0)) + fabs(e.at<float>(1, 0) - b.at<float>(1, 0)) + fabs(e.at<float>(2, 0) - b.at<float>(2, 0));

	return err;
}

float LaneDetection::poly4(std::vector<cv::Point2f> points, int n, std::vector<float>& coeff) {

	float norm_f = (float)20.f;
	float temp;
	double err = 0;
	cv::Mat a = cv::Mat(4, 4, CV_32FC1);
	cv::Mat b = cv::Mat(4, 1, CV_32FC1);
	cv::Mat c = cv::Mat(4, 4, CV_32FC1);
	cv::Mat d = cv::Mat(4, 1, CV_32FC1);
	cv::Mat e = cv::Mat(4, 1, CV_32FC1);

	for (int i = 0; i < n; i++) {
		points[i].x = points[i].x / norm_f;
		points[i].y = points[i].y / norm_f;
	}
	// configuring matrix 'a'
	a.at<float>(0, 0) = (float)n;
	temp = 0;
	for (int i = 0; i < n; i++) {
		temp += points[i].y;
	}
	a.at<float>(0, 1) = (float)temp;
	a.at<float>(1, 0) = (float)temp;
	temp = 0;
	for (int i = 0; i < n; i++) {
		temp += points[i].y * points[i].y;
	}
	a.at<float>(0, 2) = (float)temp;
	a.at<float>(1, 1) = (float)temp;
	a.at<float>(2, 0) = (float)temp;
	temp = 0;
	for (int i = 0; i < n; i++) {
		temp += points[i].y * points[i].y * points[i].y;
	}
	a.at<float>(0, 3) = (float)temp;
	a.at<float>(1, 2) = (float)temp;
	a.at<float>(2, 1) = (float)temp;
	a.at<float>(3, 0) = (float)temp;
	temp = 0;
	for (int i = 0; i < n; i++) {
		temp += points[i].y * points[i].y * points[i].y * points[i].y;
	}
	a.at<float>(1, 3) = (float)temp;
	a.at<float>(2, 2) = (float)temp;
	a.at<float>(3, 1) = (float)temp;
	temp = 0;
	for (int i = 0; i < n; i++) {
		temp += points[i].y * points[i].y * points[i].y * points[i].y * points[i].y;
	}
	a.at<float>(2, 3) = (float)temp;
	a.at<float>(3, 2) = (float)temp;
	temp = 0;
	for (int i = 0; i < n; i++) {
		temp += points[i].y * points[i].y * points[i].y * points[i].y * points[i].y * points[i].y;
	}
	a.at<float>(3, 3) = (float)temp;

	// configuring matrix 'b'
	temp = 0;
	for (int i = 0; i < n; i++) {
		temp += points[i].x;
	}
	b.at<float>(0, 0) = (float)temp;
	temp = 0;
	for (int i = 0; i < n; i++) {
		temp += points[i].x * points[i].y;
	}
	b.at<float>(1, 0) = (float)temp;
	temp = 0;
	for (int i = 0; i < n; i++) {
		temp += points[i].y * points[i].y * points[i].x;
	}
	b.at<float>(2, 0) = (float)temp;
	temp = 0;
	for (int i = 0; i < n; i++) {
		temp += points[i].y * points[i].y * points[i].y * points[i].x;
	}
	b.at<float>(3, 0) = (float)temp;

	// matrix operation
	c = a.inv();
	d = c*b;
	//printf("\n>> %f %f %f ", cvmGet(d,0,0),cvmGet(d,1,0),cvmGet(d,2,0));
	coeff[0] = d.at<float>(0, 0) * norm_f;
	coeff[1] = d.at<float>(1, 0);
	coeff[2] = d.at<float>(2, 0) / norm_f;
	coeff[3] = d.at<float>(3, 0) / norm_f / norm_f;

	//printf("%f %f %f\n", coeff[0], coeff[1], coeff[2]); 
	//cvmMul(a, d, e);
	e = a*d;
	//err = abs(cvmGet(e,0,0) - cvmGet(b,0,0))+abs(cvmGet(e,1,0) - cvmGet(b,1,0))+abs(cvmGet(e,2,0) - cvmGet(b,2,0));
	err = 0;

	for (int i = 0; i < n; i++) {
		points[i].x = (float)cvRound(points[i].x * norm_f);
		points[i].y = (float)cvRound(points[i].y * norm_f);
	}

	return err;
}

/*

//------------------------@xiebo-- motive v p -------------------------
void LaneDetection::init(cv::Mat image, mat prevRes)
{
	//create OpenCV object for line segment detection
	cv::Ptr<cv::LineSegmentDetector> ls = cv::createLineSegmentDetector(cv::LSD_REFINE_STD);

	//initialize
	lines_std.clear(); //cv::Vec4i的容器，每个里面四个int数

	//detect lines in image and store in linse_std
	//store (x1, y1) and (x2, y2) endpoints for each line segment
	ls->detect(image, lines_std);

	// Show found lines
	cv::Mat drawnLines(image);

	for (int i = 0; i<lines_std.size(); i++)
	{

		//ignore if almost vertical
		if (abs(lines_std[i][0] - lines_std[i][2]) < 10 || abs(lines_std[i][1] - lines_std[i][3]) < 10) //check if almost vertical
			continue;
		//ignore shorter lines (x1-x2)^2 + (y2-y1)^2 < minlength
		if (((lines_std[i][0] - lines_std[i][2])*(lines_std[i][0] - lines_std[i][2]) + (lines_std[i][1] - lines_std[i][3])*(lines_std[i][1] - lines_std[i][3])) < minlength)
			continue; //短的不要

		//store valid lines' endpoints for calculations
		for (int j = 0; j<4; j++)
		{
			temp.push_back(lines_std[i][j]);
		}

		points.push_back(temp); //保存每条线的参数，具体是啥不知道
		temp.clear();
	}
	ls->drawSegments(drawnLines, lines_std);
	//cv::imshow("Lines", drawnLines);
	//cout<<"Detected:"<<lines_std.size()<<endl;
	//cout<<"Filtered:"<<points.size()<<endl;
}

void LaneDetection::makeLines(int flag)
{
	// to solve Ax = b for x
	A = zeros<mat>(points.size(), 2);
	b = zeros<mat>(points.size(), 1);

	//convert given end-points of line segment into a*x + b*y = c format for calculations
	//do for each line segment detected
	for (int i = 0; i<points.size(); i++)
	{

		A(i, 0) = -(points[i][3] - points[i][1]);			//-(y2-y1)
		A(i, 1) = (points[i][2] - points[i][0]);				//x2-x1
		b(i, 0) = A(i, 0)*points[i][0] + A(i, 1)*points[i][1];	//-(y2-y1)*x1 + (x2-x1)*y1
	}
}

void LaneDetection::eval()
{
	//stores the estimated co-ordinates of the vanishing point with respect to the image
	soln = zeros<mat>(2, 1);

	//initialize error
	double err = 9999999999;

	//calculate point of intersection of every pair of lines and
	//find the sum of distance from all other lines
	//select the point which has the minimum sum of distance
	for (int i = 0; i<points.size(); i++)
	{
		for (int j = 0; j<points.size(); j++)
		{
			if (i >= j)
				continue;

			//armadillo vector
			uvec indices;

			//store indices of lines to be used for calculation
			indices << i << j;

			//extract the rows with indices specified in uvec indices
			//stores the ith and jth row of matrices A and b into Atemp and btemp respectively
			//hence creating a 2x2 matrix for calculating point of intersection
			Atemp = A.rows(indices);
			btemp = b.rows(indices);

			//if lines are parallel then skip
			if (rank(Atemp) != 2)
				continue;

			//solves for 'x' in A*x = b
			res = calc(Atemp, btemp);


			if (res.n_rows == 0 || res.n_cols == 0)
				continue;

			// calculate error assuming perfect intersection is 
			error = A*res - b;

			//reduce size of error
			error = error / 1000;

			// to store intermediate error values
			temperr = 0;
			//summation of errors
			for (int i = 0; i<error.n_rows; i++)
				temperr += (error(i, 0)*error(i, 0)) / 1000;

			//scale errors to prevent any overflows
			temperr /= 1000000;

			//if current error is smaller than previous min error then update the solution (point)
			if (err > temperr)
			{
				soln = res;
				err = temperr;
			}
		}
	}

	//cout<<"\n\nResult:\n"<<soln(0,0)<<","<<soln(1,0)<<"\nError:"<<err<<"\n\n";

	// draw a circle to visualize the approximate vanishing point
//	if (soln(0, 0) > 0 && soln(0, 0) < image.cols && soln(1, 0) > 0 && soln(1, 0) < image.rows)
	//{
		//cv::circle(image, cv::Point(soln(0, 0), soln(1, 0)), 25, cv::Scalar(0, 0, 255), 10); 
	//	vp_pt.x = soln(0, 0);
	//	vp_pt.y = soln(1, 0);
//
	//}
//	cv::imshow("win", image);

	//flush the vector
	points.clear();

	//toDo: use previous frame's result to reduce calculations and stabilize the region of vanishing point
	prevRes = soln;
}
*/
