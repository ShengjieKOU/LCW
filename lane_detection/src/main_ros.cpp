#include <iostream>
#include <string>
#include <sstream>
#include <stdlib.h>

#include <time.h>
#include <opencv2/opencv.hpp>
#include "LaneDetection.h"
#include <ros/ros.h>						 //ros 的头文件

//---获取视频信号-----
#include <image_transport/image_transport.h> //image_transport
					     //包含的是ImageTransport类，这个类提供ROS中图像的订阅和发布
#include <cv_bridge/cv_bridge.h>	     //cv_bridge
                                             //包含了CvBridge类，而CvBridge中的API可以将ROS下的sensor_msgs/Image消息类型转化成cv::Mat。
#include <sensor_msgs/image_encodings.h>	 //图像编码格式，包含对图像进行编码的函数。
#include <opencv2/imgproc/imgproc.hpp>		 //图像处理
#include <opencv2/highgui/highgui.hpp>		 //opencv GUI
//-----------


//---msg的头，需要发布的信号---------
#include "lane_detection/ld_Frame.h"
#include "lane_detection/ld_Point.h"
#include "lane_detection/ld_Coeff.h"
#include "lane_detection/ld_LaneParam.h"
//-----------

#include "JudgeLane.h"
// #include "kalman.h"

std::vector<float> hist_time;

CJudgeCenter JudgeCenter ;
static const std::string OPENCV_WINDOW = "Image window"; //申明一个GUI 的显示的字符串
LaneDetection lane;
double pixel_ratio=40; //world -->> pixel
double thres_ldw=0.65; //warning threshold

int predict_num=0;  //连续预测次数
bool predict_continue_flag = true;  //连续预测flag
std::vector<float> hist_v_LeftDis;
std::vector<float> hist_v_RightDis;
std::vector<float> hist_v_LaneWidth;
std::vector<float> hist_v_RadiusOfCurve;

// Initializing variables depending on the resolution of the input image.
// 根据车辆坐标系的方程，返回方程值(真实尺寸)
double valueAtIPM(std::vector<float>& f, float x) {
	float ans = 0.f;
	for (int i = (int)f.size() - 1; i >= 0; --i)
		ans = ans * x + f[i];
	return ans;
}
std::vector<LD_COEFF> CurveByParallel(std::vector<LD_COEFF> _ld_coeff)
{
	std::vector<LD_COEFF> tmp_ld_coeff;


}

void vector_InitValue(std::vector<float> & vector_a,float value,int num)
{
	if(vector_a.size()!=0){
		vector_a.clear();
	}else{
		for(int ii=0;ii<num;ii++){
		vector_a.push_back(value);
		}
	}
}

void vector_Update(std::vector<float> & vector_a,float value)
{
	vector_a.push_back(value);
	vector_a.erase(vector_a.begin());
}
class ImageConverter //申明一个图像转换的类
{
	ros::NodeHandle nh_; //实例化一个节点
	ros::NodeHandle nh_param;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_; //订阅节点
	image_transport::Publisher image_pub_;  //发布节点

	ros::Publisher pub_frame;//publisher frame(lane pixel/lane world)
	ros::Publisher image_mat_pub;			//发布节点
	std::string strCameraSub;
	std::string strCameraRosPub;
	std::string strCameraId;

	//std::string strCameraMatPub;
	bool verbose_lm_detction;
	bool verbose_seed_gen;
	bool verbose_run_crf;
	bool verbose_validating;
	bool verbose_kalman ;
	bool verbose;
	cv::Mat img_visual;
	cv::Mat img_visual_static;
	
	//msg obj
	lane_detection::ld_Frame ld_obj;
   public:
	ImageConverter():it_(nh_),nh_param("~")
	{
	verbose_lm_detction = false;
	verbose_seed_gen = false;
	verbose_run_crf = false;
	verbose_validating = false;
	verbose_kalman=false;
	verbose = verbose_lm_detction | verbose_seed_gen | verbose_run_crf | verbose_validating;

	nh_param.param<std::string>("CameraSub", strCameraSub, "/camera1/image_raw");
	nh_param.param<std::string>("CameraRosPub", strCameraRosPub, "/lane_detection/output_video");
	nh_param.param<std::string>("CameraId", strCameraId, "");
	// nh_param.param<std::string>("CameraMatPub",strCameraMatPub,"/image_converter/output_mat");
	// Subscrive to input video feed and publish output video feed


	image_sub_ = it_.subscribe(strCameraSub, 1, &ImageConverter::imageCb, this);
        //---------定义图像接收器，订阅话题是strCameraSub----------
	image_pub_ = it_.advertise(strCameraRosPub, 1);
	pub_frame = nh_.advertise<lane_detection::ld_Frame>("lane_data", 1);

	img_visual.create(320,480,CV_8UC3);
	img_visual_static.create(240,320,CV_8UC3);
	img_visual.setTo(0);
	img_visual_static.setTo(0);
  //  pub_frame = nh_.advertise<LANE_POLYFIT_PARAM>("lane_data", 1);

	//image_mat_pub = it_.advertise<mat_pub::CameraMatData>(strCameraMatPub, 1);
	//cv::namedWindow(OPENCV_WINDOW);
	}

	~ImageConverter()
	{
	//	cv::destroyWindow(OPENCV_WINDOW);
	}

	void imageCb(const sensor_msgs::ImageConstPtr &msg) //回调函数
	{
		//将ros中的图像类型由sensor image 转为 Mat 
		cv_bridge::CvImagePtr cv_ptr; //申明一个CvImagePtr
		try
		{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception &e)
		{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
		}

		// Update GUI Window
		cv::Mat img_resize;
		cv::Mat img_result;
		//抛出mat类型，待改进
		// image_process::mat_pub Mat_test;
		//
		cv::resize(cv_ptr->image, img_resize, cv::Size(768, 480)); //可设置外部接口，待改进
											            //put your codes here
		process(img_resize, img_result);
		//
		// cv::putText(img_result, "DIAS " + strCameraId, cv::Point(25, 40), cv::FONT_HERSHEY_DUPLEX, 0.6, CV_RGB(0, 140, 255), 1.9, 8, 0);
		//  cv::circle(img_resize, cv::Point(400,240), 10, CV_RGB(255,0,0));
		cv_ptr->image = img_result.clone();
		// cv::imshow(OPENCV_WINDOW, img_resize);
	//	cv::imshow(OPENCV_WINDOW, cv_ptr->image);
		cv::waitKey(2);
		// Output modified video stream
		//image_pub_.publish(cv_ptr->toImageMsg());
		image_pub_.publish(cv_ptr->toImageMsg());
		//  Mat_test.mat_video=img_resize.clone();
		//  image_mat_pub.publish(Mat_test);
	}
	int process(cv::Mat srcImg, cv::Mat &procImg)
	{
		//	VideoCapture >> srcImg;
		//cv::resize(srcImg, srcImg, cv::Size(853, 480));
	
		if (!lane.initialize_variable(srcImg))
		{
			return -1;
		}

		double main_time1 = clock();
		std::cout << std::endl;
		std::cout << std::endl;
		std::cout << std::endl;
		std::cout << "......................." << std::endl;
		std::cout << "new" << std::endl;
		if (!lane.initialize_Img(srcImg))
		{
			return 0;
		}

		double process_time1 = clock();
		// detecting lane markings
		lane.lane_marking_detection(verbose_lm_detction);
		

		// supermarking generation and low-level association
		lane.seed_generation(verbose_seed_gen);

		// CRF graph configuration & optimization using hungarian method
		lane.graph_generation(verbose_run_crf);

		
		lane.validating_final_seeds(verbose_validating);
		double process_time2 = clock();

		// validating
//		cv::imshow("vanishing",	lane.validating_final_seeds(verbose_validating));
		procImg = lane.kalman(verbose_kalman).clone();
		//cv::imshow("kalman",procImg);

		
		std::cout << "process_time " << (process_time2 - process_time1) << "mms" << std::endl;

		lane_detection::ld_Point tmp_Point;
		lane_detection::ld_Coeff tmp_Coeff;
		lane_detection::ld_LaneParam tmp_LaneParam_Pixel;
		lane_detection::ld_LaneParam tmp_LaneParam_World;
		ld_obj.lane_Pixel.clear();
		ld_obj.lane_World.clear();
		ld_obj.lane_Coeff.clear();
		std::vector<double> candidate_dis; 
		//候选ID
		std::vector<int> candidate_id; 

		candidate_id.clear();
		candidate_dis.clear();


//
		float lane_angle = 0.0;
		float dist_bias_1 = 0.0; //the distace in 1 meter
		float dist_bias_5 = 0.0; //the distace in 5 meter
//
		std::vector<float> Lane_angle(2);
		std::vector<float> Dist1(2);
		std::vector<float> Dist5(2);
		Lane_angle.clear();
		Dist1.clear();
		Dist5.clear();

		img_visual.setTo(0);
		img_visual_static.setTo(0);


		for (int i = 0; i < lane.v_PolyfitParam.v_ld_coeff.size(); i++)
		{
			// for(int j = 0; j< lane.v_PolyfitParam.v_WholeLane_World[i].size(); j++)
			// {
			// 	tmp_Point.x=lane.v_PolyfitParam.v_WholeLane_World[i][j].x;
			// 	tmp_Point.y=lane.v_PolyfitParam.v_WholeLane_World[i][j].y;
			// 	tmp_LaneParam_World.lane_Single.push_back(tmp_Point);
			// 	// std::cout<<"tmp_point"<<tmp_Point<<std::endl;
			// 	// std::cout<<"tmp_LaneParam_World"<<tmp_LaneParam_World.lane_Single[j]<<std::endl;

			// 	// tmp_Point.x=lane.v_PolyfitParam.v_WholeLane_Pixel[i][j].x;
			// 	// tmp_Point.y=lane.v_PolyfitParam.v_WholeLane_Pixel[i][j].y;

			// 	// tmp_LaneParam_Pixel.lane_Single.push_back(tmp_Point);

			// }
			// ld_obj.lane_Pixel.push_back(tmp_LaneParam_Pixel);
			// ld_obj.lane_World.push_back(tmp_LaneParam_World);
			tmp_Coeff.a=lane.v_PolyfitParam.v_ld_coeff[i].a;
			tmp_Coeff.b=lane.v_PolyfitParam.v_ld_coeff[i].b;
			tmp_Coeff.c=lane.v_PolyfitParam.v_ld_coeff[i].c;
			tmp_Coeff.d=lane.v_PolyfitParam.v_ld_coeff[i].d;
			tmp_Coeff.id=lane.v_PolyfitParam.v_ld_coeff[i].global_id;
			ld_obj.lane_Coeff.push_back(tmp_Coeff);

			cv::Point2f dot_p;
			std::vector<float> coeff(4);
			coeff[3] = tmp_Coeff.a;
			coeff[2] = tmp_Coeff.b;
			coeff[1] = tmp_Coeff.c;
			coeff[0] = tmp_Coeff.d;

			// for (double xx = -20; xx < 20;xx=xx+0.05) {
			dot_p.x = 0;
			dot_p.y =valueAtIPM(coeff, 0);
			if (-3.5<dot_p.y && dot_p.y<3.5){
				cv::circle(procImg,  cv::Point2f(480-dot_p.y*10,768-dot_p.x*10-384), 1, cv::Scalar(255, 0, 0), 1, 8, 0);//element 2: detected lines! valueAt is the fitted result		
								
				cv::line(img_visual,  cv::Point2f(480-dot_p.y*pixel_ratio-240,80),cv::Point2f(480-dot_p.y*pixel_ratio-240,275), cv::Scalar(255, 0, 0), 2, 8, 0);//element 2: detected lines! valueAt is the fitted result		
				
				lane_angle = atan(3*coeff[3]*dot_p.x*dot_p.x+2*coeff[2]*dot_p.x+coeff[1]);
				// dist_bias_1 = 1*sin(atan(coeff[1])) + coeff[0]*cos(atan(coeff[1])); //the distace in 1 meter
				// dist_bias_5 = 5*sin(atan(coeff[1])) + coeff[0]*cos(atan(coeff[1])); //the distace in 5 meter								
				candidate_dis.push_back(dot_p.y);
				candidate_id.push_back(i);
				Lane_angle.push_back(lane_angle);				
				// ROS_INFO("dot_p %f",dot_p.y);
			}
			//candidate_dis.push_back(0);
		}
		// cv::line(img_visual,  cv::Point2f(480-dot_p.y*pixel_ratio-240,64),cv::Point2f(480-dot_p.y*pixel_ratio-240,256), cv::Scalar(255, 0, 0), 2, 8, 0);//element 2: detected lines! valueAt is the fitted result		
		// cv::line(img_visual,  cv::Point2f(480-dot_p.y*pixel_ratio-240,64),cv::Point2f(480-dot_p.y*pixel_ratio-240,256), cv::Scalar(255, 0, 0), 2, 8, 0);//element 2: detected lines! valueAt is the fitted result		

		// float angle_final = (Lane_angle[1] + Lane_angle[0])/2;
		// float dist1_final = (Dist1[1] + Dist1[0])/2;
		// float dist5_final = (Dist5[1] + Dist5[0])/2;

		// ROS_INFO("angle_final %f",angle_final);
		// ROS_INFO("dist1_final %f",dist1_final);
		// ROS_INFO("dist5_final %f",dist5_final);

		cv::circle(procImg,  cv::Point2f(480-0*10,768-1*10-384), 2, cv::Scalar(255, 0, 0), 1, 8, 0);//element 2: detected lines! valueAt is the fitted result		
		cv::circle(procImg,  cv::Point2f(480-3*10,768-1*10-384), 2, cv::Scalar(255, 0, 0), 1, 8, 0);//element 2: detected lines! valueAt is the fitted result		
		cv::circle(procImg,  cv::Point2f(480+3*10,768-1*10-384), 2, cv::Scalar(255, 0, 0), 1, 8, 0);//element 2: detected lines! valueAt is the fitted result		


		cv::putText(img_visual_static,"DIAS LDW",cv::Point(15,25),cv::FONT_HERSHEY_DUPLEX,0.6, CV_RGB(0,140,255),1.9,8,0);
		cv::line(img_visual_static,  cv::Point2f(110,70),cv::Point2f(110,206), cv::Scalar(255, 0, 0), 2, 8, 0);//element 2: detected lines! valueAt is the fitted result		
		cv::line(img_visual_static,  cv::Point2f(210,70),cv::Point2f(210,206), cv::Scalar(255, 0, 0), 2, 8, 0);//element 2: detected lines! valueAt is the fitted result		

		double left_0 = 0.0;
		double right_0 = 0.0;
		int flag_left=0;
		int flag_right=0;
		//ROS_INFO("candidate_dis.size %d",candidate_dis.size());

		bool b_flag1=true;
		bool b_flag2=true;

		int i_flag_l=-1;
		int i_flag_r=-1;

		if (candidate_dis.size()>1){
			for(int ii=0;ii<candidate_dis.size();ii++){
				if(candidate_dis[ii]<0){
					if(b_flag1){
					left_0 = candidate_dis[ii];
					i_flag_l=ii;
					}
					b_flag1=false;

					if(candidate_dis[ii]>left_0){
					left_0 =candidate_dis[ii];
					i_flag_l=ii;
					}
					flag_left=1;
				}
				if(candidate_dis[ii]>0){
					if(b_flag2){
					right_0 = candidate_dis[ii];
					i_flag_r=ii;
					}
					b_flag2=false;

					if(candidate_dis[ii]<right_0){
					right_0 =candidate_dis[ii];
					i_flag_r=ii;
					}	
					flag_right=1;
				}
			}
		}
		else
		{
			ROS_INFO("lane detection is not reliable OR no lane ");
		}
		
		double angle_final =0.0;
		double curve_radius = 0.0;
		float curve_position_x= 0.0;
		if(i_flag_l==-1 && i_flag_r!=-1){
			angle_final = Lane_angle[i_flag_r];
			curve_radius = fabs(pow(1.0+pow((ld_obj.lane_Coeff[candidate_id[i_flag_r]].c),2),(3/2))/(2*ld_obj.lane_Coeff[candidate_id[i_flag_r]].b));

		}else if(i_flag_l!=-1 && i_flag_r==-1) {
			angle_final = Lane_angle[i_flag_l];
			curve_radius = fabs(pow(1.0+pow((ld_obj.lane_Coeff[candidate_id[i_flag_l]].c),2),(3/2))/(2*ld_obj.lane_Coeff[candidate_id[i_flag_l]].b));

		}else if(i_flag_l!=-1 && i_flag_r!=-1) {
			angle_final = (Lane_angle[i_flag_l]+Lane_angle[i_flag_r])/2;
			curve_radius = (fabs(pow(1.0+pow((ld_obj.lane_Coeff[candidate_id[i_flag_l]].c),2),(3/2))/(2*ld_obj.lane_Coeff[candidate_id[i_flag_l]].b))+fabs(pow(1.0+pow((ld_obj.lane_Coeff[candidate_id[i_flag_r]].c),2),(3/2))/(2*ld_obj.lane_Coeff[candidate_id[i_flag_r]].b)))/2.0;
		}
		if(curve_radius>3000){curve_radius=3000;}
		double mid = 0;
		double bias_dis = 0;
		double lane_width = 3.5;
		char tmp_str[20]="";

		cv::line(img_visual_static,  cv::Point2f(160,70),cv::Point2f(160,206), cv::Scalar(0, 255, 255), 1, 8, 0);//element 2: detected lines! valueAt is the fitted result		

		if(flag_left){
		cv::line(img_visual_static,  cv::Point2f(110,70),cv::Point2f(110,206), cv::Scalar(0, 255, 0), 2, 8, 0);//element 2: detected lines! valueAt is the fitted result		
		}
		if(flag_right){
		cv::line(img_visual_static,cv::Point2f(210,70),cv::Point2f(210,206), cv::Scalar(0, 255, 0), 2, 8, 0);//element 2: detected lines! valueAt is the fitted result					
		}
		if(flag_left && flag_right && (fabs(left_0)+fabs(right_0))* cos(angle_final)> 3.0)
		{
			mid =(left_0+right_0)/2;
			bias_dis = mid * cos(angle_final);
			lane_width = (fabs(left_0)+fabs(right_0))* cos(angle_final);
			//如果道路宽度有突变，则参考历史容器中的大小，做为纠正值
			if (fabs(lane_width-hist_v_LaneWidth[hist_v_LaneWidth.size()-1])>0.3)
			{
				for(int ii=0;ii<5;ii++){
				lane_width += hist_v_LaneWidth[hist_v_LaneWidth.size()-1-ii];
				}
				lane_width = lane_width/6.0;
			}
			//将道路宽度加入历史容器中
			vector_Update(hist_v_LaneWidth,lane_width);
			//连续预测标志为false
			predict_continue_flag = false;

			// ROS_INFO("BIAS DIS %f",mid);
			// ROS_INFO("BIAS DIS Vertical %f",bias_dis);
			// ROS_INFO("lane width1 %0.3f",(fabs(left_0)+fabs(right_0)));
			// ROS_INFO("lane width2 %0.3f",lane_width);
			// ROS_INFO("theta %f",angle_final);

			sprintf(tmp_str,"BIAS:%0.2f",bias_dis);
			
			if(bias_dis<-thres_ldw){
			cv::line(img_visual_static,  cv::Point2f(110,70),cv::Point2f(110,206), cv::Scalar(0, 0, 255), 2, 8, 0);//element 2: detected lines! valueAt is the fitted result		
			cv::circle(img_visual_static,  cv::Point2f(160+50*2*bias_dis/(lane_width),206), 7, cv::Scalar(0, 0, 255), 5, 8, 0);//element 2: detected lines! valueAt is the fitted result		
			}
			else if(bias_dis>thres_ldw){
			cv::line(img_visual_static,cv::Point2f(210,70),cv::Point2f(210,220), cv::Scalar(0, 0, 255), 2, 8, 0);//element 2: detected lines! valueAt is the fitted result		
			cv::circle(img_visual_static,   cv::Point2f(160+50*2*bias_dis/(lane_width),206), 7, cv::Scalar(0, 0, 255), 5, 8, 0);//element 2: detected lines! valueAt is the fitted result		
			}else{
			cv::circle(img_visual_static,   cv::Point2f(160+50*2*bias_dis/(lane_width),206), 7, cv::Scalar(0, 255, 20), 5, 8, 0);//element 2: detected lines! valueAt is the fitted result		
		//	cv::circle(img_visual_static,  cv::Point2f(160,190), 7,  cv::Scalar(0, 255, 20), 5, 8, 0);//element 2: detected lines! valueAt is the fitted result			
			}
		}else{
			cv::circle(img_visual_static,  cv::Point2f(160,206), 7,  cv::Scalar(0, 255, 20), 5, 8, 0);//element 2: detected lines! valueAt is the fitted result		
			for(int ii=0;ii<10;ii++){
			lane_width += hist_v_LaneWidth[hist_v_LaneWidth.size()-1-ii];
			}
			lane_width = lane_width/11.0;
			vector_Update(hist_v_LaneWidth,lane_width);
			predict_num++;
			predict_continue_flag =true;
		}
		if(!predict_continue_flag){
			predict_num = 0;
		}
		//连续预测次数超过五次，还未有真实检测值的时候，报警
		if(predict_continue_flag && predict_num>5)
		{//一些补尝措施
			
		}
	//根据道路宽进行车道线方程去伪。
		double judge_time1 = clock();
		JudgeCenter.SetParam(lane_width,lane_width*0.1,angle_final);
		JudgeCenter.Run(lane.v_PolyfitParam.v_ld_coeff);
		double judge_time2 = clock();
		std::cout << "judge_time " << (judge_time2 - judge_time1) << "mms" << std::endl;
		ld_obj.lane_Coeff.clear();
		for (int i = 0; i < lane.v_PolyfitParam.v_ld_coeff.size(); i++)
		{	
			tmp_Coeff.a=lane.v_PolyfitParam.v_ld_coeff[i].a;
			tmp_Coeff.b=lane.v_PolyfitParam.v_ld_coeff[i].b;
			tmp_Coeff.c=lane.v_PolyfitParam.v_ld_coeff[i].c;
			tmp_Coeff.d=lane.v_PolyfitParam.v_ld_coeff[i].d;
			tmp_Coeff.id=lane.v_PolyfitParam.v_ld_coeff[i].global_id;
			ld_obj.lane_Coeff.push_back(tmp_Coeff);

		}


		//std::cout << "hist_v_LaneWidth"<<std::endl;

		for (int ii = 0; ii < hist_v_LaneWidth.size()-1; ii++) {
			//std::cout << " "<<hist_v_LaneWidth[ii];
		}
		cv::line(img_visual,  cv::Point2f(480-bias_dis*pixel_ratio-240,80),cv::Point2f(480-bias_dis*pixel_ratio-240,275), cv::Scalar(0, 230, 0), 1, 8, 0);//element 2: detected lines! valueAt is the fitted result		
		cv::putText(img_visual,"DIAS LDW",cv::Point(15,30),cv::FONT_HERSHEY_DUPLEX,0.8, CV_RGB(0,140,255),1.9,8,0);
		cv::circle(img_visual,  cv::Point2f(480-0*pixel_ratio-240,275), 8, cv::Scalar(0, 30, 255), 8, 8, 0);//element 2: detected lines! valueAt is the fitted result		
		cv::putText(img_visual,tmp_str,cv::Point(190,70),cv::FONT_HERSHEY_DUPLEX,0.8, CV_RGB(255,120,0),1.9,8,0);
	
		cv::putText(img_visual_static,tmp_str,cv::Point(110,55),cv::FONT_HERSHEY_DUPLEX,0.6, CV_RGB(255,120,0),1.9,8,0);



	//  ld_obj.lane_Coeff.assign(lane.v_PolyfitParam.v_ld_coeff.begin(),lane.v_PolyfitParam.v_ld_coeff.end());
	//	ld_obj.lane_Pixel.assign(lane.v_PolyfitParam.v_WholeLane_Pixel.begin(),lane.v_PolyfitParam.v_WholeLane_Pixel.end());
	//	ld_obj.lane_World.assign(lane.v_PolyfitParam.v_WholeLane_World.begin(),lane.v_PolyfitParam.v_WholeLane_World.end());
		ld_obj.header.stamp=ros::Time::now();
		ld_obj.lane_width =lane_width ;
		ld_obj.bias_dis = bias_dis;
		ld_obj.cl_flag = flag_left && flag_right ;
		ld_obj.bias_theta = angle_final;
		ld_obj.curve_radius =curve_radius;

		pub_frame.publish(ld_obj);

		//cv::imshow("img_visual",img_visual);	
		// cv::imshow("img_visual_static",img_visual_static);	
	
		//if (verbose) {
		//			cv::waitKey(0);
		//		}
		cv::waitKey(1);
		std::cout << "new circle" << std::endl;
		double main_time2 = clock();
		std::cout << "main_time " << (main_time2 - main_time1) << "mms" << std::endl;
		std::cout << "FPS: " << 1000000 / (main_time2 - main_time1) << std::endl;
		hist_time.push_back(main_time2 - main_time1);
		double sum_hist_time = std::accumulate(std::begin(hist_time), std::end(hist_time), 0.0);
		double mean_hist_time = sum_hist_time / hist_time.size(); //均值
		std::cout << "mean_hist_time " << mean_hist_time << "mms" << std::endl;
		std::cout << "mean_FPS: " << 1000000 / mean_hist_time << std::endl;
		//cv::imshow("result",proImg);
		//}
		//std::cout << "ebd" << std::endl;
		//ld.~LaneDetection();
		return 0;
	}
};




int main(int argc, char **argv)
{ //
	ros::init(argc, argv, "lane_detection", ros::init_options::AnonymousName);
	hist_v_LeftDis.clear();
	hist_v_RightDis.clear();
	hist_v_LaneWidth.clear();
	hist_v_RadiusOfCurve.clear();
	vector_InitValue(hist_v_LaneWidth,3.5,10);

	ImageConverter ic;  //类ImageConverter、对象ic

	ros::spin();  //ROS消息回调处理函数
	return 0;
}


