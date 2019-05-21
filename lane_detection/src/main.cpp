#include <iostream>
#include <string>
#include <sstream>
// #include <afx.h> 
#include <opencv2/opencv.hpp>
#include "LaneDetection.h"

//using namespace cv;

int main(){
	
	// input parameter
	bool verbose_lm_detction = false;  //ȷ�������߿�ȣ�����ɸѡ
	bool verbose_seed_gen = true;    //����ͬ���߶���ϳ�һ��seed
	bool verbose_run_crf = false;    //���Ӵ���ͬһ��ֱ���ϵ�seed
	bool verbose_validating = false; //ɸѡ�����
	bool verbose = verbose_lm_detction | verbose_seed_gen | verbose_run_crf | verbose_validating;
	bool verbose_kalman = true;

	//<-------------- Common Variables definition & initialization --------------> 
	//std::string img_path = "../data/20120322142840/";
	//std::string img_path = "../data/test_data/";
    //std::string img_path = "../data/64140/";

//-----------------2018.03.05--------------------------------
    //cv::VideoCapture capture("D:\\lane_detection\\autodriving.avi");
	//cv::VideoCapture capture("/home/jxy/CRF5.5/lane.avi");
	cv::Mat procImg;
	bool judgeresult;

	int count = 1;
	int count_initial = count;
	LaneDetection ld = LaneDetection();
	ld.setcount(count);

	clock_t begin, end;
	clock_t begintotal, endtotal;

	begintotal = clock();

	while (count < 1000)
	{
		begin = clock();
		cv::Mat srcImage,tmpImage;
		//capture >> srcImage;
		count++;
		std::cout << count << std::endl;
		std::stringstream path;
		path << "/home/jxy/srcImage/" << count << ".jpg";
		//path << "/home/jxy/LaneDetect-master7.17/frame0000.jpg";
		srcImage = cv::imread(path.str());

		//if(count<1000){
		//	std::cout << count << std::endl;
		//	continue;
		//}

	//	cv::resize(srcImage, srcImage, cv::Size(srcImage.cols/2, srcImage.rows / 2));
		cv::resize(srcImage, srcImage, cv::Size(768, 480));
		// tmpImage = srcImage.clone();
		// cv::Mat mask = cv::Mat(tmpImage.size(), CV_8UC3);
		// std::vector<std::vector<cv::Point> > contour;
		// std::vector<cv::Point> pts;
		// pts.push_back(cv::Point(0, 0));
		// pts.push_back(cv::Point(0, 480));
		// pts.push_back(cv::Point(150, 480));
		// pts.push_back(cv::Point(150, 410));
		// pts.push_back(cv::Point(620, 410));
		// pts.push_back(cv::Point(620, 480));
		// pts.push_back(cv::Point(768, 480));
		// pts.push_back(cv::Point(768, 0));
/*
		pts.push_back(cv::Point(0, 0));
		pts.push_back(cv::Point(0, 480));
		pts.push_back(cv::Point(146, 480));
		pts.push_back(cv::Point(244, 480));
		pts.push_back(cv::Point(530, 480));
		pts.push_back(cv::Point(620, 480));
		pts.push_back(cv::Point(768, 480));
		pts.push_back(cv::Point(768, 0));
*/

		// contour.push_back(pts);
		// drawContours(mask, contour, 0, cv::Scalar(255,255,255), -1);
		//tmpImage.copyTo(srcImage, mask);
		//srcImage = tmpImage&mask;   //intersect
		
		if (!ld.initialize_variable(srcImage)) {
			return -1;
		}
		if (!ld.initialize_Img(srcImage)) {
			return -1;
		}

		//std::cout << "lane_marking_detection" << std::endl;
		ld.lane_marking_detection(verbose_lm_detction);

		// supermarking generation and low-level association
		ld.seed_generation(verbose_seed_gen);

		// CRF graph configuration & optimization using hungarian method
		ld.graph_generation(verbose_run_crf);

		// validating
		procImg = ld.validating_final_seeds(verbose_validating).clone();

        //kalman
		//std::cout << "kalman" << std::endl;
		ld.kalman(verbose_kalman);

		//TODO: get ld parameters. Needs setcoeffs first.
		//ld.getcoeffs();

		//judge a pixel is a lane pixel or not
		judgeresult = ld.judgepixel(311,268,procImg); //waiting for input
		//std::cout << "judgeresult: " << judgeresult << std::endl;

		cv::Mat Xm = ld.Xmoutput(); //equations output, each 3 numbers in sequence represents a quadratic equation of a line.
		//For detailed usage, see LaneDetection::kalman in LaneDetection.cpp.

		cv::waitKey(1);

		end = clock();
		std::cout << "time: " << double(end - begin) / CLOCKS_PER_SEC << "ms" << std::endl;
	}

	endtotal = clock();
	std::cout << "average time: " << (double(endtotal - begintotal) / CLOCKS_PER_SEC)/(count-count_initial) << "ms" << std::endl;

	std::cout << "ebd" << std::endl;
	//ld.~LaneDetection();
	system("pause");
	return 0;
	
}
