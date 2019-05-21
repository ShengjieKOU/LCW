#include <string>
#include <sstream>
#include <stdlib.h>

#include <opencv2/opencv.hpp>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <opencv2/imgproc/imgproc.hpp>		 //图像处理
#include <opencv2/highgui/highgui.hpp>		 //opencv GUI

#include "ldw/ld_Coeff.h"
#include "ldw/ld_Frame.h"
#include "ldw/ld_LaneParam.h"
#include "ldw/ld_Point.h"
#include "ldw/ldw_Final.h"

ros::Publisher pub_ldw;

ldw::ldw_Final ldw_out; //output message
//warning_mode: 0: two ego lanes are detected; 1: only one ego lane is detected; 2: no ego lanes; 4: too narrow
//warning: 0: no warning; 1: right warning; 2: left warning; 3: no ego lanes or too narrow

double thres_ldw = 0.5; //符合国标要求, TODO: 随横向速度变化的阈值
double lane_angle;
double angle_final;
std::vector<float> hist_v_LaneWidth;
bool predict_continue_flag;
int predict_num = 0;


double valueAtIPM(std::vector<float> &f, float x)
{
    float ans = 0.f;
    for (int i = (int)f.size() - 1; i >= 0; --i)
        ans = ans * x + f[i];
    return ans;
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

void vector_Update(std::vector<float> &vector_a, float value)
{
    vector_a.push_back(value);
    vector_a.erase(vector_a.begin());
}

static void ldw_callback(const ldw::ld_Frame &msg)
{
    std::vector<double> candidate_dis;
    std::vector<int> candidate_id;
    std::vector<float> Lane_angle(2);

    candidate_id.clear();
    candidate_dis.clear();
    Lane_angle.clear();

    //read lane data
    std::cout << "Now we enter the lane callback" << std::endl;
    std::cout << msg.lane_Coeff.size() << std::endl;
    for(int i=0;i<msg.lane_Coeff.size();i++){
        std::cout << " b: " << msg.lane_Coeff[i].b << " c: " << msg.lane_Coeff[i].c << " d: " << msg.lane_Coeff[i].d << std::endl;
        cv::Point2f dot_p;
        std::vector<float> coeff(4);
        coeff[3] = msg.lane_Coeff[i].a;
        coeff[2] = msg.lane_Coeff[i].b;
        coeff[1] = msg.lane_Coeff[i].c;
        coeff[0] = msg.lane_Coeff[i].d;

        dot_p.x = 0;
        dot_p.y = valueAtIPM(coeff, 0);
        if (-3.5 < dot_p.y && dot_p.y < 3.5)
        {
            lane_angle = atan(3 * coeff[3] * dot_p.x * dot_p.x + 2 * coeff[2] * dot_p.x + coeff[1]);
            // dist_bias_1 = 1*sin(atan(coeff[1])) + coeff[0]*cos(atan(coeff[1])); //the distace in 1 meter
            // dist_bias_5 = 5*sin(atan(coeff[1])) + coeff[0]*cos(atan(coeff[1])); //the distace in 5 meter
            candidate_dis.push_back(dot_p.y*cos(lane_angle));    //distance should be calculated according to dis and angle
            candidate_id.push_back(i);
            Lane_angle.push_back(lane_angle);
        }
    }
    std::cout << "lane data reading finished!" << std::endl;

    //warning
    cv::Mat ldw_visual; //ldw display
    ldw_visual.create(240, 320, CV_8UC3);
    ldw_visual.setTo(0);
    cv::line(ldw_visual, cv::Point2f(110, 70), cv::Point2f(110, 206), cv::Scalar(255, 0, 0), 2, 8, 0); //initial blue lane, right and left
    cv::line(ldw_visual, cv::Point2f(210, 70), cv::Point2f(210, 206), cv::Scalar(255, 0, 0), 2, 8, 0);
    cv::line(ldw_visual, cv::Point2f(160, 70), cv::Point2f(160, 206), cv::Scalar(0, 100, 0), 1, 8, 0); //center line

    double right_0 = 0.0;
    double left_0 = 0.0;
    int flag_right = 0;
    int flag_left = 0;

    bool b_flag1 = true; //flag for initializing right lane
    bool b_flag2 = true; //flag for initializing left lane

    int i_flag_l = -1;
    int i_flag_r = -1;

    //finding the nearest right lane and the nearest left lane
    if (candidate_dis.size() > 1){
        for (int ii = 0; ii < candidate_dis.size(); ii++){
            if (candidate_dis[ii] < 0){ //right
                if (b_flag1){
                    right_0 = candidate_dis[ii];
                    i_flag_l = ii;
                }
                b_flag1 = false;

                if (candidate_dis[ii] > right_0)
                {
                    right_0 = candidate_dis[ii];
                    i_flag_l = ii;
                }
                flag_right = 1;
            }
            if (candidate_dis[ii] > 0){
                if (b_flag2){
                    left_0 = candidate_dis[ii];
                    i_flag_r = ii;
                }
                b_flag2 = false;

                if (candidate_dis[ii] < left_0){
                    left_0 = candidate_dis[ii];
                    i_flag_r = ii;
                }
                flag_left = 1;
            }
        }
    }
    else
    {
        ROS_INFO("lane detection is not reliable OR no lane ");
    }

    double mid = 0; //y value of the center of the ego lane
    double right_dis;
    double left_dis;
    double bias_dis = 0;
    double lane_width = 3.5;
    char tmp_str[20] = ""; //distance output in display
    
    if (flag_right)
    {
        cv::line(ldw_visual, cv::Point2f(110, 70), cv::Point2f(110, 206), cv::Scalar(0, 255, 0), 2, 8, 0);
    }
    if (flag_left)
    {
        cv::line(ldw_visual, cv::Point2f(210, 70), cv::Point2f(210, 206), cv::Scalar(0, 255, 0), 2, 8, 0);
    }
    std::cout << "Ego lane established!" << std::endl;
    std::cout << "left_0: " << left_0 << " right_0: " << right_0 << std::endl;
    std::cout << "flag_left: " << flag_left << " flag_right: " << flag_right << std::endl;


    //LDW输出
    //正常的双边状态
    if (flag_right && flag_left && (fabs(right_0) + fabs(left_0)) * cos(angle_final) > 3.0) //ego lane width more than 3.0, valid detection
    {
        ldw_out.warning_mode = 0;

        std::cout << "two sides" << std::endl;
        mid = (right_0 + left_0) / 2;
        bias_dis = mid * cos(angle_final);
        right_dis = fabs(right_0)*cos(angle_final);
        left_dis = fabs(left_0)*cos(angle_final);
        lane_width = (fabs(right_0) + fabs(left_0)) * cos(angle_final);
        //如果道路宽度有突变，则参考历史容器中的大小，做为纠正值
        if (fabs(lane_width - hist_v_LaneWidth[hist_v_LaneWidth.size() - 1]) > 0.3)
        {
            for (int ii = 0; ii < 5; ii++)
            {
                lane_width += hist_v_LaneWidth[hist_v_LaneWidth.size() - 1 - ii];
            }
            lane_width = lane_width / 6.0;
        }
        
        //将道路宽度加入历史容器中
        vector_Update(hist_v_LaneWidth, lane_width);
        //连续预测标志为false
        predict_continue_flag = false;
        predict_num=0;

        ROS_INFO("lane width %0.3f", lane_width);
        ROS_INFO("theta %f", angle_final);

        sprintf(tmp_str, "right:%0.2f left:%0.2f", right_dis, left_dis);
        cv::putText(ldw_visual, tmp_str, cv::Point(70, 55), cv::FONT_HERSHEY_DUPLEX, 0.6, CV_RGB(255, 0, 0), 1.9, 8, 0);

        if (right_dis < thres_ldw)
        {
            cv::line(ldw_visual, cv::Point2f(210, 70), cv::Point2f(210, 206), cv::Scalar(0, 0, 255), 2, 8, 0); //红色为报警
            cv::circle(ldw_visual, cv::Point2f(160 + 50 * 2 * bias_dis / (lane_width), 206), 7, cv::Scalar(0, 0, 255), 5, 8, 0);
            ldw_out.warning = 1;
            ldw_out.left_dis = left_dis;
            ldw_out.right_dis = right_dis;
            ldw_out.predict_num = 0;
        }
        else if (left_dis < thres_ldw)
        {
            cv::line(ldw_visual, cv::Point2f(110, 70), cv::Point2f(110, 220), cv::Scalar(0, 0, 255), 2, 8, 0); 
            cv::circle(ldw_visual, cv::Point2f(160 + 50 * 2 * bias_dis / (lane_width), 206), 7, cv::Scalar(0, 0, 255), 5, 8, 0); 
            ldw_out.warning = 2;
            ldw_out.left_dis = left_dis;
            ldw_out.right_dis = right_dis;
            ldw_out.predict_num = 0;
        }
        else
        {
            cv::circle(ldw_visual, cv::Point2f(160 + 50 * 2 * bias_dis / (lane_width), 206), 7, cv::Scalar(0, 255, 20), 5, 8, 0);
            ldw_out.warning = 0;
            ldw_out.left_dis = left_dis;
            ldw_out.right_dis = right_dis;
            ldw_out.predict_num = 0;
        }
    }
    else
    {
        std::cout << "Not normal two sides..." << std::endl;
        //现在是不准确的，预测状态
        for (int ii = 0; ii < 10; ii++)
        {
            lane_width += hist_v_LaneWidth[hist_v_LaneWidth.size() - 1 - ii];
        }
        lane_width = lane_width / 11.0;
        vector_Update(hist_v_LaneWidth, lane_width);
        predict_num++;
        predict_continue_flag = true;

        //单边
        if((flag_left && !flag_right) || (!flag_right && flag_left)){
            ldw_out.warning_mode = 1;

            std::cout << "One side" << std::endl;
            char predict_str[20] = "";
            if(flag_left){
                right_0=left_0-3.5;
                sprintf(predict_str, "right predict times: %d", predict_num);
            }
            else{
                left_0=right_0+3.5;
                sprintf(predict_str, "left predict times: %d", predict_num);
            }
            mid = (right_0 + left_0) / 2;
            bias_dis = mid * cos(angle_final);
            right_dis = fabs(right_0) * cos(angle_final);
            left_dis = fabs(left_0) * cos(angle_final);

            if (right_dis < thres_ldw)
            {
                cv::line(ldw_visual, cv::Point2f(210, 70), cv::Point2f(210, 206), cv::Scalar(0, 0, 255), 2, 8, 0); //红色为报警
                cv::circle(ldw_visual, cv::Point2f(160 + 50 * 2 * bias_dis / (lane_width), 206), 7, cv::Scalar(0, 0, 255), 5, 8, 0);
                ldw_out.warning = 1;
                ldw_out.left_dis = left_dis;
                ldw_out.right_dis = right_dis;
                ldw_out.predict_num = 0;
            }
            else if (left_dis < thres_ldw)
            {
                cv::line(ldw_visual, cv::Point2f(110, 70), cv::Point2f(110, 220), cv::Scalar(0, 0, 255), 2, 8, 0);
                cv::circle(ldw_visual, cv::Point2f(160 + 50 * 2 * bias_dis / (lane_width), 206), 7, cv::Scalar(0, 0, 255), 5, 8, 0);
                ldw_out.warning = 2;
                ldw_out.left_dis = left_dis;
                ldw_out.right_dis = right_dis;
                ldw_out.predict_num = 0;
            }
            else
            {
                cv::circle(ldw_visual, cv::Point2f(160 + 50 * 2 * bias_dis / (lane_width), 206), 7, cv::Scalar(0, 255, 20), 5, 8, 0);
                ldw_out.warning = 0;
                ldw_out.left_dis = left_dis;
                ldw_out.right_dis = right_dis;
                ldw_out.predict_num = 0;
            }

            sprintf(tmp_str, "right:%0.2f left:%0.2f", right_dis, left_dis);
            cv::putText(ldw_visual, tmp_str, cv::Point(70, 55), cv::FONT_HERSHEY_DUPLEX, 0.6, CV_RGB(255, 0, 0), 1.9, 8, 0);

            cv::putText(ldw_visual, predict_str, cv::Point(70, 35), cv::FONT_HERSHEY_DUPLEX, 0.6, CV_RGB(255, 0, 0), 1.9, 8, 0);
        }
        else{
            //无有效车道线或双边车道线太近
            std::cout << "System fail!" << std::endl;
            if(!flag_right && !flag_left){
                sprintf(tmp_str, "No ego lanes");
                ldw_out.warning_mode = 2;
                ldw_out.warning = 3;
                ldw_out.left_dis = 0;
                ldw_out.right_dis = 0;
                ldw_out.predict_num = 0;
            }
            else{
                sprintf(tmp_str, "Too narrow lanes");
                ldw_out.warning_mode = 3;
                ldw_out.warning = 3;
                ldw_out.left_dis = 0;
                ldw_out.right_dis = 0;
                ldw_out.predict_num = 0;
            }
            
            cv::putText(ldw_visual, tmp_str, cv::Point(70, 55), cv::FONT_HERSHEY_DUPLEX, 0.6, CV_RGB(255, 0, 0), 1.9, 8, 0);
            
        }
    }

    // 连续预测次数超过五次，还未有真实检测值的时候，报警
    // if (predict_continue_flag && predict_num > 5)
    // {
    //     std::cout << "System fail!" << std::endl;
    //     sprintf(tmp_str, "Prediction fail");
    //     cv::putText(ldw_visual, tmp_str, cv::Point(70, 55), cv::FONT_HERSHEY_DUPLEX, 0.6, CV_RGB(255, 0, 0), 1.9, 8, 0);
    //     //TODO: rostopic output
    // }
    std::cout << "hist_v_LaneWidth, size: " << hist_v_LaneWidth.size() << std::endl;

    for (int ii = 0; ii < hist_v_LaneWidth.size() - 1; ii++)
    {
        std::cout << " " << hist_v_LaneWidth[ii];
    }

    
    cv::imshow("ldw_visual", ldw_visual);

    pub_ldw.publish(ldw_out);

    cv::waitKey(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fusion");

    ros::NodeHandle nh;
    std::cout << "begin the ldw" << std::endl;

    hist_v_LaneWidth.clear();
    vector_InitValue(hist_v_LaneWidth, 3.5, 10);

    /*lanes*/
    ros::Subscriber lidar_front_obs3d_sub = nh.subscribe("lane_data", 1, ldw_callback);

    pub_ldw = nh.advertise<ldw::ldw_Final>("ldw_final", 1000);

    ros::spin();

    return 0;
}