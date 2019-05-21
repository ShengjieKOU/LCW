#include "common.h"
#include <string>
#include <visualization_msgs/MarkerArray.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/Float32.h"

using namespace dias::adas;

dias::adas::CCommon CCommonUnit;
dias::adas::CFunc_lcw CLcwUnit;

ros::Publisher pub_classified_targets;
ros::Publisher pub_classified_targets_label;

ros::Publisher pub_danger_targets;
ros::Publisher pub_danger_targets_label;

ros::Publisher pub_lcw_flag;
ros::Publisher pub_lcw_level;

ros::Publisher pub_chosen;

//发布车辆距离信息
ros::Publisher pub_dist_lf ;
ros::Publisher pub_safe_dist_lf ;
ros::Publisher pub_dist_lr ;
ros::Publisher pub_safe_dist_lr ;
ros::Publisher pub_dist_rf ;
ros::Publisher pub_safe_dist_rf ;
ros::Publisher pub_dist_rr ;
ros::Publisher pub_safe_dist_rr ;

ros::Publisher pub_ttc_lf ;
ros::Publisher pub_ttc_lr ;
ros::Publisher pub_ttc_rf ;
ros::Publisher pub_ttc_rr ;

std_msgs::Float32 dist_now_lf ;
std_msgs::Float32 dist_safe_lf ;
std_msgs::Float32 dist_now_lr ;
std_msgs::Float32 dist_safe_lr ;
std_msgs::Float32 dist_now_rf ;
std_msgs::Float32 dist_safe_rf ;
std_msgs::Float32 dist_now_rr ;
std_msgs::Float32 dist_safe_rr ;

std_msgs::Float32 ttc_lf ;
std_msgs::Float32 ttc_lr ;
std_msgs::Float32 ttc_rf ;
std_msgs::Float32 ttc_rr ;


ros::Time lane_timestamp ;
ros::Time fusion_timestamp;

double lane_update_delta = 0.0;
double fusion_update_delta = 0.0;
double pre_warining_level[5]={1,1,1,1,1};


cv::Mat lcw_visual; 
cv::Mat car ;
   

//用矩形框在RVIZ中显示危险车道
void pub_lcw(std::vector<float> warning_flag){ 
    visualization_msgs::MarkerArray array,array_text;
    visualization_msgs::Marker marker;
    marker.header.frame_id ="fusionData";    
    marker.header.stamp = ros::Time::now();
    marker.ns = "my_namespace";
   
    for (int i = 0; i < warning_flag.size(); i++){
        marker.id = i;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        if(i==0){
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0.1;
        marker.scale.x = 20;
        marker.scale.y = 3.5;
        marker.scale.z = 0.3;
        marker.color.a = 0.2; 
        }else if(i==1){
        marker.pose.position.x = 5;
        marker.pose.position.y =3.5;
        marker.pose.position.z = 0.1;
        marker.scale.x = 10;
        marker.scale.y = 3.5;
        marker.scale.z = 0.3;
        marker.color.a = 0.2; 
        }else if(i==2){
        marker.pose.position.x = -5;
        marker.pose.position.y = 3.5;
        marker.pose.position.z = 0.1;
        marker.scale.x = 10;
        marker.scale.y = 3.5;
        marker.scale.z = 0.3;
        marker.color.a = 0.2; 
        }else if(i==3){
        marker.pose.position.x = 5;
        marker.pose.position.y = -3.5;
        marker.pose.position.z = 0.1;
        marker.scale.x = 10;
        marker.scale.y = 3.5;
        marker.scale.z = 0.3;
        marker.color.a = 0.2; 
        }else if(i==4){
        marker.pose.position.x = -5;
        marker.pose.position.y = -3.5;
        marker.pose.position.z = 0.1;
        marker.scale.x = 10;
        marker.scale.y = 3.5;
        marker.scale.z = 0.3;
        marker.color.a = 0.2;
        }

        marker.color.r = 1 - warning_flag[i];
        marker.color.g = warning_flag[i];
        marker.color.b = 0.0; 
        marker.lifetime = ros::Duration(0.25);  
     
        array.markers.push_back(marker);

        char strTmp[90];
        sprintf(strTmp, "%0.2f", warning_flag[i]);		
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.color.r = 0;
        marker.color.g = 0;
        marker.color.b = 1; 
        marker.color.a = 1; 
        marker.pose.position.z = 5.5;
        marker.scale.z = 0.6;
        marker.text = strTmp;
        array_text.markers.push_back(marker);
    }
    pub_lcw_flag.publish(array);
    pub_lcw_level.publish(array_text);
}

//用圆点在RVIZ中显示危险目标
void pub_danger(dias::adas::TARGETS_INFO danger_targets){ 
    visualization_msgs::MarkerArray array,array_text;
    visualization_msgs::Marker marker;
    marker.header.frame_id ="fusionData";    
    marker.header.stamp = ros::Time::now();
    marker.ns = "my_namespace";
    for (int i = 0; i < danger_targets._target_info.size(); i++){
        marker.id = danger_targets._target_info[i]._id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = danger_targets._target_info[i]._obs_position._x;
        marker.pose.position.y = danger_targets._target_info[i]._obs_position._y;
        marker.pose.position.z = 2;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 2;
        marker.scale.y = 2;
        marker.scale.z = 2;
        marker.color.a = 1.0; 
        marker.color.r = 1;
        marker.color.g = 0.0;
        marker.color.b = 0.0;   
        marker.lifetime = ros::Duration(0.15); 


        if(marker.pose.position.x !=3.7 && !(marker.pose.position.x>0 && marker.pose.position.x<3))
        {
            array.markers.push_back(marker);
            char strTmp[90];
            sprintf(strTmp, "ID:%d,X:%0.2f,Y:%0.2f,vx:%0.2f,vy:%0.2f",
                danger_targets._target_info[i]._id,
                danger_targets._target_info[i]._obs_position._x,
                danger_targets._target_info[i]._obs_position._y,
                danger_targets._target_info[i]._velocity._x,
                danger_targets._target_info[i]._velocity._y);		
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.pose.position.z = 5.5;
            marker.scale.z = 0.6;
            marker.text = strTmp;
            array_text.markers.push_back(marker);
        }
    }
    pub_danger_targets.publish(array);
    pub_danger_targets_label.publish(array_text);

}

//显示划分车道后的目标，不同车道的目标用不同颜色表示
void publish_visualization(){

    visualization_msgs::MarkerArray array,array_text;
    visualization_msgs::Marker marker;
    marker.header.frame_id ="fusionData"; 
    marker.header.stamp = ros::Time::now();
    marker.ns = "my_namespace";
    if(CCommonUnit._Vec_TargetsInfo_Classfied.size() !=0){
        for(int jj=0;jj<CCommonUnit._Vec_TargetsInfo_Classfied.size();jj++){
            for (int i = 0; i < CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info.size(); i++){
                marker.id = CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[i]._id;
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[i]._obs_position._x;
                marker.pose.position.y = CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[i]._obs_position._y;
                marker.pose.position.z = 2;

                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.8;
                marker.scale.y = 0.8;
                marker.scale.z = 0.8;
                marker.color.a = 1.0; 
                if(jj==0){
                marker.color.r = 0;
                marker.color.g = 1.0;
                marker.color.b = 0.0; 
                }
                else if(jj==1){
                marker.color.r = 0;
                marker.color.g = 0;
                marker.color.b = 1; 
                }
                else if(jj==2){
                marker.color.r = 1;
                marker.color.g = 0;
                marker.color.b = 0; 
                }
                else if(jj==3){
                marker.color.r = 1;
                marker.color.g = 1;
                marker.color.b = 0; 
                }
                else if(jj==4){
                marker.color.r = 1;
                marker.color.g = 0;
                marker.color.b = 1; 
                }
                else if(jj==5){
                marker.color.r = 0;
                marker.color.g = 1;
                marker.color.b = 1; 
                }

                marker.lifetime = ros::Duration(0.15);
                if(marker.pose.position.x !=3.7 && !(marker.pose.position.x>0 && marker.pose.position.x<3))
                {
                    array.markers.push_back(marker);
                    char strTmp[90];
                    sprintf(strTmp, "ID:%d,X:%0.2f,Y:%0.2f,vx:%0.2f,vy:%0.2f",
                        CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[i]._id,
                        CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[i]._obs_position._x,
                        CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[i]._obs_position._y,
                        CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[i]._velocity._x,
                        CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[i]._velocity._y);		
                    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                    marker.pose.position.z = 3.5;
                    marker.scale.z = 0.3;
                    marker.text = strTmp;
                    array_text.markers.push_back(marker);
                }
            }
        }
    }
    pub_classified_targets.publish(array);
    pub_classified_targets_label.publish(array_text);

}

//基于OpenCV的显示界面，车道用矩形框表示，危险车道变红，目标车用圆点表示
void WarningGui(std::vector<float> warning_flag, std::vector<dias::adas::TARGETS_INFO> danger_targets){

    
    cv::Mat tmp_visual = lcw_visual.clone();

    std::string str="Lane Change Warning" ;
    cv::putText(tmp_visual, str, cv::Point2f(20,15),cv::FONT_HERSHEY_DUPLEX,0.5,CV_RGB(0,0,255),0.1,8,0);
    cv::rectangle(tmp_visual,cv::Point2f(120,30),cv::Point2f(200,270),cv::Scalar(0,255,0),2,8,0);

    if (warning_flag[0]<1){
        cv::rectangle(tmp_visual,cv::Point2f(120,30),cv::Point2f(200,270),cv::Scalar(0,0,255),2,8,0);
        std::string str = "Danger Front" ;
        cv::putText(tmp_visual, str, cv::Point2f(120,290),cv::FONT_HERSHEY_DUPLEX,0.3,CV_RGB(255,0,0),0.2,8,0);
    }

    if (CCommonUnit.left_lane_flag){
    cv::rectangle(tmp_visual,cv::Point2f(40,30),cv::Point2f(120,270),cv::Scalar(0,255,0),2,8,0);
    if (warning_flag[1]<1 || warning_flag[2]<1 ){
        cv::rectangle(tmp_visual,cv::Point2f(40,30),cv::Point2f(120,270),cv::Scalar(0,0,255),2,8,0);
        std::string str = "Danger Left" ;
        cv::putText(tmp_visual, str, cv::Point2f(40,290),cv::FONT_HERSHEY_DUPLEX,0.3,CV_RGB(255,0,0),0.2,8,0);
    }
    }else {
        std::string str = "NO LEFT LANE" ;
        cv::putText(tmp_visual, str, cv::Point2f(40,290),cv::FONT_HERSHEY_DUPLEX,0.3,CV_RGB(255,0,0),0.2,8,0);
    }
    
    if (CCommonUnit.right_lane_flag){
    cv::rectangle(tmp_visual,cv::Point2f(200,30),cv::Point2f(280,270),cv::Scalar(0,255,0),2,8,0);
    if (warning_flag[3]<1 || warning_flag[4]<1 ){
        cv::rectangle(tmp_visual,cv::Point2f(200,30),cv::Point2f(280,270),cv::Scalar(0,0,255),2,8,0);
        std::string str = "Danger Right" ;
        cv::putText(tmp_visual, str, cv::Point2f(200,290),cv::FONT_HERSHEY_DUPLEX,0.3,CV_RGB(255,0,0),0.2,8,0);
    }
    }else {
        std::string str = "NO RIGHT LANE" ;
        cv::putText(tmp_visual, str, cv::Point2f(200,290),cv::FONT_HERSHEY_DUPLEX,0.3,CV_RGB(255,0,0),0.2,8,0);
    }

    for (int j=0;j<5;j++){
        if (danger_targets[j]._target_info.size()>0)
        {
            float x ;
            float y ;

            if (danger_targets[j]._target_info[0]._obs_position._x>20){
                x=30 ;
            }else if (danger_targets[j]._target_info[0]._obs_position._x<-20){
                x= 270 ;
            } else {
                x=150-6*danger_targets[j]._target_info[0]._obs_position._x ;
            }

            float l_dist ;
            float r_dist ;

            if(CCommonUnit._LaneDepValue < 0)
            {
                l_dist = CCommonUnit._LaneWidth/2  + CCommonUnit._LaneDepValue;
                r_dist = CCommonUnit._LaneWidth - l_dist ;
            }else{
                r_dist = CCommonUnit._LaneWidth/2 - CCommonUnit._LaneDepValue;
                l_dist = CCommonUnit._LaneWidth - r_dist ;
            }

            if (danger_targets[j]._target_info[0]._obs_position._y> l_dist){
                y=80 ;
            }else if (danger_targets[j]._target_info[0]._obs_position._y <-r_dist){
                y=240 ;
            }else {
                y=160 ;
                x=40;
            }

            if (warning_flag[j]<1){
                if (danger_targets[j]._target_info[0]._dist < 3)
                {
                    std::cout<< "Danger Vehicle in Blind Area !"<< std::endl ;
                    cv::circle(tmp_visual, cv::Point2f(y,x),10,cv::Scalar(0,0,255),-1,8,0);
                }
                else if (danger_targets[j]._target_info[0]._dist > 3)
                {
                    cv::circle(tmp_visual, cv::Point2f(y,x),5,cv::Scalar(0,0,255),2,8,0);
                }
            }else
            {
                if (danger_targets[j]._target_info[0]._dist < 3)
                {
                    std::cout<< "Danger Vehicle in Blind Area !"<< std::endl ;
                    cv::circle(tmp_visual, cv::Point2f(y,x),10,cv::Scalar(255,0,0),-1,8,0);
                }
                else if (danger_targets[j]._target_info[0]._dist > 3)
                {
                    cv::circle(tmp_visual, cv::Point2f(y,x),5,cv::Scalar(255,0,0),2,8,0);
                }
            }
            
        }
        
    }

    
    cv::imshow("tmp_visual", tmp_visual) ;
    cv::waitKey(1);


    std::ofstream filee ;
    filee.open("flag.txt",std::ios::app);
    if (!filee.is_open()){
        std::cout<<"Open file failure"<< std::endl ;
    }
    filee<< warning_flag[4] <<"\n" ;
    filee.close();
}

//显示系统追踪的目标，即TTC最小的目标
void publish_chosen(){

    visualization_msgs::MarkerArray array ;
    visualization_msgs::Marker marker;
    marker.header.frame_id ="fusionData"; 
    marker.header.stamp = ros::Time::now();
    marker.ns = "my_namespace";
    for (int jj=1;jj<5; jj++)
    {
        // std::ofstream file ;
        // file.open("info.txt",std::ios::app);
        // if (!file.is_open()){
        //     std::cout<<"Open file failure"<< std::endl ;
        // }
        // file<< CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._obs_position._x <<" " \
        // << CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._velocity._x << " " \
        // << CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._acceleration._x <<" " \
        // << CCommonUnit._VehSpeed._x<< " "<< CCommonUnit._VehAcceleration._x << " " \
        // <<warning.data << "\n" ;
        // file.close();
        
        if(CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info.size() !=0){    
            if (jj==1)
            {
                dist_now_lf.data = CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._dist ;
                dist_safe_lf.data = CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._safe_dist ;
                ttc_lf.data=(float)CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._dist /fabs(CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._velocity._x - CCommonUnit._VehSpeed._x);
                if (dist_now_lf.data>100)
                {
                    dist_now_lf.data = 100 ;
                }
                if (dist_safe_lf.data>100)
                {
                    dist_safe_lf.data = 100 ;
                }
                if (ttc_lf.data>20)
                {
                    ttc_lf.data = 20 ;
                }
            } 
            if (jj==2)
            {
                dist_now_lr.data = CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._dist ;
                dist_safe_lr.data = CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._safe_dist ;
                ttc_lr.data=(float)CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._dist /fabs(CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._velocity._x - CCommonUnit._VehSpeed._x);
                if (dist_now_lr.data>100)
                {
                    dist_now_lr.data = 100 ;
                }
                if (dist_safe_lr.data>100)
                {
                    dist_safe_lr.data = 100 ;
                }
                if (ttc_lr.data>20)
                {
                    ttc_lr.data = 20 ;
                }
            } 
            if (jj==3)
            {
                dist_now_rf.data = CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._dist ;
                dist_safe_rf.data = CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._safe_dist ;
                ttc_rf.data=(float)CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._dist /fabs(CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._velocity._x - CCommonUnit._VehSpeed._x);
                if (dist_now_rf.data>100)
                {
                    dist_now_rf.data = 100 ;
                }
                if (dist_safe_rf.data>100)
                {
                    dist_safe_rf.data = 100 ;
                }
                if (ttc_rf.data>20)
                {
                    ttc_rf.data = 20 ;
                }
            } 
            if (jj==4)
            {
                dist_now_rr.data = CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._dist ;
                dist_safe_rr.data = CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._safe_dist ;
                ttc_rr.data=(float)CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._dist /fabs(CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._velocity._x - CCommonUnit._VehSpeed._x);
                if (dist_now_rr.data>100)
                {
                    dist_now_rr.data = 100 ;
                }
                if (dist_safe_rr.data>100)
                {
                    dist_safe_rr.data = 100 ;
                }
                if (ttc_rr.data>20)
                {
                    ttc_rr.data = 20 ;
                }
            } 
                

            marker.id = CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._id;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._obs_position._x;
            marker.pose.position.y = CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._obs_position._y;
            marker.pose.position.z = 2;

            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.8;
            marker.scale.y = 0.8;
            marker.scale.z = 0.8;
            marker.color.a = 1.0; 
        
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;   
            marker.lifetime = ros::Duration(0.15); 

            array.markers.push_back(marker);

        }
    }
    pub_chosen.publish(array);

    pub_dist_lf.publish(dist_now_lf);
    pub_safe_dist_lf.publish(dist_safe_lf);
    pub_ttc_lf.publish(ttc_lf);

    pub_dist_lr.publish(dist_now_lr);
    pub_safe_dist_lr.publish(dist_safe_lr);
    pub_ttc_lr.publish(ttc_lr);

    pub_dist_rf.publish(dist_now_rf);
    pub_safe_dist_rf.publish(dist_safe_rf);
    pub_ttc_rf.publish(ttc_rf);

    pub_dist_rr.publish(dist_now_rr);
    pub_safe_dist_rr.publish(dist_safe_rr);
    pub_ttc_rr.publish(ttc_rr);

}


//接收融合信息
void callback_fusion_result(const adas_module::TrackArray msg){
    fusion_timestamp = ros::Time::now();
	dias::adas::OBSVector3d speed;
	dias::adas::OBSVector3d acc;
	float yaw;
	speed._x = msg.self_velocity.x;
	speed._y = msg.self_velocity.y;
	speed._z = msg.self_velocity.z;
	acc._x = msg.self_acceleration.x;
	acc._y = msg.self_acceleration.y;
	acc._z = msg.self_acceleration.z;
	yaw = msg.imu_pose.z;

	CCommonUnit.CarParam_set(speed,yaw,acc);
    CLcwUnit.CarParam_set(speed,yaw, acc);
	CCommonUnit.Update_Target_Status(msg);
    // CCommonUnit.Target_Classified_Fusion(msg);

    publish_visualization();
    
}

// 换道危险评估与预警
int warning_keep_num[5]={0,0,0,0,0};
int warning_keep_threshold = 200 ;
int warning_times[5]={0,0,0,0,0};
int warning_times_threshold = 0 ;

void lane_danger_check(){

    //选择TTC最小目标
    Select_Min_TTC(CCommonUnit._Vec_TargetsInfo_Classfied[0]);
    Select_Min_TTC(CCommonUnit._Vec_TargetsInfo_Classfied[1]);
    Select_Min_TTC(CCommonUnit._Vec_TargetsInfo_Classfied[2]);
    Select_Min_TTC(CCommonUnit._Vec_TargetsInfo_Classfied[3]);
    Select_Min_TTC(CCommonUnit._Vec_TargetsInfo_Classfied[4]);
    Select_Min_TTC(CCommonUnit._Vec_TargetsInfo_Classfied[5]);

    publish_chosen();

    // 分析换道危险
    //CLcwUnit.SelfLaneFront_SafeDist(CCommonUnit._Vec_TargetsInfo_Classfied[0]);
    CLcwUnit.TargetLaneFront_SafeDist(CCommonUnit._Vec_TargetsInfo_Classfied[1]);
    CLcwUnit.TargetLaneRear_SafeDist(CCommonUnit._Vec_TargetsInfo_Classfied[2]);
    CLcwUnit.TargetLaneFront_SafeDist(CCommonUnit._Vec_TargetsInfo_Classfied[3]);
    CLcwUnit.TargetLaneRear_SafeDist(CCommonUnit._Vec_TargetsInfo_Classfied[4]);
 
    
    
    //判断有无车道
    std::vector<float> a;
    for (int ii=0;ii<5;ii++){
        a.push_back(1);
    }

    if (CCommonUnit.left_lane_flag==false){
        a[1]=0 ;
        a[2]=0 ;
        // std::cout<< "----no left lane----"<< std::endl ;
    }

    if (CCommonUnit.right_lane_flag==false){
        a[3]=0;
        a[4]=0;
        // std::cout<< "----no right lane----"<<std::endl;
    }

    int index ;
    
    std::vector<dias::adas::TARGETS_INFO> temp;   //存储危险目标，记录最危险系数（每帧重置）
    dias::adas::TARGETS_INFO tmp_target ;
    temp.push_back(tmp_target);
    temp.push_back(tmp_target);
    temp.push_back(tmp_target);
    temp.push_back(tmp_target);
    temp.push_back(tmp_target);
    //输出危险目标状态信息
    for(int jj=0;jj<5;jj++){
        if(CCommonUnit._Vec_TargetsInfo_Classfied[jj]._warning_flag==1){

            index = CCommonUnit._Vec_TargetsInfo_Classfied[jj]._warning_id;
            // std::cout<< "ego speed :"<< CCommonUnit._VehSpeed._x << std::endl;

            if(jj==0){
            std::cout<<"××××danger front id×××:"<<index<<"×××obs position××× :"<<CCommonUnit._TargetsInfo_Raw._target_info[index]._obs_position._x<<","<<CCommonUnit._TargetsInfo_Raw._target_info[index]._obs_position._y<<std::endl;	
            std::cout<<"×××obs velocity××× :"<<CCommonUnit._TargetsInfo_Raw._target_info[index]._velocity._x<<","<<CCommonUnit._TargetsInfo_Raw._target_info[index]._velocity._y<<std::endl;
            // std::cout<< "repeat number" << CCommonUnit._TargetsInfo_Raw._target_info[index].repeat_num << std::endl;
            a[0]=(float)CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._dist/CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._safe_dist;
            warning_times[0] = warning_times[0]+1;
            if (warning_times[0]>warning_times_threshold)
            {
                a[0]=0;
                
                tmp_target._target_info.push_back(CCommonUnit._TargetsInfo_Raw._target_info[index]);
            }else
            {
                a[0]=1 ;
            }
            pre_warining_level[0] = a[0] ;
            temp[0]._target_info.push_back(CCommonUnit._TargetsInfo_Raw._target_info[index]);
            }

            else if (jj==1 && CCommonUnit.left_lane_flag){
            std::cout<<"××××danger left front id×××:"<<index<<"×××obs position××× :"<<CCommonUnit._TargetsInfo_Raw._target_info[index]._obs_position._x<<","<<CCommonUnit._TargetsInfo_Raw._target_info[index]._obs_position._y<<std::endl;	
            std::cout<<"×××obs velocity××× :"<<CCommonUnit._TargetsInfo_Raw._target_info[index]._velocity._x<<","<<CCommonUnit._TargetsInfo_Raw._target_info[index]._velocity._y<<std::endl;
            // std::cout<< "repeat number" << CCommonUnit._TargetsInfo_Raw._target_info[index].repeat_num << std::endl;
            a[1]=(float)CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._dist/CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._safe_dist;
            
            warning_times[1] = warning_times[1]+1;
            if (warning_times[1]>warning_times_threshold)
            {
                a[1]=0;
                
                tmp_target._target_info.push_back(CCommonUnit._TargetsInfo_Raw._target_info[index]);
            }else
            {
                a[1]=1 ;
                std::cout<<"warning delaied============="<<std::endl;
            }
            pre_warining_level[1] = a[1];
            temp[1]._target_info.push_back(CCommonUnit._TargetsInfo_Raw._target_info[index]);

            float ttc = 0 ;
            std::cout << "distance :"<< (float)CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._dist<< std::endl ;
            ttc=(float)CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._dist /fabs(CCommonUnit._TargetsInfo_Raw._target_info[index]._velocity._x - CCommonUnit._VehSpeed._x);
            std::ofstream outfile ;
            outfile.open("ttc.txt",std::ios::app);
            if (!outfile.is_open()){
                std::cout<<"Open file failure"<< std::endl ;
            }
            outfile<< "ttc left front: " <<ttc << " ego speed: "<< CCommonUnit._VehSpeed._x<< " target speed: "<< CCommonUnit._TargetsInfo_Raw._target_info[index]._velocity._x \
            << " safe distance "<<CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._safe_dist<<" current dist "<<(float)CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._dist \
            <<" ego acc: "<< CCommonUnit._VehAcceleration._x << " target acc:  "<<CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._acceleration._x <<"\n" ;
            outfile.close();
            }

            else if (jj==2 && CCommonUnit.left_lane_flag){
            std::cout<<"××××danger left back id×××:"<<index<<"×××obs position××× :"<<CCommonUnit._TargetsInfo_Raw._target_info[index]._obs_position._x<<","<<CCommonUnit._TargetsInfo_Raw._target_info[index]._obs_position._y<<std::endl;	
            std::cout<<"×××obs velocity××× :"<<CCommonUnit._TargetsInfo_Raw._target_info[index]._velocity._x<<", self vel"<<CCommonUnit._VehSpeed._x \
            <<" ego acc: "<< CCommonUnit._VehAcceleration._x << " target acc: "<<CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._acceleration._x <<std::endl;
            // std::cout<< "repeat number" << CCommonUnit._TargetsInfo_Raw._target_info[index].repeat_num << std::endl;
            a[2] = (float)CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._dist/CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._safe_dist;
            
            warning_times[2] = warning_times[2]+1;
            if (warning_times[2]>warning_times_threshold)
            {
                a[2]=0;         
                tmp_target._target_info.push_back(CCommonUnit._TargetsInfo_Raw._target_info[index]);
            }else
            {
                a[2]=1 ;
            }
            pre_warining_level[2]=a[2];
            temp[2]._target_info.push_back(CCommonUnit._TargetsInfo_Raw._target_info[index]);
            

            float ttc = 0 ;
            std::cout << "distance :"<< (float)CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._dist<< std::endl ;
            ttc=(float)CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._dist /fabs(CCommonUnit._TargetsInfo_Raw._target_info[index]._velocity._x - CCommonUnit._VehSpeed._x);
            std::ofstream outfile ;
            outfile.open("ttc.txt",std::ios::app);
            if (!outfile.is_open()){
                std::cout<<"Open file failure"<< std::endl ;
            }
            outfile<< "ttc left back: " <<ttc << " ego speed:  "<< CCommonUnit._VehSpeed._x<< " target speed: "<< CCommonUnit._TargetsInfo_Raw._target_info[index]._velocity._x \
            << " safe distance "<<CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._safe_dist<<" current dist  "<<(float)CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._dist \
            <<" ego acc: "<< CCommonUnit._VehAcceleration._x << " target acc: "<<CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._acceleration._x << "\n" ;
            outfile.close();
            }      

            else if (jj==3 && CCommonUnit.right_lane_flag){
            std::cout<<"××××danger right front id×××:"<<index<<"×××obs position××× :"<<CCommonUnit._TargetsInfo_Raw._target_info[index]._obs_position._x<<","<<CCommonUnit._TargetsInfo_Raw._target_info[index]._obs_position._y<<std::endl;
            std::cout<<"×××obs velocity××× :"<<CCommonUnit._TargetsInfo_Raw._target_info[index]._velocity._x<<","<<CCommonUnit._TargetsInfo_Raw._target_info[index]._velocity._y<<std::endl;
            // std::cout<< "repeat number" << CCommonUnit._TargetsInfo_Raw._target_info[index].repeat_num << std::endl;
            a[3]=(float)CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._dist/CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._safe_dist;
            warning_times[3] = warning_times[3]+1;
            if (warning_times[3]>warning_times_threshold)
            {
                a[3]=0;
                
                tmp_target._target_info.push_back(CCommonUnit._TargetsInfo_Raw._target_info[index]);
            }else
            {
                a[3]=1 ;
            }
            
            pre_warining_level[3]=a[3];
            temp[3]._target_info.push_back(CCommonUnit._TargetsInfo_Raw._target_info[index]);

            float ttc = 0 ;
            std::cout << "distance :"<< (float)CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._dist<< std::endl ;
            ttc=(float)CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._dist /fabs(CCommonUnit._TargetsInfo_Raw._target_info[index]._velocity._x - CCommonUnit._VehSpeed._x);
            std::ofstream outfile ;
            outfile.open("ttc.txt",std::ios::app);
            if (!outfile.is_open()){
                std::cout<<"Open file failure"<< std::endl ;
            }
            outfile<< "ttc right front: " <<ttc << " ego speed:  "<< CCommonUnit._VehSpeed._x<< " target speed: "<< CCommonUnit._TargetsInfo_Raw._target_info[index]._velocity._x \
            << " safe distance "<<CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._safe_dist<<" current dist "<<(float)CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._dist \
            <<" ego acc:  "<< CCommonUnit._VehAcceleration._x << " target acc:  "<<CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._acceleration._x << "\n" ;
            outfile.close();
            } 

            else if (jj==4 && CCommonUnit.right_lane_flag){
            std::cout<<"××××danger right back id×××:"<<index<<"×××obs position××× :"<<CCommonUnit._TargetsInfo_Raw._target_info[index]._obs_position._x<<","<<CCommonUnit._TargetsInfo_Raw._target_info[index]._obs_position._y<<std::endl;	
            std::cout<<"×××obs velocity××× :"<<CCommonUnit._TargetsInfo_Raw._target_info[index]._velocity._x<<","<<CCommonUnit._TargetsInfo_Raw._target_info[index]._velocity._y<<std::endl;
            // std::cout<< "repeat number" << CCommonUnit._TargetsInfo_Raw._target_info[index].repeat_num << std::endl;         
            a[4]=(float)CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._dist/CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._safe_dist;
            
            warning_times[4] = warning_times[4]+1;
            if (warning_times[4]>warning_times_threshold)
            {
                a[4]=0;
                
                tmp_target._target_info.push_back(CCommonUnit._TargetsInfo_Raw._target_info[index]);
            }else
            {
                a[4]=1 ;
            }
            pre_warining_level[4]=a[4];
            temp[4]._target_info.push_back(CCommonUnit._TargetsInfo_Raw._target_info[index]);

            float ttc = 0 ;
            std::cout << "distance :"<< (float)CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._dist<< std::endl ;
            ttc=(float)CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._dist /fabs(CCommonUnit._TargetsInfo_Raw._target_info[index]._velocity._x - CCommonUnit._VehSpeed._x);
            std::ofstream outfile ;
            outfile.open("ttc.txt",std::ios::app);
            if (!outfile.is_open()){
                std::cout<<"Open file failure"<< std::endl ;
            }
            outfile<< "ttc right back: " <<ttc << " ego speed:  "<< CCommonUnit._VehSpeed._x<< " target speed: "<< CCommonUnit._TargetsInfo_Raw._target_info[index]._velocity._x \
            << " safe distance "<<CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._safe_dist<<" current dist  "<<(float)CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._dist \
            <<" ego acc: "<< CCommonUnit._VehAcceleration._x << " target acc: "<<CCommonUnit._Vec_TargetsInfo_Classfied[jj]._target_info[0]._acceleration._x << "\n" ;
            outfile.close();
            }
            
        }else 
        { //11.21 说明 侧后没有目标&&当前帧未检测到&&上一帧检测到 持续报警100帧
            if (jj==0 && a[0]==1 && pre_warining_level[0]!=1){
                a[0] = pre_warining_level[0];
                warning_keep_num[0]++;
                if( warning_keep_num[0] > warning_keep_threshold)
                {
                  pre_warining_level[0] = 1;
                  warning_keep_num[0] = 0;
                }
            }
            else if (jj==1 && a[1]==1 && pre_warining_level[1]!=1){
                a[1] = pre_warining_level[1];
                warning_keep_num[1]++;
                if( warning_keep_num[1] > warning_keep_threshold)
                {
                  pre_warining_level[1] = 1;
                  warning_keep_num[1] = 0;
                }
            }
            else if (jj==2 && a[2]==1 && pre_warining_level[2]!=1){
                a[2] = pre_warining_level[2];
                warning_keep_num[2]++;
                if( warning_keep_num[2] > warning_keep_threshold)
                {
                  pre_warining_level[2] = 1;
                  warning_keep_num[2] = 0;
                }
            }
            else if (jj==3 && a[3]==1 && pre_warining_level[3]!=1){
                a[3] = pre_warining_level[3];
                warning_keep_num[3]++;
                if( warning_keep_num[3] > warning_keep_threshold)
                {
                  pre_warining_level[3] = 1;
                  warning_keep_num[3] = 0;
                }
            }
            else if (jj==4 && a[4]==1 && pre_warining_level[4]!=1){
                a[4] = pre_warining_level[4];
                warning_keep_num[4]++;
                if( warning_keep_num[4] > warning_keep_threshold)
                {
                  pre_warining_level[4] = 1;
                  warning_keep_num[4] = 0;
                }
            }
        }

    }
    pub_lcw(a);
    pub_danger(tmp_target);
    WarningGui(a, temp);

}
//接收车道线检测信息
void callback_lane_data(const adas_module::ld_Frame msg){

    lane_timestamp = ros::Time::now();
	CCommonUnit.LaneParam_set(msg);
}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "adas_module");
	ros::NodeHandle nh;
	std::string lane_data_msg_name = "/lane_data";
	std::string fusion_result_msg_name = "/final_fusion";
	int system_loop = 0;
    int self_vel_loop=0;
    float MinVel=0/3.6 ;
    int error_flag =0;
    /*接收车道线检测模块结果*/
    ros::Subscriber lane_data_sub = nh.subscribe(lane_data_msg_name, 1, callback_lane_data);
    ros::Subscriber fusion_result_sub = nh.subscribe(fusion_result_msg_name, 1, callback_fusion_result);

	pub_classified_targets = nh.advertise<visualization_msgs::MarkerArray>("/targets_classified", 1);
	pub_classified_targets_label=nh.advertise<visualization_msgs::MarkerArray>("/targets_classified_label", 1);
    pub_danger_targets = nh.advertise<visualization_msgs::MarkerArray>("/targets_danger", 1);
	pub_danger_targets_label=nh.advertise<visualization_msgs::MarkerArray>("/targets_danger_label", 1);
    pub_lcw_flag = nh.advertise<visualization_msgs::MarkerArray>("/warning_lcw_flag", 1);
    pub_lcw_level = nh.advertise<visualization_msgs::MarkerArray>("/warning_lcw_level", 1);
    pub_chosen = nh.advertise<visualization_msgs::MarkerArray>("/vehicle_chosen", 1);
    pub_dist_lf = nh.advertise<std_msgs::Float32>("/dist_lf", 1);
    pub_safe_dist_lf = nh.advertise<std_msgs::Float32>("/dist_safe_lf", 1);
    pub_ttc_lf = nh.advertise<std_msgs::Float32>("/ttc_lf", 1);

    pub_dist_lr = nh.advertise<std_msgs::Float32>("/dist_lr", 1);
    pub_safe_dist_lr = nh.advertise<std_msgs::Float32>("/dist_safe_lr", 1);
    pub_ttc_lr = nh.advertise<std_msgs::Float32>("/ttc_lr", 1);

    pub_dist_rf = nh.advertise<std_msgs::Float32>("/dist_rf", 1);
    pub_safe_dist_rf = nh.advertise<std_msgs::Float32>("/dist_safe_rf", 1);
    pub_ttc_rf = nh.advertise<std_msgs::Float32>("/ttc_rf", 1);

    pub_dist_rr = nh.advertise<std_msgs::Float32>("/dist_rr", 1);
    pub_safe_dist_rr = nh.advertise<std_msgs::Float32>("/dist_safe_rr", 1);
    pub_ttc_rr = nh.advertise<std_msgs::Float32>("/ttc_rr", 1);

    //clear ttc info file
    
    std::ofstream outfile("ttc.txt", std::ios::trunc);
    outfile.close();

    std::ofstream file("info.txt", std::ios::trunc);
    file.close();

    std::ofstream filee("flag.txt", std::ios::trunc);
    filee.close();

    
    
    //时钟初始化
    ros::Time::init();
    lane_timestamp = ros::Time::now();
    fusion_timestamp = ros::Time::now();
    ros::Rate loop(100);

    lcw_visual.create(300, 360, CV_8UC3);
    lcw_visual.setTo(255);
    // car = cv::imread("/home/skou/Pictures/car.jpg") ;
    // cv::transpose(car,car);
    // cv::flip(car,car,0);

    // cv::resize(car,car, cv::Size(car.cols/10, car.rows/10),0,0,cv::INTER_LINEAR) ;
    // cv::Rect roi = cv::Rect(140,117.5,car.cols,car.rows);
    // car.copyTo(lcw_visual(roi));

    cv::circle(lcw_visual, cv::Point2f(320,30),10,cv::Scalar(0,255,0),-1,8,0);
    cv::rectangle(lcw_visual,cv::Point2f(150,140),cv::Point2f(170,160),cv::Scalar(255,0,0),-1,8,0);

    //初始值 防止系统在未call back前死掉
    CCommonUnit._CurRadRoad =1000;
    while (ros::ok){
        ros::spinOnce();
        lane_update_delta = ros::Time::now().toSec() - lane_timestamp.toSec();
        fusion_update_delta = ros::Time::now().toSec() - fusion_timestamp.toSec();
        lane_danger_check();

        if(system_loop < 100 ){
            if(CCommonUnit._CurRadRoad < 500){
                system_loop ++;
            }else{
                system_loop = 0;
            }
        }else{
            // std::cout<< "road radius of curvature too small !!" << std::endl ; 
            error_flag =1;
            //break;
        }

        if (self_vel_loop < 100){
            if(CCommonUnit._VehSpeed._x < MinVel){
                self_vel_loop ++;
            }else{
                self_vel_loop = 0;
            }
        }
        else {
            // std::cout<<"vehicle speed too small !!"<<std::endl ;
            error_flag =2;
            //break ;
        }

        if(lane_update_delta > 3.0)
        {
            // std::cout<<"Too old,lane update delta lane:"<<lane_update_delta<<std::endl;
            error_flag =3;
            //break ;
        }

        if(fusion_update_delta > 3.0){
            // std::cout<<"Too old,fusion update delta fusion:"<<fusion_update_delta<<std::endl;
            error_flag =4;
           // break ;
        }

        
	loop.sleep();
	}


    // while (ros::ok){
    //     if (error_flag!=0){
    //         cv::circle(lcw_visual, cv::Point2f(320,30),10,cv::Scalar(0,0,255),-1,8,0);
    //         lane_danger_check();  
    //     }
            
    //     loop.sleep();

    // }
	return 1;	

}
