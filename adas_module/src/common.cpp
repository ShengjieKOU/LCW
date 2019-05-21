#include "common.h"

float LaneWidthMin = 2.5 ;  //标准车道宽度上下限
float LaneWidthMax = 4.5 ;
int num = 0 ;

bool dias::adas::compare(dias::adas::TARGET_INFO a,dias::adas::TARGET_INFO b)
{
	return a._dist < b._dist; //升序排列
}

void dias::adas::Select_Min_TTC(TARGETS_INFO &targets){ // 筛选区域内距离自车最近的五个目标
	TARGETS_INFO tmp;
	tmp = targets;
	if(tmp._target_info.size()>0){
		// std::sort(tmp._target_info.begin(),tmp._target_info.end(),compare);

		targets._target_info.clear();
		targets._target_info.push_back(tmp._target_info[0]);
		for(int ii=1;ii<tmp._target_info.size();ii++){
			if((tmp._target_info[ii]._dist/tmp._target_info[ii]._velocity._x >0)&&(tmp._target_info[ii]._dist/tmp._target_info[ii]._velocity._x<targets._target_info[0]._dist/targets._target_info[0]._velocity._x)){
				targets._target_info.clear();
				targets._target_info.push_back(tmp._target_info[ii]);
			}
			// if(ii>0){break;	}			
			// targets._target_info.push_back(tmp._target_info[ii]);
		}
	}	
}

double valueAtIPM(std::vector<float> &f, float x)
{
    float ans = 0.f;
    for (int i = (int)f.size() - 1; i >= 0; --i)
        ans = ans * x + f[i];
    return ans;
}

dias::adas::CCommon::CCommon(){
	TARGETS_INFO tmp_targets;
	_Vec_TargetsInfo_Classfied.push_back(tmp_targets);

	_Vec_TargetsInfo_Classfied.push_back(tmp_targets);

	_Vec_TargetsInfo_Classfied.push_back(tmp_targets);

	_Vec_TargetsInfo_Classfied.push_back(tmp_targets);	

	_Vec_TargetsInfo_Classfied.push_back(tmp_targets);

	_Vec_TargetsInfo_Classfied.push_back(tmp_targets);

}


void dias::adas::CCommon::LaneParam_set(adas_module::ld_Frame msg){

	if(msg.curve_radius!=0){
	_CurRadRoad = msg.curve_radius;  //曲率半径
	}
	 if(msg.bias_dis!=0){
     _LaneDepValue = msg.bias_dis; //偏离车道中心距离
	 }
     _LaneWidth = msg.lane_width ;  //车道宽度
     _LaneAngle = msg.bias_theta;  //道路与车中线夹角

	lane_coeff.clear();
	for(int i=0;i<msg.lane_Coeff.size();i++){

        lanecoeff coeff;
        coeff.a = msg.lane_Coeff[i].a;
        coeff.b = msg.lane_Coeff[i].b;
        coeff.c = msg.lane_Coeff[i].c;
        coeff.d = msg.lane_Coeff[i].d;
		lane_coeff.push_back(coeff) ;
    }

	int lane_num = lane_coeff.size() ;
	
	std::vector<float> lane_intercept ;
	for (int i=0; i<lane_coeff.size(); i++){
		std::vector<float> coeff(4) ;
		coeff[3]=lane_coeff[i].a ;
		coeff[2]=lane_coeff[i].b ;
		coeff[1]=lane_coeff[i].c ;
		coeff[0]=lane_coeff[i].d ;
		float y0 = valueAtIPM(coeff,0) ;
		lane_intercept.push_back(y0) ;
	}

	std::sort(lane_intercept.begin(), lane_intercept.end(),std::greater<float>()) ;

	left_lane_flag = false; // left
	right_lane_flag = false; // right

	int self_id = -1;	

	for (int i=0 ; i<lane_intercept.size(); i++){
		if (lane_intercept[i]>0 && lane_intercept[i+1]<0){
			self_id= i ;	
		}
	}
	num ++ ;
	std::cout<<"entre lane judge============"<<num<<std::endl ;

	//判断左右车道线是否存在
	if (self_id!=-1){
		if (self_id>0 && (lane_intercept[self_id-1]-lane_intercept[self_id]) > LaneWidthMin \
				&&(lane_intercept[self_id-1]-lane_intercept[self_id]) < LaneWidthMax){
			left_lane_flag = true ;
			std::cout<<"left lane detected-----"<<std::endl;
		}
		if (self_id < lane_num-2 && (lane_intercept[self_id+1]-lane_intercept[self_id+2])> LaneWidthMin \
				&& (lane_intercept[self_id+1]-lane_intercept[self_id+2]) < LaneWidthMax){
			right_lane_flag = true ;
			std::cout<<"right lane detected-----"<<std::endl;
		}
	}


	//记录历史车道线存在标签
	int hist_size = 100;
	if (left_lane_flag)
	{
		hist_left_lane_flag.push_back(1.0);

		if (hist_left_lane_flag.size()>hist_size)
		{
			hist_left_lane_flag.erase(hist_left_lane_flag.begin());
		}
	}else if (!left_lane_flag)
	{
		hist_left_lane_flag.push_back(0);
		if (hist_left_lane_flag.size()>hist_size)
		{
			hist_left_lane_flag.erase(hist_left_lane_flag.begin());
		}
	}

	if (right_lane_flag)
	{
		hist_right_lane_flag.push_back(1);
		if (hist_right_lane_flag.size()>hist_size)
		{
			hist_right_lane_flag.erase(hist_right_lane_flag.begin());
		}
	}else if (!right_lane_flag)
	{
		hist_right_lane_flag.push_back(0);
		if (hist_right_lane_flag.size()>hist_size)
		{
			hist_right_lane_flag.erase(hist_right_lane_flag.begin());
		}
	}

	int valid_size = 0 ;

	//如果当前判断没有车道，从历史信息查看，如果历史信息有车道，做修正
	float hist_left_lane = std::accumulate(hist_left_lane_flag.begin(), hist_left_lane_flag.end(),0);
	if (left_lane_flag)
	{
		if (hist_left_lane > valid_size)
		{
			left_lane_flag = true ;
			std::cout<<"left lane verified---"<<std::endl;
		}else
		{
			left_lane_flag = false ;
			std::cout <<"left lane delete-----"<<std::endl ;
		}
	}else if (!left_lane_flag)
	{
		if (hist_left_lane>0)
		{
			left_lane_flag = true ;
			std::cout<<"left disappear correct---"<<std::endl ;
		}else
		{
			left_lane_flag = false ;
			std::cout<<"left disappear verified---"<<std::endl ;
		}

	}
	float hist_right_lane = std::accumulate(hist_right_lane_flag.begin(), hist_right_lane_flag.end(),0);
	if (right_lane_flag)
	{
		if (hist_right_lane > valid_size)
		{
			right_lane_flag = true ;
			std::cout<<"right lane verified---"<<std::endl;
		}else
		{
			right_lane_flag = false ;
			std::cout <<"right lane delete-----"<<std::endl ;
		}
	}else if (!right_lane_flag)
	{
		if (hist_right_lane>0)
		{
			right_lane_flag = true ;
			std::cout<<"right disappear correct---"<<std::endl ;
		}else
		{
			right_lane_flag = false ;
			std::cout<<"right disappear verified---"<<std::endl ;
		}

	}

}

void dias::adas::CCommon::CarParam_set(OBSVector3d VehSpeed,float YawVelocity,OBSVector3d VehAcceleration){
     _VehSpeed = VehSpeed; //自车车速
     _YawVelocity = YawVelocity; //横摆角速度
	 _VehAcceleration =VehAcceleration;//自车加速度
}

//根据车道宽度，车道线宽度筛选
//目标分区域存储到_Vec_TargetsInfo_Classfied中
void dias::adas::CCommon::Target_Classified(TARGET_INFO target){

	float left_lane_width ;
	float right_lane_width ;

	left_lane_width=_LaneWidth -1;//考虑摄像头到真实坐标系的误差，-1 m做修正
	right_lane_width=_LaneWidth-1;//

	if (!left_lane_flag)
	{
		left_lane_width = 0;
	}

	if (!right_lane_flag)
	{
		right_lane_width = 0 ;
	}

	float l_dist =0.0;
	float r_dist =0.0;

	if(_LaneDepValue < 0)
	{
		l_dist = _LaneWidth/2  + _LaneDepValue;
		r_dist = _LaneWidth - l_dist ;
	}else{
		r_dist = _LaneWidth/2 - _LaneDepValue;
		l_dist = _LaneWidth - r_dist ;
	}
	// std::cout<<"lane width: "<< l_dist << "and :"<<left_lane_width <<std::endl ;
	
	float veh_back = 1.03;
	float veh_front = 3.7;

	// 考虑传感器检测范围，目前只针对100内目标
	// if (fabs(target._obs_position._x)>100 ||target._velocity._x<0.5){
	if (fabs(target._velocity._x)<1.0){
		_Vec_TargetsInfo_Classfied[5]._target_info.push_back(target);
	}
	//自车道 前车
	else if( target._obs_position._x  > veh_front && (-(r_dist) < target._obs_position._y && target._obs_position._y< (l_dist)) ){
		_Vec_TargetsInfo_Classfied[0]._target_info.push_back(target);
	}		
	//左侧车道 前车
	else if( target._obs_position._x  > veh_front && ((l_dist) < target._obs_position._y && target._obs_position._y < (l_dist + left_lane_width))){
		_Vec_TargetsInfo_Classfied[1]._target_info.push_back(target);
		// std::cout<<"target acc:"<< target._acceleration._x<< std::endl ;
	}	
	//左侧车道 后车
	else if( target._obs_position._x  < -veh_back && (l_dist) < target._obs_position._y &&target._obs_position._y < (l_dist + left_lane_width) ){
		_Vec_TargetsInfo_Classfied[2]._target_info.push_back(target);
		// std::cout<<"target acc:"<< target._acceleration._x<< std::endl ;
		// std::cout<<"target RB acc:"<< target._acceleration._x<< "self :"<<_VehAcceleration._x <<std::endl ;
	}	
	//右侧车道 前车
	else if( target._obs_position._x  > veh_front && -(r_dist+right_lane_width) <target._obs_position._y && target._obs_position._y < -(r_dist) ){
		_Vec_TargetsInfo_Classfied[3]._target_info.push_back(target);
		// std::cout<<"target acc:"<< target._acceleration._x<< std::endl ;		
	}	
	//右侧车道 后车
	else if( target._obs_position._x  < -veh_back && -(r_dist+right_lane_width) < target._obs_position._y && target._obs_position._y < -(r_dist) ){
		_Vec_TargetsInfo_Classfied[4]._target_info.push_back(target);

		// std::cout<<"target RB acc:"<< target._acceleration._x<< "self :"<<_VehAcceleration._x <<std::endl ;
	}else
	{
		_Vec_TargetsInfo_Classfied[5]._target_info.push_back(target);
	}

} 

// 更新目标状态
void dias::adas::CCommon::Update_Target_Status(adas_module::TrackArray msg){
	std::cout<<"Entre update--"<<std::endl;
    float veh_leng = 4.0;
	float veh_back = 1.03;
	float veh_front = 3.7;
	float veh_width = 2.5;
	float R0 = _CurRadRoad;
	float R1 = _CurRadRoad;
	float alphi = 0.0;
	float theta = 0.0;

	int flag_curve = 0;
	float r2=0;
	TARGET_INFO tmp_target;
	_TargetsInfo_Raw._target_info.clear();
	_Vec_TargetsInfo_Classfied[0]._target_info.clear();
	_Vec_TargetsInfo_Classfied[1]._target_info.clear();
	_Vec_TargetsInfo_Classfied[2]._target_info.clear();
	_Vec_TargetsInfo_Classfied[3]._target_info.clear();
	_Vec_TargetsInfo_Classfied[4]._target_info.clear();
	_Vec_TargetsInfo_Classfied[5]._target_info.clear();

	
	//  直道/弯道判断

	if (_CurRadRoad > 1000){
		flag_curve = 0;
	}else if(_CurRadRoad>500 && _CurRadRoad<1000){
		flag_curve = 1;
		std::cout<< "curve road 00000000000"<<std::endl ;
	}
	
	

	float count_delete=0;
    for (int ii=0;ii<msg.tracks.size();ii++)
    {
		// if (msg.tracks[ii].id==312 )
		// {
		// 	std::cout<<"target acc:"<< msg.tracks[ii].acceleration.x<< std::endl ;
		// }
		
		// 设置追踪次数限制，追踪次数大于5次为真实目标
		if (msg.tracks[ii].flag_repeat>5){
		tmp_target._id =ii-count_delete;
		tmp_target.repeat_num=msg.tracks[ii].flag_repeat ;
		if (flag_curve==0){ //直道情形
			tmp_target._obs_position._x = msg.tracks[ii].obs_position.x;
			tmp_target._obs_position._y = msg.tracks[ii].obs_position.y;
			
			//坐标原点在车辆后轴，相对距离应考虑车辆长度。
			if (msg.tracks[ii].obs_position.x >0 ){
			tmp_target._dist = msg.tracks[ii].obs_position.x - veh_front ;
			}else{
			tmp_target._dist = fabs( msg.tracks[ii].obs_position.x + veh_back)  ;
			}
			tmp_target._velocity._x = msg.tracks[ii].velocity.x + _VehSpeed._x;
			tmp_target._velocity._y = msg.tracks[ii].velocity.y + _VehSpeed._y;
			tmp_target._velocity._z = msg.tracks[ii].velocity.z;
			tmp_target._acceleration._x = msg.tracks[ii].acceleration.x + _VehAcceleration._x;
			tmp_target._acceleration._y = msg.tracks[ii].acceleration.y + _VehAcceleration._y;
			// tmp_target._acceleration._x = 0;
			// tmp_target._acceleration._y = 0;
		}
		else if (flag_curve==1) { //弯道情况
			alphi = atan(fabs(msg.tracks[ii].obs_position.y/msg.tracks[ii].obs_position.x));                        
			r2=pow(msg.tracks[ii].obs_position.x,2)+pow(msg.tracks[ii].obs_position.y,2);
			R1= sqrt(pow(R0,2)+r2-2*R0*sqrt(r2)*cos(alphi + PI/2)); //目标处的曲率半径粗略估计
			theta= acos((pow(R1,2)+pow(R0,2)-r2)/(2*R1*R0));
			tmp_target._obs_position._x = msg.tracks[ii].obs_position.x;
			tmp_target._obs_position._y = msg.tracks[ii].obs_position.y;
			if (msg.tracks[ii].obs_position.x >0 ){
				tmp_target._dist = R1 * fabs(alphi) - veh_front ;
			}else{
				tmp_target._dist = R1 * fabs(alphi) - veh_back ;
			}
			tmp_target._velocity._x = msg.tracks[ii].velocity.x + _VehSpeed._x;
			tmp_target._velocity._y = msg.tracks[ii].velocity.y + _VehSpeed._y;
			tmp_target._velocity._z = msg.tracks[ii].velocity.z;
			tmp_target._acceleration._x = msg.tracks[ii].acceleration.x + _VehAcceleration._x;
			tmp_target._acceleration._y = msg.tracks[ii].acceleration.y + _VehAcceleration._y;
			// tmp_target._acceleration._x = 0;
			// tmp_target._acceleration._y = 0;
		} 
		_TargetsInfo_Raw._target_info.push_back(tmp_target);
		Target_Classified(tmp_target);
		}else {
			count_delete++;
		}
	}
}
void dias::adas::CCommon::Target_Classified_Fusion(adas_module::TrackArray msg){
 
	TARGET_INFO tmp_target;
    float veh_leng = 4.0;
	float veh_back = 1.03;
	float veh_front = 3.7;
	float veh_width = 2.5;
	float R0 = _CurRadRoad;
	float R1 = _CurRadRoad;
	float alphi = 0.0;
	float theta = 0.0;

	int flag_curve = 0;
	float r2=0;
	if (_CurRadRoad > 2000){
		flag_curve = 0;
	}else{
		flag_curve = 1;
	}
	_TargetsInfo_Raw._target_info.clear();
	_Vec_TargetsInfo_Classfied[0]._target_info.clear();
	_Vec_TargetsInfo_Classfied[1]._target_info.clear();
	_Vec_TargetsInfo_Classfied[2]._target_info.clear();
	_Vec_TargetsInfo_Classfied[3]._target_info.clear();
	_Vec_TargetsInfo_Classfied[4]._target_info.clear();
	_Vec_TargetsInfo_Classfied[5]._target_info.clear();

    for (int ii=0;ii<msg.tracks.size();ii++)
    {
		tmp_target._id = ii;
		tmp_target._obs_position._x = msg.tracks[ii].obs_position.x;
		tmp_target._obs_position._y = msg.tracks[ii].obs_position.y;
		tmp_target._velocity._x = msg.tracks[ii].velocity.x + _VehSpeed._x;
		tmp_target._velocity._y = msg.tracks[ii].velocity.y + _VehSpeed._y;
		tmp_target._velocity._z = msg.tracks[ii].velocity.z;
		tmp_target._acceleration._x = msg.tracks[ii].acceleration.x + _VehAcceleration._x;
		tmp_target._acceleration._y = msg.tracks[ii].acceleration.y + _VehAcceleration._y;
		_TargetsInfo_Raw._target_info.push_back(tmp_target);
		//Road Model
		if(flag_curve == 0){
			if (msg.tracks[ii].obs_position.x >0 ){
				tmp_target._dist = R1 * fabs(alphi) - veh_front ;
			}else{
				tmp_target._dist = R1 * fabs(alphi) - veh_back ;
			}
		}else if (flag_curve == 1){
			r2 = pow(msg.tracks[ii].obs_position.x,2)+pow(msg.tracks[ii].obs_position.y,2);
			theta = atan((msg.tracks[ii].obs_position.y)/(msg.tracks[ii].obs_position.x));
			R1 = sqrt(r2+pow(R0,2)-2*sqrt(r2)*R0*cos(theta+PI/2));
			alphi = acos((pow(R1,2)+pow(R0,2)-r2)/(2*R1*R0));
			if (msg.tracks[ii].obs_position.x >0 ){
				tmp_target._dist = R1 * fabs(alphi) - veh_front ;
			}else{
				tmp_target._dist = R1 * fabs(alphi) - veh_back ;
			}
		tmp_target._velocity._x = (msg.tracks[ii].velocity.x + _VehSpeed._x)*cos(alphi)+(msg.tracks[ii].velocity.y + _VehSpeed._y)*sin(alphi);
		tmp_target._acceleration._x = (msg.tracks[ii].acceleration.x + _VehAcceleration._x)*cos(alphi)+(msg.tracks[ii].acceleration.y + _VehAcceleration._y)*sin(alphi);
		}// Done
		if (msg.tracks[ii].tracks_lane == 0 && (msg.tracks[ii].obs_position.x-3.7)>0){
			
			_Vec_TargetsInfo_Classfied[0]._target_info.push_back(tmp_target);
		}else if (msg.tracks[ii].tracks_lane == 1 && (msg.tracks[ii].obs_position.x-3.7)>0){
			
			_Vec_TargetsInfo_Classfied[1]._target_info.push_back(tmp_target);

		}else if (msg.tracks[ii].tracks_lane == 1 && (msg.tracks[ii].obs_position.x-3.7)<0){
			
			_Vec_TargetsInfo_Classfied[2]._target_info.push_back(tmp_target);
					
		}else if  (msg.tracks[ii].tracks_lane == -1 && (msg.tracks[ii].obs_position.x-3.7)>0){
			
			_Vec_TargetsInfo_Classfied[3]._target_info.push_back(tmp_target);	
		}else if  (msg.tracks[ii].tracks_lane == -1 && (msg.tracks[ii].obs_position.x-3.7)<0){
			
			_Vec_TargetsInfo_Classfied[4]._target_info.push_back(tmp_target);	
		}else{
			
			_Vec_TargetsInfo_Classfied[5]._target_info.push_back(tmp_target);	
		}
	}

}
	
