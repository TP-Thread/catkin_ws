/***************************************************************************************************************************
*
* Author: bingo
* Email: bingobin.lw@gmail.com
* Time: 2020.08.25
* Description: 实现px4 quadrotor yolo检测，kcf跟踪
***************************************************************************************************************************/
#include "tracking_down_kcf_quadrotor.h"
using namespace std;
using namespace Eigen;
PX4KcfDownTracking::PX4KcfDownTracking(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private) {
  Initialize();
  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.1), &PX4KcfDownTracking::CmdLoopCallback, this); //周期为0.1s
  //订阅目标相对飞机在图像像素点的位置
  target_pose_sub_ = nh_private_.subscribe("/target_position", 1, &PX4KcfDownTracking::TargetPoseCallback, this,ros::TransportHints().tcpNoDelay());

  position_sub_ = nh_private_.subscribe("/mavros/local_position/pose", 1, &PX4KcfDownTracking::Px4PosCallback,this,ros::TransportHints().tcpNoDelay());

  state_sub_ = nh_private_.subscribe("/mavros/state", 1, &PX4KcfDownTracking::Px4StateCallback,this,ros::TransportHints().tcpNoDelay());
  // 【服务】修改系统模式
  set_mode_client_ = nh_private_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

}

PX4KcfDownTracking::~PX4KcfDownTracking() {
  //Destructor
}

/**
* @name       S_SETPOINT_VEL PX4KcfDownTracking::TrackingPidProcess(Eigen::Vector3d &currentPos,Eigen::Vector3d &expectPos)

* @brief      pid控制程序
*             
* @param[in]  &currentPos 当前飞机相对目标的位置
*             
* @param[in]  &expectPos 期望位置
* @param[out] 机体系下x,y的期望速度
*
* @param[out] 
**/
Eigen::Vector3d PX4KcfDownTracking::TrackingPidProcess(Eigen::Vector3d &currentPos,Eigen::Vector3d &expectPos)
{
  Eigen::Vector3d s_PidOut;
	/*X方向的pid控制*/
	s_PidItemX.difference = expectPos[0] - currentPos[0];
	s_PidItemX.intergral += s_PidItemX.difference;
	if(s_PidItemX.intergral >= 500)		
		s_PidItemX.intergral = 500;
	else if(s_PidItemX.intergral <= -500) 
		s_PidItemX.intergral = -500;
	s_PidItemX.differential =  s_PidItemX.difference  - s_PidItemX.tempDiffer;
  s_PidItemX.tempDiffer = s_PidItemX.difference;
//	cout << "s_PidItemX.tempDiffer: " << s_PidItemX.tempDiffer << endl;
//	cout << "s_PidItemX.differential: " << s_PidItemX.differential << endl;

	s_PidOut[0] = s_PidXY.p*s_PidItemX.difference + s_PidXY.d*s_PidItemX.differential + s_PidXY.i*s_PidItemX.intergral;
	/*Y方向的pid控制*/
	s_PidItemY.difference = expectPos[1] - currentPos[1];
	s_PidItemY.intergral += s_PidItemY.difference;
	if(s_PidItemY.intergral >= 500)		
		s_PidItemY.intergral = 500;
	else if(s_PidItemY.intergral <= -500) 
		s_PidItemY.intergral = -500;
	s_PidItemY.differential =  s_PidItemY.difference  - s_PidItemY.tempDiffer;
  s_PidItemY.tempDiffer = s_PidItemY.difference;
	s_PidOut[1] = s_PidXY.p*s_PidItemY.difference + s_PidXY.d*s_PidItemY.differential + s_PidXY.i*s_PidItemY.intergral;
	return s_PidOut;
}
void PX4KcfDownTracking::CmdLoopCallback(const ros::TimerEvent& event)
{
  TrackingStateUpdate();
}


/**
* @name       void PX4KcfDownTracking::TrackingStateUpdate()
* @brief      状态机更新函数
*             
* @param[in]  无
*             
* @param[in]  无
* @param[out] 
*
* @param[out] 
**/
void PX4KcfDownTracking::TrackingStateUpdate()
{

//	desire_vel_ = LandingPidProcess(target_pose_,markers_yaw_,desire_pose_,0);
//	cout << "desire_vel_[0]:  "<< desire_vel_[0] <<endl;
//	cout << "desire_vel_[1]:  "<< desire_vel_[1] <<endl;
//	cout << "desire_vel_[2]:  "<< desire_vel_[2] <<endl;
//	cout << "desire_vel_[3]:  "<< desire_vel_[3] <<endl;
//	cout << "markers_yaw_: "  << markers_yaw_ << endl;
//	cout << "target_pose_[0]:  "<<  target_pose_[0] << endl;
//	cout << "target_pose_[1]:  "<<  target_pose_[1] << endl;
//	cout << "target_pose_[2]:  "<<  target_pose_[2] << endl;
//	cout << "desire_pose_[0]:  "<<  desire_pose_[0] << endl;
//	cout << "desire_pose_[1]:  "<<  desire_pose_[1] << endl;
//	cout << "desire_pose_[2]:  "<<  desire_pose_[2] << endl;
//	cout << "detect_state : " << detect_state << endl;
	switch(TrackingState)
	{
		case WAITING:
			if(px4_state_.mode != "OFFBOARD")//等待offboard模式
			{
				temp_pos_drone[0] = px4_pose_[0];
				temp_pos_drone[1] = px4_pose_[1];
				temp_pos_drone[2] = px4_pose_[2];
				OffboardControl_.send_pos_setpoint(temp_pos_drone, 0);
			}
			if(px4_state_.mode == "OFFBOARD")
			{
				temp_pos_drone[0] = px4_pose_[0];
				temp_pos_drone[1] = px4_pose_[1];
				temp_pos_drone[2] = px4_pose_[2];
				TrackingState = CHECKING;
				cout << "CHECKING" <<endl;
			}
			break;
		case CHECKING:
			if(px4_pose_[0] == 0 && px4_pose_[1] == 0) 			//没有位置信息则执行降落模式
			{
				cout << "Check error, make sure have local location" <<endl;
				mode_cmd_.request.custom_mode = "AUTO.LAND";
				set_mode_client_.call(mode_cmd_);
				TrackingState = WAITING;	
			}
			else
			{
				TrackingState = PREPARE;
				cout << "PREPARE" <<endl;
			}
			
			break;
		case PREPARE:											//起飞到指定高度
			posxyz_target[0] = temp_pos_drone[0];
			posxyz_target[1] = temp_pos_drone[1];
			posxyz_target[2] = search_alt_;
			if((px4_pose_[2]<=search_alt_+0.1) && (px4_pose_[2]>=search_alt_-0.1))
			{
				TrackingState = SEARCH;
			}
			else if(detect_state == true)
			{
			//	TrackingState = SEARCH;
			}
			OffboardControl_.send_pos_setpoint(posxyz_target, 0);					
			if(px4_state_.mode != "OFFBOARD")				//如果在准备中途中切换到onboard，则跳到WAITING
			{
				TrackingState = WAITING;
			}

			break;
		case SEARCH:
			if(detect_state == true)
			{
				TrackingState = TRACKING;
			  cout << "TRACKING" <<endl;
			}	
			else//这里无人机没有主动搜寻目标
			{
				OffboardControl_.send_pos_setpoint(posxyz_target, 0);
			}
			if(px4_state_.mode != "OFFBOARD")				//如果在SEARCH途中切换到onboard，则跳到WAITING
			{
				TrackingState = WAITING;
			}
     // cout << "SEARCH" <<endl;
			break;
		case TRACKING:
			{
				if(detect_state == true)
				{
					desire_vel_ = TrackingPidProcess(target_pose_,desire_pose_);

					//cout << "search_" <<endl;
				}
			  else
				{
					desire_vel_[0] = 0;
					desire_vel_[1] = 0;
					desire_vel_[2] = 0;
				}

				if(px4_state_.mode != "OFFBOARD")			//如果在LANDING中途中切换到onboard，则跳到WAITING
				{
					TrackingState = WAITING;
				}
/*
				if(desire_xyVel_[0] >= 0.3)
				{
					desire_xyVel_[0] = 0.3;
				}
				if(desire_xyVel_[0] <= -0.3)
				{
					desire_xyVel_[0] = -0.3;
				}
				if(desire_xyVel_[1] >= 0.3)
				{
					desire_xyVel_[1] = 0.3;
				}
				if(desire_xyVel_[1] <= -0.3)
				{
					desire_xyVel_[1] = -0.3;
				}
*/
				desire_xyVel_[0] = desire_vel_[0];
				desire_xyVel_[1] = desire_vel_[1];
				OffboardControl_.send_body_velxy_posz_setpoint(desire_xyVel_,search_alt_);
			}

			break;
		case TRACKOVER:
			{
				mode_cmd_.request.custom_mode = "AUTO.LAND";
        		set_mode_client_.call(mode_cmd_);
				TrackingState = WAITING;
			}

			break;

		default:
			cout << "error" <<endl;
	}	

}

/*接收目标相对飞机的像素点位置*/
void PX4KcfDownTracking::TargetPoseCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
      target_pose_[0] = 640-msg->position.x;
      target_pose_[1] = msg->position.y;
//			cout << "target_pose_[0]:"  << target_pose_[0] << endl;
//			cout << "target_pose_[1]:"  << target_pose_[1] << endl;

}

/*接收来自飞控的当前飞机位置*/                  
void PX4KcfDownTracking::Px4PosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Read the Drone Position from the Mavros Package [Frame: ENU]
    Eigen::Vector3d pos_drone_fcu_enu(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);

    px4_pose_ = pos_drone_fcu_enu;
}
/*接收来自飞控的当前飞机状态*/
void PX4KcfDownTracking::Px4StateCallback(const mavros_msgs::State::ConstPtr& msg)
{
	px4_state_ = *msg;
}

/*初始化*/
void PX4KcfDownTracking::Initialize()
{
  //读取offboard模式下飞机的搜索高度
  nh_private_.param<float>("search_alt_", search_alt_, 3);

  nh_private_.param<float>("PidXY_p", s_PidXY.p, 0.4);
  nh_private_.param<float>("PidXY_d", s_PidXY.d, 0.05);
  nh_private_.param<float>("PidXY_i", s_PidXY.i, 0.01);
  //期望的飞机相对目标的位置
  float desire_pose_x,desire_pose_y,desire_pose_z;
  nh_private_.param<float>("desire_pose_x", desire_pose_x, 0);
  nh_private_.param<float>("desire_pose_y", desire_pose_y, 0);
  desire_pose_[0] = desire_pose_x;
  desire_pose_[1] = desire_pose_y;

  detect_state = 1;
  desire_vel_[0] = 0;
  desire_vel_[1] = 0;

  desire_xyVel_[0]  = 0;
  desire_xyVel_[1]  = 0;
  s_PidItemX.tempDiffer = 0;
  s_PidItemY.tempDiffer = 0;
  s_PidItemX.intergral = 0;
  s_PidItemY.intergral = 0;

  cout << "search_alt_ = " << search_alt_ << endl;
  cout << "PidXY_p = " << s_PidXY.p << endl;
  cout << "PidXY_d = " << s_PidXY.d << endl;
  cout << "PidXY_i = " << s_PidXY.i << endl;
  cout << "desire_pose_x = " << desire_pose_[0] << endl;
  cout << "desire_pose_y = " << desire_pose_[1] << endl;

}
int main(int argc, char** argv) {
  ros::init(argc,argv,"tracking_down_kcf_quadrotor");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  PX4KcfDownTracking PX4KcfDownTracking(nh, nh_private);

  ros::spin();
  return 0;
}
