/***************************************************************************************************************************
*
* Author: bingo
* Email: bingobin.lw@gmail.com
* Time: 2020.08.08
* Description: 实现ardupilot飞机 yolo目标检测进而使用kcf目标跟踪
***************************************************************************************************************************/
#include "tracking_ardupilot.h"
using namespace std;
using namespace Eigen;
APMtracking::APMtracking(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private) {
  Initialize();
  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.1), &APMtracking::CmdLoopCallback, this); //周期为0.1s
  //订阅目标相对飞机在图像像素点的位置
  target_pose_sub_ = nh_private_.subscribe("/target_position", 1, &APMtracking::TargetPoseCallback, this,ros::TransportHints().tcpNoDelay());
  //订阅是否检测到目标标志位
  target_state_sub_ = nh_private_.subscribe("/tracking/if_tracking", 1, &APMtracking::TargetStateCallback, this,ros::TransportHints().tcpNoDelay());

  position_sub_ = nh_private_.subscribe("/mavros/global_position/local", 1, &APMtracking::ApmPosCallback,this,ros::TransportHints().tcpNoDelay());

  state_sub_ = nh_private_.subscribe("/mavros/state", 1, &APMtracking::ApmStateCallback,this,ros::TransportHints().tcpNoDelay());
  // 【服务】修改系统模式
  set_mode_client_ = nh_private_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

}

APMtracking::~APMtracking() {
  //Destructor
}

/**
* @name       S_SETPOINT_VEL APMtracking::TrackingPidProcess(Eigen::Vector3d &currentPos,Eigen::Vector3d &expectPos)

* @brief      pid控制程序
*             
* @param[in]  &currentPos 当前飞机相对降落板的位置
*             
* @param[in]  &expectPos 期望位置
* @param[out] 机体系下x,y,z的期望速度
*
* @param[out] 
**/
Eigen::Vector3d APMtracking::TrackingPidProcess(Eigen::Vector3d &currentPos,Eigen::Vector3d &expectPos)
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
void APMtracking::CmdLoopCallback(const ros::TimerEvent& event)
{
			
//				temp_pos_drone[0] = 0.1;
//				temp_pos_drone[1] = 0.1;
//				temp_pos_drone[2] = 1.5;
//				OffboardControl_.send_body_velxy_posz_setpoint(temp_pos_drone,1.5);
				//OffboardControl_.send_pos_setpoint(temp_pos_drone, 0);
  TrackingStateUpdate();
}


/**
* @name       void APMtracking::TrackingStateUpdate()
* @brief      状态机更新函数
*             
* @param[in]  无
*             
* @param[in]  无
* @param[out] 
*
* @param[out] 
**/
void APMtracking::TrackingStateUpdate()
{

/*	desire_vel_ = TrackingPidProcess(target_pose_,desire_pose_);
	cout << "desire_vel_[0]:  "<< desire_vel_[0] <<endl;
	cout << "desire_vel_[1]:  "<< desire_vel_[1] <<endl;
	cout << "target_pose_[0]:  "<<  target_pose_[0] << endl;
	cout << "target_pose_[1]:  "<<  target_pose_[1] << endl;
	cout << "desire_pose_[0]:  "<<  desire_pose_[0] << endl;
	cout << "desire_pose_[1]:  "<<  desire_pose_[1] << endl;
	cout << "detect_state : " << detect_state << endl;
	desire_xyVel_[0] = desire_vel_[0];
	desire_xyVel_[1] = desire_vel_[1];
	OffboardControl_.send_body_velxy_posz_setpoint(desire_xyVel_,search_alt_);
*/
	switch(TrackingState)
	{
		case WAITING:
			if(apm_state_.mode != "GUIDED")//等待GUIDED模式
			{
				temp_pos_drone[0] = 0;
				temp_pos_drone[1] = 0;
				temp_pos_drone[2] = 2;
				OffboardControl_.send_pos_setpoint(temp_pos_drone, 0);
			}
			if(apm_state_.mode == "GUIDED")
			{
				temp_pos_drone[0] = 0;
				temp_pos_drone[1] = 0;
				temp_pos_drone[2] = 2;
				TrackingState = TRACKING;
				cout << "TRACKING" <<endl;
			}
			break;
		case CHECKING:
			if(apm_pose_[0] == 0 && apm_pose_[1] == 0) 			//没有位置信息则执行降落模式
			{
				cout << "Check error, make sure have local location" <<endl;
				mode_cmd_.request.custom_mode = "LAND";
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
			if((apm_pose_[2]<=search_alt_+0.1) && (apm_pose_[2]>=search_alt_-0.1))
			{
				cout << "PREPARE FINISH" <<endl;
				//TrackingState = SEARCH;
			}
			else if(detect_state.data == 1)
			{
			//	TrackingState = SEARCH;
			}
			OffboardControl_.send_pos_setpoint(posxyz_target, 0);					
			if(apm_state_.mode != "GUIDED")				//如果在准备中途中切换到onboard，则跳到WAITING
			{
				TrackingState = WAITING;
			}

			break;
		case SEARCH:
			if(detect_state.data == 1)
			{
				TrackingState = TRACKING;
			  cout << "TRACKING" <<endl;
			}	
			else//这里无人机没有主动搜寻目标
			{
				OffboardControl_.send_pos_setpoint(posxyz_target, 0);
			}
			if(apm_state_.mode != "GUIDED")				//如果在SEARCH途中切换到onboard，则跳到WAITING
			{
				TrackingState = WAITING;
			}
     // cout << "SEARCH" <<endl;
			break;
		case TRACKING:
			{
		//		if(detect_state.data == 1)
		//		{
					desire_vel_ = TrackingPidProcess(target_pose_,desire_pose_);

					//cout << "search_" <<endl;
		//		}
		//	  else
		//		{
		//			desire_vel_[0] = 0;
		//			desire_vel_[1] = 0;
		//		}
//	cout << "desire_vel_[0]:  "<< desire_vel_[0] <<endl;
//	cout << "desire_vel_[1]:  "<< desire_vel_[1] <<endl;
				if(apm_state_.mode != "GUIDED")			//如果在TRACKING中途中切换到onboard，则跳到WAITING
				{
				//	TrackingState = WAITING;
				}
				desire_xyVel_[0] = desire_vel_[0];
				desire_xyVel_[1] = desire_vel_[1];
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
//				desire_xyVel_[0] = 0.2;
//				desire_xyVel_[1] = 0;
				OffboardControl_.send_body_velxy_posz_setpoint(desire_xyVel_,search_alt_);
			}

			break;
		case TRACKOVER:
			{
				mode_cmd_.request.custom_mode = "LAND";
        set_mode_client_.call(mode_cmd_);
				TrackingState = WAITING;
			}

			break;

		default:
			cout << "error" <<endl;
	}	

}

/*接收目标相对飞机的像素点位置*/
void APMtracking::TargetPoseCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
      target_pose_[0] = 640-msg->position.x;
      target_pose_[1] = msg->position.y;
//			cout << "target_pose_[0]:"  << target_pose_[0] << endl;
//			cout << "target_pose_[1]:"  << target_pose_[1] << endl;

}

/*接收是否检测到目标标志位*/
void APMtracking::TargetStateCallback(const std_msgs::Int8::ConstPtr &msg)
{
      detect_state = *msg;

}
/*接收来自飞控的当前飞机位置*/                  
void APMtracking::ApmPosCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // Read the Drone Position from the Mavros Package [Frame: ENU]
    Eigen::Vector3d pos_drone_fcu_enu(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);

    apm_pose_ = pos_drone_fcu_enu;
    cout << "apm_pose_[0]:"  << apm_pose_[0] << endl;
    cout << "apm_pose_[1]:"  << apm_pose_[1] << endl;
}
/*接收来自飞控的当前飞机状态*/
void APMtracking::ApmStateCallback(const mavros_msgs::State::ConstPtr& msg)
{
	apm_state_ = *msg;
}

/*初始化*/
void APMtracking::Initialize()
{
  //读取GUIDED模式下飞机的高度
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

  detect_state.data = 0;
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
  ros::init(argc,argv,"tracking_ardupilot");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  APMtracking APMtracking(nh, nh_private);

  ros::spin();
  return 0;
}
