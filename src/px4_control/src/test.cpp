#include "landing_quadrotor.h"

using namespace std;
using namespace Eigen;

/**
  * @brief      PX4Landing 构造函数          
  * @param[in]  nh          ros::NodeHandle 类型的引用            
  * @param[in]  nh_private  ros::NodeHandle 类型的引用
  *             ：后面表示将构造函数参数值赋给 PX4Landing 类的成员变量。
  **/
PX4Landing::PX4Landing(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):nh_(nh),nh_private_(nh_private) 
{
    Initialize();

    // 用全局句柄创建定时器，周期为0.1s，定时器触发回调函数，this表示回调函数属于哪个对象
    cmdloop_timer_ = nh_.createTimer(ros::Duration(0.1), &PX4Landing::CmdLoopCallback, this);

    // 订阅无人机当前状态
    state_sub_ = nh_private_.subscribe("/mavros/state", 1, &PX4Landing::Px4StateCallback, this, ros::TransportHints().tcpNoDelay());
    // 订阅无人机local坐标系位置
    position_sub_ = nh_private_.subscribe("/mavros/local_position/pose", 1, &PX4Landing::Px4PosCallback, this, ros::TransportHints().tcpNoDelay());
    // 订阅降落板相对飞机位置
    ar_pose_sub_ = nh_private_.subscribe("/ar_pose_marker", 1, &PX4Landing::ArPoseCallback, this, ros::TransportHints().tcpNoDelay());

    // 创建修改系统模式的客户端
    set_mode_client_ = nh_private_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
}

/**
  * @brief      参数初始化         
  **/
void PX4Landing::Initialize()
{
    // 读取offboard模式下飞机的搜索高度和搜索ID
    nh_private_.param<float>("search_alt_", search_alt_, 3);
    nh_private_.param<float>("markers_id_", markers_id_, 4.0);

    // 无人机降落时的PID参数
    nh_private_.param<float>("PidXY_p", s_PidXY.p, 0.4);
    nh_private_.param<float>("PidXY_d", s_PidXY.d, 0.05);
    nh_private_.param<float>("PidXY_i", s_PidXY.i, 0.01);
    nh_private_.param<float>("PidZ_p", s_PidZ.p, 0.1);
    nh_private_.param<float>("PidZ_d", s_PidZ.d, 0);
    nh_private_.param<float>("PidZ_i", s_PidZ.i, 0);
    nh_private_.param<float>("PidYaw_p", s_PidYaw.p, 0);
    nh_private_.param<float>("PidYaw_d", s_PidYaw.d, 0);
    nh_private_.param<float>("PidYaw_i", s_PidYaw.i, 0);

    // 期望的飞机相对降落板的位置
    float desire_pose_x,desire_pose_y,desire_pose_z;
    nh_private_.param<float>("desire_pose_x", desire_pose_x, 0);
    nh_private_.param<float>("desire_pose_y", desire_pose_y, 0);
    nh_private_.param<float>("desire_pose_z", desire_pose_z, 0);
    nh_private_.param<float>("desire_yaw_", desire_yaw_, 0);
    desire_pose_[0] = desire_pose_x;
    desire_pose_[1] = desire_pose_y;
    desire_pose_[2] = desire_pose_z;

    detect_state = false;
    desire_vel_[0] = 0;
    desire_vel_[1] = 0;
    desire_vel_[2] = 0;
    desire_vel_[3] = 0;
    desire_xyzVel_[0]  = 0;
    desire_xyzVel_[1]  = 0;
    desire_xyzVel_[2]  = 0;
    desire_yawVel_ = 0;
    
    s_PidItemX.tempDiffer = 0;
    s_PidItemY.tempDiffer = 0;
    s_PidItemZ.tempDiffer = 0;
    s_PidItemYaw.tempDiffer = 0;
    s_PidItemX.intergral = 0;
    s_PidItemY.intergral = 0;
    s_PidItemZ.intergral = 0;
    s_PidItemYaw.intergral = 0;
}

/**
  * @brief      PID 控制程序          
  * @param[in]  &currentPos 当前飞机相对降落板的位置,currentYaw 当前飞机相对降落板的方向            
  * @param[in]  &expectPos 期望位置，expectYaw 飞机相对降落板的期望方向:默认0
  * @param[out] 机体系下x,y,z的期望速度,以及yaw方向的期望速度。
  **/
Eigen::Vector4d PX4Landing::LandingPidProcess(Eigen::Vector3d &currentPos,float currentYaw,Eigen::Vector3d &expectPos,float expectYaw)
{
    Eigen::Vector4d s_PidOut;

	/* X方向的pid控制 */
	s_PidItemX.difference = expectPos[0] - currentPos[0];
	s_PidItemX.intergral += s_PidItemX.difference;

	if(s_PidItemX.intergral >= 100)		
	    s_PidItemX.intergral = 100;
	else if(s_PidItemX.intergral <= -100) 
		s_PidItemX.intergral = -100;

	s_PidItemX.differential =  s_PidItemX.difference - s_PidItemX.tempDiffer;
    s_PidItemX.tempDiffer = s_PidItemX.difference;

	s_PidOut[0] = s_PidXY.p*s_PidItemX.difference + s_PidXY.d*s_PidItemX.differential + s_PidXY.i*s_PidItemX.intergral;

	/* Y方向的pid控制 */
	s_PidItemY.difference = expectPos[1] - currentPos[1];
	s_PidItemY.intergral += s_PidItemY.difference;

	if(s_PidItemY.intergral >= 100)		
		s_PidItemY.intergral = 100;
	else if(s_PidItemY.intergral <= -100) 
		s_PidItemY.intergral = -100;

	s_PidItemY.differential =  s_PidItemY.difference - s_PidItemY.tempDiffer;
    s_PidItemY.tempDiffer = s_PidItemY.difference;

	s_PidOut[1] = s_PidXY.p*s_PidItemY.difference + s_PidXY.d*s_PidItemY.differential + s_PidXY.i*s_PidItemY.intergral;

	/* Z方向的pid控制 */
	s_PidItemZ.difference = expectPos[2] - currentPos[2];
	s_PidItemZ.intergral += s_PidItemZ.difference;

	if(s_PidItemZ.intergral >= 100)		
		s_PidItemZ.intergral = 100;
	else if(s_PidItemZ.intergral <= -100) 
		s_PidItemZ.intergral = -100;

	s_PidItemZ.differential =  s_PidItemZ.difference - s_PidItemZ.tempDiffer;
    s_PidItemZ.tempDiffer = s_PidItemZ.difference;

	s_PidOut[2] = s_PidZ.p*s_PidItemZ.difference + s_PidZ.d*s_PidItemZ.differential + s_PidZ.i*s_PidItemZ.intergral;

	/* Yaw方向的pid控制 */
	s_PidItemYaw.difference =  expectYaw - currentYaw;
	s_PidItemYaw.intergral += s_PidItemYaw.difference;

	if(s_PidItemYaw.intergral >= 100)		
		s_PidItemYaw.intergral = 100;
	else if(s_PidItemYaw.intergral <= -100) 
		s_PidItemYaw.intergral = -100;

	s_PidItemYaw.differential =  s_PidItemYaw.difference - s_PidItemYaw.tempDiffer;
    s_PidItemYaw.tempDiffer = s_PidItemYaw.difference;

	s_PidOut[3] = s_PidYaw.p*s_PidItemYaw.difference + s_PidYaw.d*s_PidItemYaw.differential + s_PidYaw.i*s_PidItemYaw.intergral;

	return s_PidOut;
}

/**
  * @brief      0.1s 状态机更新函数         
  **/
void PX4Landing::CmdLoopCallback(const ros::TimerEvent& event)
{
    LandingStateUpdate();
}

/* 状态机更新函数 */
void PX4Landing::LandingStateUpdate()
{
	switch(FlyState)
	{
		case WAITING:
			if(px4_state_.mode == "AUTO.RTL" && detect_state == true)   // 进入返航模式，同时检测到二维码
			{
				mode_cmd_.request.custom_mode = "OFFBOARD";
                // 请求修改飞行模式的服务
				set_mode_client_.call(mode_cmd_);
			}

			if(px4_state_.mode != "OFFBOARD")//等待offboard模式
			{
				temp_pos_drone[0] = px4_pose_[0];
				temp_pos_drone[1] = px4_pose_[1];
				temp_pos_drone[2] = px4_pose_[2];
				OffboardControl_.send_pos_setpoint(temp_pos_drone, 0);
			}
			else
			{
				temp_pos_drone[0] = px4_pose_[0];
				temp_pos_drone[1] = px4_pose_[1];
				temp_pos_drone[2] = px4_pose_[2];
				FlyState = CHECKING;
				cout << "CHECKING" <<endl;
			}          
			break;
		case CHECKING:
			if(px4_pose_[0] == 0 && px4_pose_[1] == 0)  //没有位置信息则执行降落模式
			{
				cout << "Check error, make sure have local location" <<endl;
				mode_cmd_.request.custom_mode = "AUTO.LAND";
                // 请求修改飞行模式的服务
				set_mode_client_.call(mode_cmd_);
				FlyState = WAITING;	
			}
			else
			{
				FlyState = PREPARE;
				cout << "PREPARE" <<endl;
			}	
			break;
		case PREPARE:   // 起飞到指定高度
			posxyz_target[0] = temp_pos_drone[0];
			posxyz_target[1] = temp_pos_drone[1];
			posxyz_target[2] = search_alt_;
            
			if((px4_pose_[2]<=search_alt_+0.1) && (px4_pose_[2]>=search_alt_-0.1))
			{
				FlyState = SEARCH;
                cout << "SEARCH" <<endl;
			}
			OffboardControl_.send_pos_setpoint(posxyz_target, 0);

            // 如果在准备中途中切换到onboard，则跳到WAITING
			if(px4_state_.mode != "OFFBOARD")   
			{
				FlyState = WAITING;
			}
			break;
		case SEARCH:
			posxyz_target[0] = 0;
			posxyz_target[1] = 0;
			posxyz_target[2] = 6;
			OffboardControl_.send_pos_setpoint(posxyz_target, 0);

            // desire_xyzVel_[0] = 0;
            // desire_xyzVel_[1] = 0;
            // desire_xyzVel_[2] = 0.5;
            // desire_yawVel_ = 0;
            // OffboardControl_.send_body_velxyz_setpoint(desire_xyzVel_, desire_yawVel_);

			// if(detect_state == true)
			// {
			// 	FlyState = LANDING;
			//     cout << "LANDING" <<endl;
			// }	
			// else    // 这里无人机没有主动搜寻目标
			// {
            //     posxyz_target[0] = px4_pose_[0];
			// 	posxyz_target[1] = px4_pose_[1];
			// 	posxyz_target[2] = search_alt_;
			// 	OffboardControl_.send_pos_setpoint(posxyz_target, 0);
            //     cout << "SEARCHING TARGET" <<endl;
			// }

            // 如果在SEARCH途中切换到onboard，则跳到WAITING
			if(px4_state_.mode != "OFFBOARD")   
			{
				FlyState = WAITING;
			}
			break;
		case LANDING:
			{
				if(detect_state == true)
				{               
					desire_vel_ = LandingPidProcess(ar_pose_,markers_yaw_, desire_pose_, desire_yaw_);

                    desire_xyzVel_[0] = desire_vel_[1];
                    desire_xyzVel_[1] = desire_vel_[0];
                    desire_xyzVel_[2] = desire_vel_[2];
                    desire_yawVel_ = desire_vel_[3];

                    OffboardControl_.send_body_velxyz_setpoint(desire_xyzVel_, desire_yawVel_);
				}
			    else
				{
                    FlyState = SEARCH;
                    cout << "SEARCH" <<endl;
				}

				if(ar_pose_[2] <= 0.8)
				{
					FlyState = LANDOVER;
					cout << "LANDOVER" <<endl;
				}

                // 如果在LANDING中途中切换到onboard，则跳到WAITING
				if(px4_state_.mode != "OFFBOARD")   
				{
					FlyState = WAITING;
				}
			}
			break;
		case LANDOVER:
			{
				mode_cmd_.request.custom_mode = "AUTO.LAND";
                // 请求修改飞行模式的服务
        		set_mode_client_.call(mode_cmd_);
				FlyState = WAITING;
			}
			break;
		default:
			cout << "error" <<endl;
	}	
}

/**
  * @brief      接收降落板相对无人机的位置以及偏航角       
  **/
void PX4Landing::ArPoseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
    detect_state = false;
    double temp_roll,temp_pitch,temp_yaw;
    tf::Quaternion quat;

    // C++11引入的范围for循环，遍历接收到的AR标签消息，&item引用代表msg->markers中的当前元素
    for(auto &item : msg->markers)
    {
        // 如果标签的ID与预期的ID匹配
        if(item.id == markers_id_)
        {
            detect_state = true;
            // 获取标签在相机坐标系中的位置信息
            ar_pose_[0] = item.pose.pose.position.x;
            ar_pose_[1] = item.pose.pose.position.y;
            ar_pose_[2] = item.pose.pose.position.z;
            // 获取标签在相机坐标系中的姿态信息（四元数），并将其转换为欧拉角
            tf::quaternionMsgToTF(item.pose.pose.orientation,quat);
            tf::Matrix3x3(quat).getRPY(temp_roll,temp_pitch,temp_yaw);
            // 更新标签的yaw角度
            markers_yaw_ = temp_yaw;
        }
    }
}

/**
  * @brief      接收来自飞控的当前飞机local坐标系位置      
  **/                 
void PX4Landing::Px4PosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // 读取无人机在local坐标系中的位置，东北天，坐标原点在PX4上电的地方
    Eigen::Vector3d pos_drone_fcu_enu(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);

    px4_pose_ = pos_drone_fcu_enu;
}

/**
  * @brief      接收来自飞控的当前飞机状态     
  **/  
void PX4Landing::Px4StateCallback(const mavros_msgs::State::ConstPtr& msg)
{
	px4_state_ = *msg;
}

/**
  * @brief      主函数     
  **/ 
int main(int argc, char** argv) 
{
    ros::init(argc,argv,"landing_quadrotor");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    // 隐式调用构造函数初始化对象
    PX4Landing PX4Landing(nh, nh_private);

    ros::spin();
    return 0;
}
