#include <ros/ros.h>
#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <stdio.h>
#include <Eigen/Eigen>

#include <mavros_msgs/State.h>              // 订阅查看无人机的状态
#include <mavros_msgs/SetMode.h>            // 设置无人机的飞行模式
#include <geometry_msgs/PoseStamped.h>      // 用来进行发送目标位置
#include <mavros_msgs/PositionTarget.h>     // 用来进行发送目标速度

using namespace std;

mavros_msgs::State current_state;

mavros_msgs::SetMode mode_cmd;
ros::ServiceClient set_mode_client;

enum
{
    WAITING,		//等待offboard模式
	CHECKING,		//检查飞机状态
	PREPARE,		//起飞到第一个点
	REST,			//休息一下
	FLY,			//飞圆形路经
	FLYOVER,		//结束		
}FlyState;

Eigen::Vector3d pos_drone;
Eigen::Vector3d temp_pos_drone;
Eigen::Vector3d temp_pos_target;
Eigen::Vector3d pos_target;// offboard模式下，发送给飞控的期望值
ros::Publisher setpoint_raw_local_pub;

float desire_z = 1; // 期望高度
float desire_Radius = 1;    // 期望圆轨迹半径

float MoveTimeCnt = 0;
float priod = 1000.0;   // 减小数值可增大飞圆形的速度

/**
  * @brief      接收来自飞控的当前飞机状态        
  **/
void state_cb(const mavros_msgs::State::ConstPtr& msg) 
{
	current_state = *msg;
}

/**
  * @brief      接收来自飞控的当前飞机local位置       
  **/                 
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Read the Drone Position from the Mavros Package [Frame: ENU 东北天]
    Eigen::Vector3d pos_drone_fcu_enu(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);

    pos_drone = pos_drone_fcu_enu;
}

/**
  * @brief      发送位置期望值至飞控（输入：期望xyz,期望yaw）      
  **/ 
void send_pos_setpoint(const Eigen::Vector3d& pos_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;
    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    pos_setpoint.type_mask = 0b000111111000;  // 000 111 111 000  xyz + yaw

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = pos_sp[0];
    pos_setpoint.position.y = pos_sp[1];
    pos_setpoint.position.z = pos_sp[2];

    pos_setpoint.yaw = yaw_sp;

    setpoint_raw_local_pub.publish(pos_setpoint);
}

/**
  * @brief      状态机更新函数     
  **/ 
void FlyState_update(void)
{
	switch(FlyState)
	{
		case WAITING:
			if(current_state.mode != "OFFBOARD")//等待offboard模式
			{
				temp_pos_drone[0] = pos_drone[0];
				temp_pos_drone[1] = pos_drone[1];
				temp_pos_drone[2] = pos_drone[2];
                send_pos_setpoint(temp_pos_drone, 0);   // 在进入 OFFBOARD 模式之前，必须已经开始流式传输设定点。否则模式开关将被拒绝。
			}
			else
			{
				pos_target[0] = temp_pos_drone[0];
				pos_target[1] = temp_pos_drone[1];
				pos_target[2] = temp_pos_drone[2];
				send_pos_setpoint(pos_target, 0);

				FlyState = CHECKING;
                cout << "CHECKING" <<endl;
			}
			break;
		case CHECKING:
			if(pos_drone[0] == 0 && pos_drone[1] == 0) 			//没有位置信息则执行降落模式
			{
				cout << "Check error, make sure have local location" <<endl;

				mode_cmd.request.custom_mode = "AUTO.LAND";
				set_mode_client.call(mode_cmd);

				FlyState = WAITING;	
			}
			else
			{
				FlyState = PREPARE;
                cout << "PREPARE" <<endl;
				MoveTimeCnt = 0;
			}
			break;
		case PREPARE:   // 起飞到圆轨迹的第一个点,起点在X负半轴
			temp_pos_target[0] = temp_pos_drone[0] - desire_Radius;
			temp_pos_target[1] = temp_pos_drone[1];
			temp_pos_target[2] = desire_z;

            // 分500步进行，没有直接飞到指定点，是为了实现到达后跳转到 REST
			MoveTimeCnt +=2;
			if(MoveTimeCnt >=500)
			{
				FlyState = REST;
                cout << "REST" <<endl;
				MoveTimeCnt = 0;
			}

			pos_target[0]=temp_pos_drone[0]+(temp_pos_target[0]-temp_pos_drone[0])*(MoveTimeCnt/500);
			pos_target[1]=temp_pos_drone[1]+(temp_pos_target[1]-temp_pos_drone[1])*(MoveTimeCnt/500);
			pos_target[2]=temp_pos_drone[2]+(temp_pos_target[2]-temp_pos_drone[2])*(MoveTimeCnt/500);
			send_pos_setpoint(pos_target, 0);					
			if(current_state.mode != "OFFBOARD")				// 如果在准备中途中切换到onboard，则跳到WAITING
			{
				FlyState = WAITING;
			}
			break;
		case REST:  // 确保到达起始点
			pos_target[0] = temp_pos_drone[0] - desire_Radius;
			pos_target[1] = temp_pos_drone[1] ;
			pos_target[2] = desire_z;
			send_pos_setpoint(pos_target, 0);

			MoveTimeCnt +=1;
			if(MoveTimeCnt >= 100)
			{
				MoveTimeCnt = 0;
				FlyState = FLY;
                cout << "FLY" <<endl;
			}

			if(current_state.mode != "OFFBOARD")				// 如果在REST途中切换到onboard，则跳到WAITING
			{
				FlyState = WAITING;
			}
			break;
		case FLY:
			{
				float phase = 3.1415926;						//起点在X负半轴
				float Omega = 2.0*3.14159*MoveTimeCnt / priod;	//0~2pi
				MoveTimeCnt += 3;								//调此数值可改变飞圆形的速度
				if(MoveTimeCnt >=priod)							//走一个圆形周期
				{
					FlyState = FLYOVER;
                    cout << "FLYOVER" <<endl;
				}

				pos_target[0] = temp_pos_drone[0]+desire_Radius*cos(Omega+phase);			 
				pos_target[1] = temp_pos_drone[1]+desire_Radius*sin(Omega+phase); 
				pos_target[2] = desire_z;
				send_pos_setpoint(pos_target, 0);
                //send_body_velxyz_setpoint();

				if(current_state.mode != "OFFBOARD")			//如果在飞圆形中途中切换到onboard，则跳到WAITING
				{
					FlyState = WAITING;
				}
			}
			break;
		case FLYOVER:   // 飞行结束后的动作
			{
				mode_cmd.request.custom_mode = "AUTO.LAND";
                set_mode_client.call(mode_cmd);
				FlyState = WAITING;
			}
			break;
		default:
			cout << "error" <<endl;
	}	
}				

/**
  * @brief      0.1s 状态机更新函数         
  **/
void CmdLoopCallback(const ros::TimerEvent& event)
{
    FlyState_update();
}

/**
  * @brief      主函数         
  **/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "circular_offboard");

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    // 用全局句柄创建定时器，周期为0.1s，定时器触发回调函数，this表示回调函数属于哪个对象
    ros::Timer cmdloop_timer_ = nh.createTimer(ros::Duration(0.1), CmdLoopCallback);

    // 获取launch文件中的节点局部参数
   	nh_private.param<float>("desire_z", desire_z, 1.0);
   	nh_private.param<float>("desire_Radius", desire_Radius, 1.0);
    // 启动修改系统模式的服务
    set_mode_client = nh_private.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // 订阅无人机的状态
    ros::Subscriber state_sub = nh_private.subscribe<mavros_msgs::State>("/mavros/state", 1, state_cb);
    // 订阅无人在local世界坐标系的位姿
    ros::Subscriber position_sub = nh_private.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, pos_cb);

    // 发布无人机目标位置
    setpoint_raw_local_pub = nh_private.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    FlyState = WAITING;

    ros::spin();
    return 0;
}
