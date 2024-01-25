/***************************************************************************************************************************
*
* Author: bingo
* Email: 1554459957@qq.com
* Time: 2019.09.19
* Description: simulink vfh control px4, simulink程序见volans/src/modules/matlab/simulink_vfh/simulinkVFH.slx
*  
***************************************************************************************************************************/

//ROS 头文件
#include <ros/ros.h>
#include <Eigen/Eigen>

//自定义头文件
#include "offboard_control.h"

//topic 头文件
#include <iostream>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>

using namespace std;
float desire_vx = 0; 
float desire_yaw = 0; 
Eigen::Vector3d desire_vel;//期望速度，只控制机体坐标系下X方向，即机头方向
float desire_yawRate = 0.2;//期望角速度 ，+时飞机从上往下看逆时针旋转。

float desire_z = 1.5; //期望高度  
/*接收来自simulink的期望x方向的速度，以及期望偏航角，其中angular.z为正表示从上往下看飞机应该逆时针旋转，当为0时表示此刻机头对应的区域可以安全通行，msg->linear.x表示此刻期望的飞机前进速度*/                               
void targetMsg_cb(const geometry_msgs::Twist::ConstPtr &msg)
{
  desire_vel[0] =  msg->linear.x;
  desire_yawRate =  msg->angular.z;
  //ROS_INFO("I heard: [%f],[%f]", desire_vx,desire_yaw);
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "simulinkVFH");
    ros::NodeHandle nh("~");
    ros::Rate rate(20.0);
    OffboardControl OffboardControl_; 
    //【订阅】Lidar数据
    ros::Subscriber lidar_sub = nh.subscribe<geometry_msgs::Twist>("/simulink/cmd_vel", 100, targetMsg_cb);

    ros::Publisher setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);

    nh.param<float>("desire_z", desire_z, 1.5);
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {

		ros::spinOnce();
		OffboardControl_.send_body_velxy_posz_yaw_setpoint(desire_vel,desire_z,desire_yawRate);
		rate.sleep();
    }
    return 0;

}



