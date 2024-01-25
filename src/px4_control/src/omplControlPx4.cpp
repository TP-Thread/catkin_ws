/***************************************************************************************************************************
*
* Author: bingo
* Email: 1554459957@qq.com
* Time: 2019.10.29
* Description: 接收来自路径规划发来的waypoints，继而控制无人机走规划好的航点
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
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
using namespace std;

Eigen::Vector3d desire_pos;//期望位置
float desire_yaw = 0;//期望航向角

                             
void targetMsg_cb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &msg)
{
  size_t n = msg->points.size() - 1;
  desire_pos[0] =  msg->points[n].transforms[0].translation.x;
  desire_pos[1] =  msg->points[n].transforms[0].translation.y;
  desire_pos[2] =  msg->points[n].transforms[0].translation.z;
  //ROS_INFO("I heard: [%f],[%f]", desire_pos,desire_yaw);
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "omplControlPx4");
    ros::NodeHandle nh("~");
    ros::Rate rate(20.0);
    OffboardControl OffboardControl_; 
    //【订阅】期望航点
    ros::Subscriber traj_sub = nh.subscribe<trajectory_msgs::MultiDOFJointTrajectory>("/px4/trajectory", 10, targetMsg_cb);

    ros::Publisher setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {

		ros::spinOnce();
		OffboardControl_.send_pos_setpoint(desire_pos,desire_yaw);
		rate.sleep();
    }
    return 0;

}



