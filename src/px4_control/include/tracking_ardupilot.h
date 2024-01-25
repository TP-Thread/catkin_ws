#include <ros/ros.h>
#include <iostream>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int8.h>
#include "offboard_control.h"
#include "px4_control_cfg.h"
#include <tf/transform_datatypes.h>
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
using namespace std;
using namespace Eigen;
class APMtracking {
 public:
    /**
     *默认构造函数
     */
    APMtracking(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    /**
     * 析构函数
     */
    ~APMtracking();
    void Initialize();
   OffboardControl OffboardControl_;
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Timer cmdloop_timer_;
  void CmdLoopCallback(const ros::TimerEvent& event);
  void TrackingStateUpdate();
  void TargetPoseCallback(const geometry_msgs::Pose::ConstPtr &msg);
  void TargetStateCallback(const std_msgs::Int8::ConstPtr &msg);
  void ApmPosCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void ApmStateCallback(const mavros_msgs::State::ConstPtr& msg);
  Eigen::Vector3d TrackingPidProcess(Eigen::Vector3d &currentPos,Eigen::Vector3d &expectPos);
  Eigen::Vector3d temp_pos_drone;
  Eigen::Vector3d posxyz_target;//期望飞机的空间位置
  Eigen::Vector3d velxy_posz_target;//GUIDED模式下，发送给飞控的期望值
  Eigen::Vector3d  target_pose_;  //目标相对飞机在图像像素点的位置
  Eigen::Vector3d  apm_pose_; //接收来自飞控的飞机位置 
  Eigen::Vector3d desire_pose_;//期望的飞机相对降落板的位置
  mavros_msgs::State apm_state_;//飞机的状态
  mavros_msgs::SetMode mode_cmd_;
  float search_alt_;//跟踪高度
  std_msgs::Int8 detect_state;//是否检测到目标标志位
  Eigen::Vector3d desire_vel_;
  Eigen::Vector3d desire_xyVel_;
  S_PID s_PidXY,s_PidZ;
  S_PID_ITEM s_PidItemX;
  S_PID_ITEM s_PidItemY;
  enum
 {
  WAITING,		//等待GUIDED模式
  CHECKING,		//检查飞机状态
  PREPARE,		//起飞到指定高度
  SEARCH,		//搜索
  TRACKING,	        //检测到目标，开始跟踪
  TRACKOVER,		//结束		
}TrackingState = WAITING;//初始状态WAITING

  ros::Subscriber target_pose_sub_;
  ros::Subscriber target_state_sub_;
  ros::Subscriber position_sub_;
  ros::Subscriber state_sub_;
  ros::ServiceClient set_mode_client_;
};
