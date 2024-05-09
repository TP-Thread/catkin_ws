#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // 用于TF2库和ROS消息类型之间相互转换
#include <Eigen/Eigen>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt32.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include "offboard_control.h"
#include "px4_control_cfg.h"
#include <iostream>

class PX4Landing
{
public:
    // 构造函数
    PX4Landing(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

    void Initialize();
    OffboardControl OffboardControl_;

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Timer cmdloop_timer_;

    ros::Subscriber apriltag_sub_;
    ros::Subscriber position_sub_;
    ros::Subscriber state_sub_;
    ros::ServiceClient set_mode_client_;

    mavros_msgs::State px4_state_; // 飞机的状态
    mavros_msgs::SetMode mode_cmd_;

    void CmdLoopCallback(const ros::TimerEvent &event);
    void LandingStateUpdate();
    void AprilPoseCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);
    void Px4PosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void Px4StateCallback(const mavros_msgs::State::ConstPtr &msg);
    Eigen::Vector4d LandingPidProcess(Eigen::Vector3d &currentPos, float currentYaw, Eigen::Vector3d &expectPos, float expectYaw);

    Eigen::Vector3d px4_pose_; // 接收飞控的东北天local坐标
    Eigen::Vector3d temp_pos_drone;
    Eigen::Vector3d posxyz_target; // 期望飞机的空间位置

    bool detect_state;             // 是否检测到降落板标志位
    Eigen::Vector3d desire_pose_;  // 期望的飞机相对降落板的位置
    float desire_yaw_;             // 期望的飞机相对降落板的偏航角
    Eigen::Vector3d markers_pose_; // apriltags 降落板相对飞机位置
    float markers_yaw_;            // 二维码相对飞机的偏航角
    Eigen::Vector4d desire_vel_;
    Eigen::Vector3d desire_xyzVel_;
    float desire_yawVel_;

    S_PID s_PidXY, s_PidZ, s_PidYaw;
    S_PID_ITEM s_PidItemX;
    S_PID_ITEM s_PidItemY;
    S_PID_ITEM s_PidItemZ;
    S_PID_ITEM s_PidItemYaw;

    float search_alt_;
    float marker1_id_; // 需要检测到的二维码，默认是4
    float marker2_id_; // 需要检测到的二维码，默认是4

    Eigen::Vector3d april1_pose_; // apriltag1 降落板相对飞机位置
    Eigen::Vector3d april2_pose_; // apriltag2 降落板相对飞机位置
    float marker1_yaw_;           // 二维码相对飞机的偏航角
    float marker2_yaw_;           // 二维码相对飞机的偏航角

    enum
    {
        WAITING,          // 等待offboard模式
        SEARCHING,        // 起飞到指定高度搜索目标
        CHECKING,         // 检查合作目标
        LANDING,          // 检测到降落板，开始降落
        LANDOVER,         // 结束
    } FlyState = WAITING; // 初始状态WAITING
};
