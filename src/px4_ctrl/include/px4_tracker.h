#include "px4_cmd.h"

#include <std_msgs/Bool.h>
#include <std_msgs/UInt32.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // 用于TF2库和ROS消息类型之间相互转换
#include <robot_vision/BoundingBoxes.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

class PX4Tracker
{
public:
    PX4Tracker(const ros::NodeHandle &nh); // 构造函数
    void Initialize();                     // 参数初始化

    PX4Cmd px4cmd_; // 用来发送PX4控制指令

private:
    ros::NodeHandle nh_; // 节点句柄

    ros::Subscriber state_sub_;    // 订阅飞机mavros状态
    ros::Subscriber position_sub_; // 订阅飞机local坐标系位置

    ros::Subscriber yolotag_sub_;  // 订阅目标平台中心图像坐标
    ros::Subscriber apriltag_sub_; // 订阅目标平台相对无人机的位置

    ros::ServiceClient arming_client_;   // 用于解锁飞机
    ros::ServiceClient set_mode_client_; // 用于修改飞行模式

    ros::Timer cmdloop_timer_; // 定时器

    float search_alt_, track_alt_; // 搜索高度和跟踪高度

    Eigen::Vector2d desire_imgc_;  // 期望的图像中心坐标
    Eigen::Vector2d yolotag_imgc_; // 检测框的中心坐标

    Eigen::Vector3d desire_pose_;   // 期望的飞机相对合作目标的位置
    float desire_yaw_;              // 期望的飞机相对合作目标的偏航角
    Eigen::Vector3d apriltag_pose_; // apriltags 合作目标相对飞机位置
    float apriltag_yaw_;            // 合作目标相对飞机的偏航角

    bool detect_track_state, detect_land_state; // 是否检测到合作目标

    mavros_msgs::State px4_state_; // 飞机的状态
    mavros_msgs::CommandBool arm_cmd_;
    mavros_msgs::SetMode mode_cmd_;

    Eigen::Vector3d px4_pose_;      // 接收飞控的东北天local坐标
    Eigen::Vector3d temp_pos_drone; // 临时存储飞机位置
    Eigen::Vector3d posxyz_target;  // 期望的飞机位置

    Eigen::Vector4d desire_vel_;
    Eigen::Vector3d desire_xyzVel_;
    float desire_yawVel_;

    S_PID i_PidXY, i_PidZ; // 图像伺服PID
    S_PID_ITEM i_PidItemX, i_PidItemY, i_PidItemZ;

    S_PID p_PidXY, p_PidZ, p_PidYaw; // 位置伺服PID
    S_PID_ITEM p_PidItemX, p_PidItemY, p_PidItemZ, p_PidItemYaw;

    enum
    {
        WAITING,          // 等待offboard模式
        PREPARING,        // 起飞到指定高度
        SEARCHING,        // 搜索目标
        TRACKING,         // 检测到二维码，开始跟踪
        LANDING,          // 检测到降落板，开始降落
        LANDOVER,         // 结束
    } FlyState = WAITING; // 初始状态WAITING

    void Px4StateCallback(const mavros_msgs::State::ConstPtr &msg);
    void Px4PosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void YoloPoseCallback(const robot_vision::BoundingBox::ConstPtr &msg);
    void AprilPoseCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);
    Eigen::Vector4d TrackerPidProcess(Eigen::Vector2d &currentPos, Eigen::Vector2d &expectPos);
    Eigen::Vector4d TrackerPidProcess(Eigen::Vector3d &currentPos, float currentYaw, Eigen::Vector3d &expectPos, float expectYaw);
    void CmdLoopCallback(const ros::TimerEvent &event);
    void TrackerStateUpdate();
};
