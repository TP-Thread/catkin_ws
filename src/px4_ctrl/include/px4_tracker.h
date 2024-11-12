#include "px4_cmd.h"

#include <std_msgs/Bool.h>
#include <std_msgs/UInt32.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // 用于TF2库和ROS消息类型之间相互转换
#include <robot_vision/BoundingBoxes.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

class PX4Tracker
{
public:
    // 构造函数
    PX4Tracker(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

    void Initialize();
    PX4Cmd px4cmd_;

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Timer cmdloop_timer_;

    ros::Subscriber state_sub_;
    ros::Subscriber position_sub_;

    ros::Subscriber yolotag_sub_;
    ros::Subscriber apriltag_sub_;

    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;

    mavros_msgs::State px4_state_; // 飞机的状态
    mavros_msgs::CommandBool arm_cmd_;
    mavros_msgs::SetMode mode_cmd_;

    void CmdLoopCallback(const ros::TimerEvent &event);
    void TrackerStateUpdate();
    void YoloPoseCallback(const robot_vision::BoundingBoxes::ConstPtr &msg);
    void AprilPoseCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);
    void Px4PosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void Px4StateCallback(const mavros_msgs::State::ConstPtr &msg);
    Eigen::Vector4d TrackerPidProcess(Eigen::Vector2d &currentPos, Eigen::Vector2d &expectPos);
    Eigen::Vector4d TrackerPidProcess(Eigen::Vector3d &currentPos, float currentYaw, Eigen::Vector3d &expectPos, float expectYaw);

    Eigen::Vector3d px4_pose_; // 接收飞控的东北天local坐标
    Eigen::Vector3d temp_pos_drone;
    Eigen::Vector3d posxyz_target; // 期望飞机的空间位置
    float search_alt_, track_alt_;

    bool detect_track_state, detect_land_state; // 是否检测到降落板标志位

    Eigen::Vector2d desire_imgc_;  // 图像中心坐标和无人机期望跟踪高度
    Eigen::Vector2d yolotag_imgc_; // 检测框的中心坐标和无人机当前高度

    Eigen::Vector3d desire_pose_; // 期望的飞机相对降落板的位置
    float desire_yaw_;            // 期望的飞机相对降落板的偏航角

    Eigen::Vector3d apriltag_pose_; // apriltags 降落板相对飞机位置
    float apriltag_yaw_;            // 二维码相对飞机的偏航角

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
};
