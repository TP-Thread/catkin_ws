/**
 * @file    px4_landing.cpp
 * @brief   实现 PX4 二维码降落
 */

#include "px4_tracker.h"

using namespace std;

/**
 * @brief  PX4Tracker 构造函数
 * @param  nh          ros::NodeHandle 类型的引用
 * @param  nh_private  ros::NodeHandle 类型的引用
 *          ：后面表示将构造函数参数值赋给 PX4Tracker 类的成员变量。
 **/
PX4Tracker::PX4Tracker(const ros::NodeHandle &nh) : nh_(nh)
{
    Initialize();

    // 用全局句柄创建定时器，周期为0.1s，定时器触发回调函数，this表示回调函数属于哪个对象
    cmdloop_timer_ = nh_.createTimer(ros::Duration(0.1), &PX4Tracker::CmdLoopCallback, this);

    // 订阅无人机当前状态
    state_sub_ = nh_.subscribe("/mavros/state", 1, &PX4Tracker::Px4StateCallback, this, ros::TransportHints().tcpNoDelay());
    // 订阅无人机local坐标系位置
    position_sub_ = nh_.subscribe("/mavros/local_position/pose", 1, &PX4Tracker::Px4PosCallback, this, ros::TransportHints().tcpNoDelay());
    // 订阅降落板相对飞机位置
    apriltag_sub_ = nh_.subscribe("/tag_detections", 1, &PX4Tracker::AprilPoseCallback, this, ros::TransportHints().tcpNoDelay());

    // 创建修改系统模式的客户端
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
}

/**
 * @brief      参数初始化
 **/
void PX4Tracker::Initialize()
{
    // 读取offboard模式下飞机的搜索高度
    nh_.param<float>("search_alt_", search_alt_, 5);

    // 期望的飞机相对降落板的位置
    float desire_pose_x, desire_pose_y, desire_pose_z;
    nh_.param<float>("desire_pose_x", desire_pose_x, 0);
    nh_.param<float>("desire_pose_y", desire_pose_y, 0);
    nh_.param<float>("desire_pose_z", desire_pose_z, 0);
    nh_.param<float>("desire_yaw_", desire_yaw_, 0);
    desire_pose_[0] = desire_pose_x;
    desire_pose_[1] = desire_pose_y;
    desire_pose_[2] = desire_pose_z;

    // 无人机降落时的PID参数
    nh_.param<float>("p_PidXY_p", p_PidXY.p, 0.4);
    nh_.param<float>("p_PidXY_i", p_PidXY.i, 0.01);
    nh_.param<float>("p_PidXY_d", p_PidXY.d, 0.05);

    nh_.param<float>("p_PidZ_p", p_PidZ.p, 0.1);
    nh_.param<float>("p_PidZ_i", p_PidZ.i, 0);
    nh_.param<float>("p_PidZ_d", p_PidZ.d, 0);

    nh_.param<float>("p_PidYaw_p", p_PidYaw.p, 0.2);
    nh_.param<float>("p_PidYaw_i", p_PidYaw.i, 0);
    nh_.param<float>("p_PidYaw_d", p_PidYaw.d, 0);

    detect_land_state = false;
    desire_vel_[0] = 0;
    desire_vel_[1] = 0;
    desire_vel_[2] = 0;
    desire_vel_[3] = 0;
    desire_xyzVel_[0] = 0;
    desire_xyzVel_[1] = 0;
    desire_xyzVel_[2] = 0;
    desire_yawVel_ = 0;

    p_PidItemX.tempDiffer = 0;
    p_PidItemY.tempDiffer = 0;
    p_PidItemZ.tempDiffer = 0;
    p_PidItemYaw.tempDiffer = 0;
    p_PidItemX.intergral = 0;
    p_PidItemY.intergral = 0;
    p_PidItemZ.intergral = 0;
    p_PidItemYaw.intergral = 0;
}

/**
 * @brief      PID 控制程序
 * @param[in]  &currentPos 当前飞机相对降落板的位置,currentYaw 当前飞机相对降落板的方向
 * @param[in]  &expectPos 期望位置，expectYaw 飞机相对降落板的期望方向:默认0
 * @param[out] 机体系下x,y,z的期望速度,以及yaw方向的期望速度。
 **/
Eigen::Vector4d PX4Tracker::TrackerPidProcess(Eigen::Vector3d &currentPos, float currentYaw, Eigen::Vector3d &expectPos, float expectYaw)
{
    Eigen::Vector4d s_PidOut;

    /* X方向的pid控制 */
    p_PidItemX.difference = expectPos[0] - currentPos[0];
    p_PidItemX.intergral += p_PidItemX.difference;

    if (p_PidItemX.intergral >= 100)
        p_PidItemX.intergral = 100;
    else if (p_PidItemX.intergral <= -100)
        p_PidItemX.intergral = -100;

    p_PidItemX.differential = p_PidItemX.difference - p_PidItemX.tempDiffer;
    p_PidItemX.tempDiffer = p_PidItemX.difference;

    s_PidOut[0] = p_PidXY.p * p_PidItemX.difference + p_PidXY.d * p_PidItemX.differential + p_PidXY.i * p_PidItemX.intergral;

    /* Y方向的pid控制 */
    p_PidItemY.difference = expectPos[1] - currentPos[1];
    p_PidItemY.intergral += p_PidItemY.difference;

    if (p_PidItemY.intergral >= 100)
        p_PidItemY.intergral = 100;
    else if (p_PidItemY.intergral <= -100)
        p_PidItemY.intergral = -100;

    p_PidItemY.differential = p_PidItemY.difference - p_PidItemY.tempDiffer;
    p_PidItemY.tempDiffer = p_PidItemY.difference;

    s_PidOut[1] = p_PidXY.p * p_PidItemY.difference + p_PidXY.d * p_PidItemY.differential + p_PidXY.i * p_PidItemY.intergral;

    /* Z方向的pid控制 */
    p_PidItemZ.difference = expectPos[2] - currentPos[2];
    p_PidItemZ.intergral += p_PidItemZ.difference;

    if (p_PidItemZ.intergral >= 100)
        p_PidItemZ.intergral = 100;
    else if (p_PidItemZ.intergral <= -100)
        p_PidItemZ.intergral = -100;

    p_PidItemZ.differential = p_PidItemZ.difference - p_PidItemZ.tempDiffer;
    p_PidItemZ.tempDiffer = p_PidItemZ.difference;

    s_PidOut[2] = p_PidZ.p * p_PidItemZ.difference + p_PidZ.d * p_PidItemZ.differential + p_PidZ.i * p_PidItemZ.intergral;

    /* Yaw方向的pid控制 */
    p_PidItemYaw.difference = expectYaw - currentYaw;
    p_PidItemYaw.intergral += p_PidItemYaw.difference;

    if (p_PidItemYaw.intergral >= 100)
        p_PidItemYaw.intergral = 100;
    else if (p_PidItemYaw.intergral <= -100)
        p_PidItemYaw.intergral = -100;

    p_PidItemYaw.differential = p_PidItemYaw.difference - p_PidItemYaw.tempDiffer;
    p_PidItemYaw.tempDiffer = p_PidItemYaw.difference;

    s_PidOut[3] = p_PidYaw.p * p_PidItemYaw.difference + p_PidYaw.d * p_PidItemYaw.differential + p_PidYaw.i * p_PidItemYaw.intergral;

    return s_PidOut;
}

/**
 * @brief   10Hz状态机更新函数
 **/
void PX4Tracker::CmdLoopCallback(const ros::TimerEvent &event)
{
    TrackerStateUpdate();
}

/* 状态机更新函数 */
void PX4Tracker::TrackerStateUpdate()
{
    switch (FlyState)
    {
    case WAITING:
        if (px4_state_.mode != "OFFBOARD") // 等待offboard模式
        {
            temp_pos_drone[0] = px4_pose_[0];
            temp_pos_drone[1] = px4_pose_[1];
            temp_pos_drone[2] = px4_pose_[2];
            px4cmd_.send_pos_setpoint(temp_pos_drone, 0); // 在进入OFFBOARD模式之前，必须已经开始流式传输设定点。否则模式开关将被拒绝。
        }
        else
        {
            FlyState = PREPARING;
            cout << "PREPARING" << endl;
        }
        break;
    case PREPARING: // 起飞到指定高度
        posxyz_target[0] = temp_pos_drone[0];
        posxyz_target[1] = temp_pos_drone[1];
        posxyz_target[2] = search_alt_;

        if ((px4_pose_[2] <= search_alt_ + 0.2) && (px4_pose_[2] >= search_alt_ - 0.2))
        {
            FlyState = SEARCHING;
            cout << "SEARCHING" << endl;
        }
        px4cmd_.send_pos_setpoint(posxyz_target, 0);

        break;
    case SEARCHING:
        if (detect_land_state == true)
        {
            FlyState = TRACKING;
            cout << "TRACKING" << endl;
        }
        else // 如果没有检测到二维码则升高一段距离
        {
            posxyz_target[0] = px4_pose_[0];
            posxyz_target[1] = px4_pose_[1];
            posxyz_target[2] = (px4_pose_[2] + 0.1) < 10 ? (px4_pose_[2] + 0.1) : 10;
            px4cmd_.send_pos_setpoint(posxyz_target, 0);
            cout << "SEARCHING Target" << endl;
        }

        break;
    case TRACKING:
        if (detect_land_state == true)
        {
            if (abs(apriltag_pose_[0]) < 0.5 && abs(apriltag_pose_[1]) < 0.5)
            {
                FlyState = LANDING;
                cout << "LANDING" << endl;
            }
            else
            {
                desire_vel_ = TrackerPidProcess(apriltag_pose_, apriltag_yaw_, desire_pose_, desire_yaw_);

                desire_xyzVel_[0] = desire_vel_[1];
                desire_xyzVel_[1] = desire_vel_[0];
                desire_xyzVel_[2] = 0;
                desire_yawVel_ = desire_vel_[3];

                px4cmd_.send_body_velxyz_setpoint(desire_xyzVel_, desire_yawVel_);
            }
        }
        else
        {
            FlyState = SEARCHING;
            cout << "SEARCHING" << endl;
        }

        break;
    case LANDING:
        if (detect_land_state == true)
        {
            if (abs(apriltag_pose_[0]) < 0.5 && abs(apriltag_pose_[1]) < 0.5)
            {
                if (apriltag_pose_[2] > 0.2)
                {
                    desire_vel_ = TrackerPidProcess(apriltag_pose_, apriltag_yaw_, desire_pose_, desire_yaw_);

                    desire_xyzVel_[0] = desire_vel_[1];
                    desire_xyzVel_[1] = desire_vel_[0];
                    desire_xyzVel_[2] = desire_vel_[2];
                    desire_yawVel_ = desire_vel_[3];

                    px4cmd_.send_body_velxyz_setpoint(desire_xyzVel_, desire_yawVel_);
                    cout << "当前高度:" << apriltag_pose_[2] << endl;

                    // 如果在准备中途中切换到onboard，则保持当前位置
                    if (px4_state_.mode != "OFFBOARD")
                    {
                        cout << "离线信号丢失" << endl;
                        temp_pos_drone[0] = px4_pose_[0];
                        temp_pos_drone[1] = px4_pose_[1];
                        temp_pos_drone[2] = px4_pose_[2];
                        px4cmd_.send_pos_setpoint(temp_pos_drone, 0); // 在进入OFFBOARD模式之前，必须已经开始流式传输设定点，否则模式开关将被拒绝。
                    }
                }
                else
                {
                    mode_cmd_.request.custom_mode = "AUTO.LAND";
                    set_mode_client_.call(mode_cmd_);
                    FlyState = LANDOVER;
                    cout << "LANDOVER" << endl;
                }
            }
        }
        else
        {
            FlyState = SEARCHING;
            cout << "SEARCHING" << endl;
        }

        break;
    case LANDOVER:
        arm_cmd_.request.value = false;
        arming_client_.call(arm_cmd_);

        break;
    default:
        cout << "error" << endl;
    }
}

/**
 * @brief   接收 apriltag_ros 降落板相对无人机的位置以及偏航角
 **/
void PX4Tracker::AprilPoseCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    detect_land_state = false;
    double temp_roll, temp_pitch, temp_yaw;
    tf2::Quaternion quat;

    for (auto &item : msg->detections)
    {
        // 如果标签的ID与预期的ID匹配
        if (item.pose.pose.pose.position.z > 0.1)
        {
            detect_land_state = true;
            // 获取标签在相机坐标系中的位置信息
            apriltag_pose_[0] = item.pose.pose.pose.position.x;
            apriltag_pose_[1] = item.pose.pose.pose.position.y;
            apriltag_pose_[2] = item.pose.pose.pose.position.z;
            // 将ROS消息中的四元数表示转换为TF2库中的tf2::Quaternion对象
            tf2::fromMsg(item.pose.pose.pose.orientation, quat);
            // 获取标签在相机坐标系中的姿态信息（四元数），并将其转换为欧拉角
            tf2::Matrix3x3(quat).getRPY(temp_roll, temp_pitch, temp_yaw);
            // 更新标签的yaw角度
            apriltag_yaw_ = temp_yaw;
        }
    }
}

/**
 * @brief   读取无人机在local坐标系中的位置，东北天，坐标原点在PX4上电的地方
 **/
void PX4Tracker::Px4PosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    Eigen::Vector3d pos_drone_fcu_enu(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    px4_pose_ = pos_drone_fcu_enu;
}

/**
 * @brief   接收来自飞控的当前飞机状态
 **/
void PX4Tracker::Px4StateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    px4_state_ = *msg;
}

/**
 * @brief   主函数
 **/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_landing");
    ros::NodeHandle nh("~"); // 私有的NodeHandle对象

    // 隐式调用构造函数初始化对象
    PX4Tracker px4tracker(nh);

    ros::spin();
    return 0;
}
