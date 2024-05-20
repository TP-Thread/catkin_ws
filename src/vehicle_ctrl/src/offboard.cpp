#include <ros/ros.h>
// CommandBool服务的头文件，该服务的类型为mavros_msgs：：CommandBool，用来进行无人机解锁
#include <mavros_msgs/CommandBool.h>
// SetMode服务的头文件，该服务的类型为mavros_msgs：：SetMode，用来设置无人机的飞行模式，切换offboard
#include <mavros_msgs/SetMode.h>
// 订阅的消息体的头文件，该消息体的类型为mavros_msgs：：State，查看无人机的状态
#include <mavros_msgs/State.h>
// 发布的位置消息体对应的头文件，该消息体的类型为geometry_msgs：：PoseStamped，用来进行发送目标位置
#include <geometry_msgs/PoseStamped.h>

// 建立一个订阅消息体类型的变量，用于存储订阅的信息
mavros_msgs::State current_state;

// 订阅时的回调函数，接收到消息时赋值给 current_state
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // 订阅 mavros/state 话题，<>里面为模板参数，传入的是订阅的消息体类型，（）里面传入三个参数，分别是订阅的话题名、缓存大小、回调函数
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

    // 发布话题之前需要公告，并获取句柄
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    // 启动服务1，设置客户端（Client）名称为arming_client，启动服务用的函数为nh下的serviceClient<>()函数，<>里面是该服务的类型，（）里面是该服务的路径
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

    // 启动服务2，设置客户端（Client）名称为set_mode_client
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // 官方要求local_pos_pub发布速率必须快于2Hz，这里设置为20Hz
    // PX4在两个Offboard命令之间有一个500ms的延时，如果超过此延时，系统会将回到无人机进入Offboard模式之前的最后一个模式
    ros::Rate rate(20);

    // 等待飞控连接 mavros，current_state 是订阅的mavros的状态，连接成功后跳出循环
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("mavros connected");

    // 先实例化一个geometry_msgs::PoseStamped类型的对象，并对其赋值，最后将其发布出去
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 4;
    pose.pose.position.z = 2;

    // 在进入Offboard模式之前，必须已经启动了local_pos_pub数据流，否则模式切换将被拒绝
    // 这里的100可以被设置为任意数
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // 建立一个类型为CommandBool的服务端arm_cmd，并将其中的是否解锁设为"true"
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // 建立一个类型为SetMode的服务端offb_set_mode，并将其中的模式mode设为"OFFBOARD"
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // 更新时间
    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {
        // 判断是否解锁，如果没有解锁，则客户端arming_client向服务端arm_cmd发起请求call，这里是5秒钟进行一次判断，避免飞控被大量的请求阻塞
        if(!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if(arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed"); 
            }
            last_request = ros::Time::now();
        }
        else    // 已经解锁后，判断当前模式是否为 OFFBOARD 模式，如果不是，则客户端set_mode_client向服务端offb_set_mode发起请求call
        {
            if(current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("OFFBOARD enabled"); 
                }
                last_request = ros::Time::now();
            }
        }

        // 发布位置信息，飞机解锁后打开offboard模式才能飞起来
        local_pos_pub.publish(pose);

        // 在spinOnce函数中调用回调函数
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}