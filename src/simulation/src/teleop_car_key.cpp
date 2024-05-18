#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_B 0x62
#define KEYCODE_C 0x63
#define KEYCODE_D 0x64
#define KEYCODE_E 0x65
#define KEYCODE_F 0x66
#define KEYCODE_G 0x67
#define KEYCODE_Q 0x71
#define KEYCODE_R 0x72
#define KEYCODE_T 0x74
#define KEYCODE_V 0x76

class KeyboardReader
{
public:
    KeyboardReader()
        : kfd(0)
    {
        // get the console in raw mode
        tcgetattr(kfd, &cooked);
        struct termios raw;
        memcpy(&raw, &cooked, sizeof(struct termios));
        raw.c_lflag &= ~(ICANON | ECHO);
        // Setting a new line, then end of file
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
        tcsetattr(kfd, TCSANOW, &raw);
    }
    void readOne(char *c)
    {
        int rc = read(kfd, c, 1);
        if (rc < 0)
        {
            throw std::runtime_error("read failed");
        }
    }
    void shutdown()
    {
        tcsetattr(kfd, TCSANOW, &cooked);
    }

private:
    int kfd;
    struct termios cooked;
};

KeyboardReader input;

class TeleopCar
{
public:
    TeleopCar();
    void keyLoop();

private:
    ros::NodeHandle nh_;
    double linear_, angular_, l_scale_, a_scale_;
    ros::Publisher twist_pub_;
};

TeleopCar::TeleopCar() : linear_(0),
                         angular_(0),
                         l_scale_(1.0),
                         a_scale_(1.0)
{
    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("scale_linear", l_scale_, l_scale_);

    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("catvehicle/cmd_vel", 1);
}

void quit(int sig)
{
    (void)sig;
    input.shutdown();
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_car_key");
    TeleopCar teleop_turtle;

    signal(SIGINT, quit);

    teleop_turtle.keyLoop();
    quit(0);

    return (0);
}

void TeleopCar::keyLoop()
{
    char c;
    bool dirty = false;

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the turtle. 'q' to quit.");

    for (;;)
    {
        // get the next event from the keyboard
        try
        {
            input.readOne(&c);
        }
        catch (const std::runtime_error &)
        {
            perror("read():");
            return;
        }

        linear_ = angular_ = 0;
        ROS_DEBUG("value: 0x%02X\n", c);

        switch (c)
        {
        case KEYCODE_LEFT:
            ROS_DEBUG("LEFT");
            linear_ = 0.5;
            angular_ = 1.0;
            dirty = true;
            break;
        case KEYCODE_RIGHT:
            ROS_DEBUG("RIGHT");
            linear_ = 0.5;
            angular_ = -1.0;
            dirty = true;
            break;
        case KEYCODE_UP:
            ROS_DEBUG("UP");
            linear_ = 1.0;
            dirty = true;
            break;
        case KEYCODE_DOWN:
            ROS_DEBUG("DOWN");
            linear_ = -1.0;
            dirty = true;
            break;
        case KEYCODE_Q:
            ROS_DEBUG("quit");
            linear_ = angular_ = 0;
            dirty = true;
            break;
        }

        geometry_msgs::Twist twist;
        twist.angular.z = a_scale_ * angular_;
        twist.linear.x = l_scale_ * linear_;
        if (dirty == true)
        {
            twist_pub_.publish(twist);
            dirty = false;
        }
    }

    return;
}
