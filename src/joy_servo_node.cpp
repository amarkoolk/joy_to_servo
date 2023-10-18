#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/UInt16.h"
#include "geometry_msgs/Twist.h"

class JoyServo {
    private:
        ros::Publisher cmd_vel_publisher;
        ros::Subscriber joy_subscriber;
    public:
        JoyServo(ros::NodeHandle *nh) 
        {
            cmd_vel_publisher = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1000);   
            joy_subscriber = nh->subscribe("joy", 1000, &JoyServo::joy_callback, this);

        }
    void joy_callback(const sensor_msgs::Joy::ConstPtr& msg)
    {
        float left_horizontal = msg->axes[0];
        float left_vertical = msg->axes[1];
        float right_horizontal = msg->axes[2];
        float right_vertical = msg->axes[3];

        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = -right_horizontal;
        twist_msg.angular.z = -left_vertical;
        cmd_vel_publisher.publish(twist_msg);
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_servo_node");
    ros::NodeHandle n;
    JoyServo joy_servo_node = JoyServo(&n);
    ros::spin();
    return 0;
}