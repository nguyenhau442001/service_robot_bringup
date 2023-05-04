#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>

class OdometryConverter
{
public:
    OdometryConverter()
    {
        // Initialize ROS node
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");

        // Subscribe to Odom topic
        sub_ = nh.subscribe("odom", 10, &OdometryConverter::odom_callback, this);

        // Publishers for roll, pitch, and yaw angles
        roll_pub_ = nh.advertise<std_msgs::Float64>("/converted_odom/roll", 10);
        pitch_pub_ = nh.advertise<std_msgs::Float64>("/converted_odom/pitch", 10);
        yaw_pub_ = nh.advertise<std_msgs::Float64>("/converted_odom/yaw", 10);

        // Initialize tf listener
        listener_.reset(new tf::TransformListener);
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // Extract the Quaternion from the message
        tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

        // Convert Quaternion to roll-pitch-yaw angles
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // Convert roll-pitch-yaw angles to degrees
        roll = roll * 180.0 / M_PI;
        pitch = pitch * 180.0 / M_PI;
        yaw = yaw * 180.0 / M_PI;

        // Publish roll, pitch, and yaw angles
        std_msgs::Float64 roll_msg, pitch_msg, yaw_msg;
        roll_msg.data = roll;
        pitch_msg.data = pitch;
        yaw_msg.data = yaw;
        roll_pub_.publish(roll_msg);
        pitch_pub_.publish(pitch_msg);
        yaw_pub_.publish(yaw_msg);
    }

    void run()
    {
        // Spin ROS node
        ros::spin();
    }

private:
    ros::Subscriber sub_;
    ros::Publisher roll_pub_;
    ros::Publisher pitch_pub_;
    ros::Publisher yaw_pub_;
    boost::shared_ptr<tf::TransformListener> listener_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_converter");
    OdometryConverter odom_converter;
    odom_converter.run();
    return 0;
}

