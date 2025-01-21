#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include <cmath>

double x = 0.0;  
double y = 0.0; 
double theta = 0.0;  

ros::Publisher odom_pub;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    double dt = 0.1;
    double linear_velocity = msg->linear.x;
    double angular_velocity = msg->angular.z;

    double delta_theta = angular_velocity * dt;
    double delta_x = linear_velocity * cos(theta) * dt;
    double delta_y = linear_velocity * sin(theta) * dt;

    x += delta_x;
    y += delta_y;
    theta += delta_theta;

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "odom";

    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

    odom_msg.twist.twist.linear.x = linear_velocity;
    odom_msg.twist.twist.angular.z = angular_velocity;

    odom_pub.publish(odom_msg);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, 0.0));
    transform.setRotation(tf::Quaternion(0, 0, sin(theta / 2), cos(theta / 2)));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/cmd_vel", 10, cmdVelCallback);

    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);

    ros::spin();

    return 0;
}

