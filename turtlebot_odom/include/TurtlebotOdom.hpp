#ifndef TURTLEBOT_ODOM_HPP
#define TURTLEBOT_ODOM_HPP

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class TurtlebotOdom : public rclcpp::Node {
private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;

    //Ground truth pose broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> pose_tf_broadcaster_;
    geometry_msgs::msg::TransformStamped pose_tf_;
    std::string pose_frame_id_;
    std::string tb_name_;

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
public:
    TurtlebotOdom();
};

#endif
