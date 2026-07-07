#ifndef TURTLEBOT_WAYPOINT_CONTROLLER_HPP
#define TURTLEBOT_WAYPOINT_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <std_srvs/srv/empty.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

class TurtlebotWaypointController : public rclcpp::Node {
private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_service_;

    std::string tb_name_;
    bool running_;
    int current_waypoint_;
    bool bypass_start_;

    std::vector<geometry_msgs::msg::Point> waypoints_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> waypoint_tf_broadcaster_;
    geometry_msgs::msg::TransformStamped waypoint_tf_;

    double xy_threshold_, yaw_threshold_;

    double quat_to_yaw(geometry_msgs::msg::Quaternion q);
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr);
    void start(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response);

public:
	TurtlebotWaypointController();
};

#endif
