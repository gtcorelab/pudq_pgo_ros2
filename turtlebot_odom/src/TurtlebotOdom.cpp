#include "TurtlebotOdom.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

TurtlebotOdom::TurtlebotOdom() : Node("turtlebot_odom_node") {
    RCLCPP_INFO(this->get_logger(), "Initializing Turtlebot Odometry Node");

     //Get my namespace (remove the slash with substr)
    tb_name_ = std::string(this->get_namespace()).substr(1);

    pose_frame_id_ = std::string("world");
    pose_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    pose_tf_.header.frame_id = pose_frame_id_;
    pose_tf_.child_frame_id = std::string(tb_name_).append("_pose");

    // imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", 10, std::bind(&TurtlebotOdom::imu_callback, this, _1));
    
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&TurtlebotOdom::odom_callback, this, _1));
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
}

void TurtlebotOdom::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
    RCLCPP_INFO(this->get_logger(), "got imu");

    (void) imu_msg;

    //Todo: Implement UDQ IMU integration
}

void TurtlebotOdom::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {

    auto now = this->get_clock()->now();

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now;
    pose.header.frame_id = pose_frame_id_;
    pose.pose = odom_msg->pose.pose;

    pose_publisher_->publish(pose);

    //Broadcast tf
    pose_tf_.header.stamp = now;
    pose_tf_.transform.translation.x = pose.pose.position.x;
    pose_tf_.transform.translation.y = pose.pose.position.y;
    pose_tf_.transform.translation.z = pose.pose.position.z;
    pose_tf_.transform.rotation = pose.pose.orientation;
    pose_tf_broadcaster_->sendTransform(pose_tf_);
}

