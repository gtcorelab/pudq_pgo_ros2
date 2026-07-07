#ifndef PUDQ_KEYFRAME_GENERATOR_HPP
#define PUDQ_KEYFRAME_GENERATOR_HPP

#include <array>
#include <random>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

//TF2
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

//Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

//REEF
// #include <relative_nav/Edge.h>
#include <pudq_msgs/msg/pudq_edge.hpp>
#include <pudq_msgs/msg/pudq_vertex.hpp>
#include <pudq_lib/pudq_lib.hpp>

class PUDQKeyframeGenerator : public rclcpp::Node {
private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;

    // ros::Publisher edge_publisher_, pudq_edge_publisher_, vertex_true_publisher_;
    rclcpp::Publisher<pudq_msgs::msg::PUDQVertex>::SharedPtr vertex_true_publisher_;
    rclcpp::Publisher<pudq_msgs::msg::PUDQEdge>::SharedPtr pudq_edge_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    int num_edges_;
    double t_threshold_, theta_threshold_;
    double sigma_t_, sigma_theta_;

    std::string robot_name_;
    std::string fixed_frame_id_, map_frame_id_;

    std::default_random_engine gen_;
    std::normal_distribution<double> normrnd_t_, normrnd_theta_;

    Eigen::Vector3d map_pose_prev_;

    Eigen::Vector4d map_pudq_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> map_tf_broadcaster_;
    geometry_msgs::msg::TransformStamped map_tf_;

    bool initialized_;

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg);
    void generate_keyframe(Eigen::Vector3d pose);
    void timer_callback();
public:
	PUDQKeyframeGenerator();
};

#endif
