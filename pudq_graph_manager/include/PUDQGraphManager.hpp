#ifndef PUDQ_GRAPH_MANAGER_HPP
#define PUDQ_GRAPH_MANAGER_HPP

#include <queue>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

//TF2
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <std_msgs/msg/color_rgba.hpp>
#include <std_srvs/srv/empty.hpp>

//PUDQ libraries
#include <pudq_msgs/msg/pudq_edge.hpp>
#include <pudq_msgs/msg/pudq_vertex.hpp>

// #include <pudq_msgs/PUDQMultiEdge.h>
// #include <pudq_msgs/PUDQMultiVertex.h>
// #include <pudq_msgs/msg_PUDQVertexArray.h>

#include <pudq_lib/pudq_lib.hpp>
#include <pudq_pgo_lib/pudq_pgo_lib.hpp>
#include <pudq_pgo_lib/PUDQGraph.hpp>

class PUDQGraphManager : public rclcpp::Node {
private:
    rclcpp::Subscription<pudq_msgs::msg::PUDQVertex>::SharedPtr vertex_subscriber_;
    rclcpp::Subscription<pudq_msgs::msg::PUDQEdge>::SharedPtr edge_subscriber_;
    rclcpp::Subscription<pudq_msgs::msg::PUDQEdge>::SharedPtr lc_subscriber_;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr vertices_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr edge_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr odom_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr edge_true_publisher_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr print_graph_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr print_cost_service_;

    //TF listener
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    rclcpp::TimerBase::SharedPtr viz_timer_;

    //Parameters
    bool g2o_mode_;
    std::string g2o_file_;

    std::string robot_name_, map_frame_id_, fixed_frame_id_;
    double sigma_t_, sigma_theta_;

    bool est_marker_init_, odom_marker_init_, true_marker_init_, multi_lc_viz_init_;

    // bool map_frame_init_;
    Eigen::Vector4d map_pudq_;

    std_msgs::msg::ColorRGBA edge_color_, lc_color_, multi_lc_color_, true_color_, odom_color_;

    visualization_msgs::msg::Marker edges_marker_, multi_lc_marker_, edges_true_marker_, odom_marker_;
    visualization_msgs::msg::MarkerArray legend_markers_;

    //Graph storage object
    PUDQGraph G;

    //Odom graph object
    std::vector<Eigen::Vector3d> odom_vertices_;
    std::queue<pudq_msgs::msg::PUDQEdge::SharedPtr> lc_edge_msg_queue;

    bool saved_g2o_;

    // std::map<int, std::string> neighbor_list_;
    // std::vector<unsigned int> public_vertices_;

    void initialize_graph();

    int read_g2o_file();
    int write_g2o_file(std::string filename);

    void initialize_viz();
    void update_estimate_viz();
    // void update_multi_lc_viz();
    void update_odom_viz();
    void update_truth_viz();

    // void publish_public_poses();

    void vertex_callback(const pudq_msgs::msg::PUDQVertex::SharedPtr vertex_msg);
    void edge_callback(const pudq_msgs::msg::PUDQEdge::SharedPtr edge_msg);
    void lc_edge_callback(const pudq_msgs::msg::PUDQEdge::SharedPtr lc_edge_msg);

    void process_lc_edge(const pudq_msgs::msg::PUDQEdge::SharedPtr lc_edge_msg);
    // void multi_lc_edge_callback(const pudq_msgs::PUDQMultiEdgeConstPtr &multi_lc_edge_msg);
    // void public_vertex_callback(const pudq_msgs::PUDQVertexArrayConstPtr &vertex_array_msg, int robot_id);

    void print_graph(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response);
    void print_cost(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response);

    void viz_timer_callback();
    
public:
    PUDQGraphManager();
};

#endif