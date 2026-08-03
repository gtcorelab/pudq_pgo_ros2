#include "PUDQKeyframeGenerator.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

using namespace pudq_lib;


PUDQKeyframeGenerator::PUDQKeyframeGenerator() : Node("pudq_keyframe_generator_node") {
	RCLCPP_INFO(this->get_logger(), "Initializing PUDQ Keyframe Generator Node");

    robot_name_ = std::string(this->get_namespace()).substr(1);

    //Declare all parameters
    this->declare_parameter("fixed_frame", "world");
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("t_threshold", 0.1);
    this->declare_parameter("theta_threshold", 0.2);
    this->declare_parameter("sigma_t", 0.02);
    this->declare_parameter("sigma_theta", 0.01);

    //Get parameters
    std::string map_frame_noprefix;
    this->get_parameter("fixed_frame", fixed_frame_id_);
    this->get_parameter("map_frame", map_frame_noprefix);
    map_frame_id_ = std::string(robot_name_).append("_").append(map_frame_noprefix);

    this->get_parameter("t_threshold", t_threshold_);
    this->get_parameter("theta_threshold", theta_threshold_);
    this->get_parameter("sigma_t", sigma_t_);
    this->get_parameter("sigma_theta", sigma_theta_);

    normrnd_t_ = std::normal_distribution<double>(0.0, sigma_t_);
    normrnd_theta_ = std::normal_distribution<double>(0.0, sigma_theta_);

    num_edges_ = 0;
    initialized_ = false;

    map_pudq_ = pudq_identity();
    map_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

	pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("pose", 10, std::bind(&PUDQKeyframeGenerator::pose_callback, this, _1));

    // edge_publisher_ = nh_.advertise<relative_nav::Edge>("edge", 10);
    vertex_true_publisher_ = this->create_publisher<pudq_msgs::msg::PUDQVertex>("pudq_vertex_true", 10);
    pudq_edge_publisher_ = this->create_publisher<pudq_msgs::msg::PUDQEdge>("pudq_edge", 10);

    timer_ = this->create_wall_timer(1s, std::bind(&PUDQKeyframeGenerator::timer_callback, this));
}

void PUDQKeyframeGenerator::generate_keyframe(Eigen::Vector3d map_pose) {
    unsigned int edge_i = num_edges_;
    unsigned int edge_j = num_edges_ + 1;

    //Convert measurement to PUDQ
    Eigen::Vector4d x_j = pose_to_pudq(map_pose);

    //Compute true delta-pose in PUDQ space
    Eigen::Vector4d x_i = pose_to_pudq(map_pose_prev_);
    Eigen::Vector4d z_ij_true = pudq_compose(pudq_inv(x_i), x_j);

    //Compute noisy delta-pose
    //Todo: Randomize covariance matrix (currently isotropic distribution)
    Eigen::Vector3d eta_ij;
    eta_ij << normrnd_theta_(gen_), normrnd_t_(gen_), normrnd_t_(gen_);

    //Todo: Change to correlated noise
    Eigen::Matrix3d pudq_cov = Eigen::Matrix3d::Zero();
    pudq_cov(0,0) = sigma_theta_*sigma_theta_;
    pudq_cov(1,1) = sigma_t_*sigma_t_;
    pudq_cov(2,2) = sigma_t_*sigma_t_;

    // Eigen::Vector4d z_ij_tilde = pose_to_pudq(pudq_to_pose(z_ij_true) + eta_ij);

    Eigen::Vector4d z_ij_pudq = pudq_compose(z_ij_true, pudq_lib::Lie_Exp_1(eta_ij));
    Eigen::Vector3d z_ij_eucl = pudq_to_pose(z_ij_pudq);

    // ROS_WARN_STREAM("x_i: [" << map_pose_prev_(0) << ", "<< map_pose_prev_(1) << ", "<< map_pose_prev_(2) << "]");
    // ROS_WARN_STREAM("x_j: [" << pose(0) << ", "<< pose(1) << ", "<< pose(2) << "]");
    // ROS_WARN_STREAM("delta_pose: [" << delta_pose(0) << ", "<< delta_pose(1) << ", "<< delta_pose(2) << "]");
    // ROS_WARN_STREAM("Computed delta-pose: (" << delta_pose(0) << ", " << delta_pose(1) << ", " << delta_pose(2) << std::endl);

    //BAD
    // std::vector<double> z_ij_pudq_msg{z_ij_pudq(0), z_ij_pudq(1), z_ij_pudq(2), z_ij_pudq(3)};
    // std::vector<double> cov_pudq_msg{pudq_cov(0,0), 0.0, 0.0, 0.0, pudq_cov(1,1), 0.0, 0.0, 0.0, pudq_cov(2,2)};

    //GOOD
    std::array<double, 4> z_ij_pudq_msg{z_ij_pudq(0), z_ij_pudq(1), z_ij_pudq(2), z_ij_pudq(3)};
    std::array<double, 9> cov_pudq_msg{pudq_cov(0,0), 0.0, 0.0, 0.0, pudq_cov(1,1), 0.0, 0.0, 0.0, pudq_cov(2,2)};

    //Compute 3d Euclidean covariance
    Eigen::Matrix3d eucl_cov = pudq_cov_to_eucl(pudq_cov, eta_ij(0));
    Eigen::MatrixXd eucl_cov_3d = eucl_cov_2d_to_3d(eucl_cov);

    //Convert 6x6 euclidean Covariance matrix to 36x1 boost array for relative_nav Edge message
    std::vector<double> cov_eucl_msg(36);
    int k=0;
    for (int i=0; i<6; i++) {
        for (int j=0; j<6; j++) {
            cov_eucl_msg[k] = eucl_cov_3d(i,j);
        }
    }

    //Publish PUDQ edge message
    rclcpp::Time now = this->get_clock()->now();
    pudq_msgs::msg::PUDQEdge pudq_edge_msg;
    pudq_edge_msg.header.stamp = now;
    pudq_edge_msg.edge_i = edge_i;
    pudq_edge_msg.edge_j = edge_j;
    pudq_edge_msg.delta_pose = z_ij_pudq_msg;
    pudq_edge_msg.covariance = cov_pudq_msg;
    pudq_edge_publisher_->publish(pudq_edge_msg);

    //Dont uncomment
    //Publish Euclidean edge message
    // relative_nav::Edge edge_msg;
    // edge_msg.header.stamp = now;
    // edge_msg.from_node_id = edge_i;
    // edge_msg.to_node_id = edge_j;
    // edge_msg.transform.translation.x = z_ij_eucl(0);
    // edge_msg.transform.translation.y = z_ij_eucl(1);
    // edge_msg.transform.translation.z = 0.0;
    // edge_msg.transform.rotation = yaw_to_quat(z_ij_eucl(2));
    // edge_msg.covariance = cov_eucl_msg;
    // edge_publisher_.publish(edge_msg);

    //Publish identity vertex if this is the first edge to be published
    if (num_edges_ == 0) {
        std::array<double, 4> pudq_kf_init{1.0, 0.0, 0.0, 0.0};
        pudq_msgs::msg::PUDQVertex pudq_vertex_msg;
        pudq_vertex_msg.header.stamp = now;
        pudq_vertex_msg.id = 0;
        pudq_vertex_msg.pose = pudq_kf_init;
        vertex_true_publisher_->publish(pudq_vertex_msg);
    }

    //Publish the keyframe pose (vertex) x_j in the map frame
    pudq_msgs::msg::PUDQVertex pudq_vertex_msg;
    pudq_vertex_msg.header.stamp = now;
    pudq_vertex_msg.id = edge_j;
    std::array<double, 4> pudq_vertex{x_j(0), x_j(1), x_j(2), x_j(3)};
    pudq_vertex_msg.pose = pudq_vertex;
    vertex_true_publisher_->publish(pudq_vertex_msg);

    //Increment edge count
    num_edges_++;
    map_pose_prev_ = map_pose;

    //Dont uncomment
    //Add delta-pose to graph
    // pudq_msgs::AddPlanarDeltaPose srv;
    // srv.request.delta_pose = z_ij_srv;
    // srv.request.x_j_true = x_j_true_srv;
    // srv.request.information = information_srv;
    // if (add_dp_client_.call(srv)) {
    //     if (srv.response.success) {
    //         //Store current pose to previous
    //         pose_prev_ = pose;
    //     } else {
    //         ROS_ERROR("Service add_delta_pose called, but failed to update the graph.");
    //     }
    // } else {
    //     ROS_ERROR("Failed to call service add_delta_pose");
    // }
}

void PUDQKeyframeGenerator::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg) {
    double yaw = quat_to_yaw(pose_msg->pose.orientation);

    //Extract 2D translation and rotation
    Eigen::Vector3d pose;
    pose << pose_msg->pose.position.x, pose_msg->pose.position.y, yaw;

    //Initialize previous pose if not done already
    if (!initialized_) {

        //Initialize map transform PUDQ and TF
        map_pudq_ = pose_to_pudq(pose);

        //Initialize pose in map frame
        map_pose_prev_ = pudq_to_pose(pudq_identity());

        //Initialize map frame
        map_tf_.header.frame_id = fixed_frame_id_;
        map_tf_.child_frame_id = map_frame_id_;
        map_tf_.transform.translation.x = pose_msg->pose.position.x;
        map_tf_.transform.translation.y = pose_msg->pose.position.y;
        map_tf_.transform.translation.z = 0.0;
        map_tf_.transform.rotation = yaw_to_quat(yaw);

        RCLCPP_WARN(this->get_logger(), "Map frame initialized.");

        initialized_ = true;
        return;
    }

    //Convert the global pose into the local map frame pose
    Eigen::Vector3d map_pose = pudq_to_pose(pudq_compose(pudq_inv(map_pudq_), pose_to_pudq(pose)));

    double dt = (map_pose.segment(0,2) - map_pose_prev_.segment(0,2)).norm();
    
    // double dtheta = dist_theta(pose(2), map_pose_prev_(2));

    //If delta from previous keyframe is big enough, generate a new delta-pose "measurement"
    if (dt >= t_threshold_) {
        //Generate a keyframe for this pose (expressed in the local map frame)
        generate_keyframe(map_pose);
    }
}

void PUDQKeyframeGenerator::timer_callback() {
    if (initialized_) {
        map_tf_.header.stamp = this->get_clock()->now();
        map_tf_broadcaster_->sendTransform(map_tf_);
    }
}
