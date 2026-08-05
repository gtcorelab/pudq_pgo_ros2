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

    //Read lc detection parameters
    this->declare_parameter("intra_lc_range", 1.0);
    this->declare_parameter("intra_lc_fov", M_PI);
    this->declare_parameter("intra_lc_prob", 0.05);

    //Read lc noise parameters
    this->declare_parameter("intra_sigma_t", 0.01);
    this->declare_parameter("intra_sigma_theta", 0.01);

    //Get parameters
    std::string map_frame_noprefix;
    this->get_parameter("fixed_frame", fixed_frame_id_);
    this->get_parameter("map_frame", map_frame_noprefix);
    map_frame_id_ = std::string(robot_name_).append("_").append(map_frame_noprefix);

    this->get_parameter("t_threshold", t_threshold_);
    this->get_parameter("theta_threshold", theta_threshold_);
    this->get_parameter("sigma_t", sigma_t_);
    this->get_parameter("sigma_theta", sigma_theta_);

    //Read lc detection parameters
    this->get_parameter("intra_lc_range", intra_lc_range_);
    this->get_parameter("intra_lc_fov", intra_lc_fov_);
    this->get_parameter("intra_lc_prob", intra_lc_prob_);

    //Read lc noise parameters
    this->get_parameter("intra_sigma_t", intra_sigma_t_);
    this->get_parameter("intra_sigma_theta", intra_sigma_theta_);

    normrnd_t_ = std::normal_distribution<double>(0.0, sigma_t_);
    normrnd_theta_ = std::normal_distribution<double>(0.0, sigma_theta_);

    num_odom_edges_ = 0;
    num_vertices_ = 0;

    initialized_ = false;

    map_pudq_ = pudq_identity();
    map_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

	pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("pose", 10, std::bind(&PUDQKeyframeGenerator::pose_callback, this, _1));

    vertex_true_publisher_ = this->create_publisher<pudq_msgs::msg::PUDQVertex>("pudq_vertex_true", 10);
    pudq_edge_publisher_ = this->create_publisher<pudq_msgs::msg::PUDQEdge>("pudq_edge", 10);
    pudq_lc_publisher_ = this->create_publisher<pudq_msgs::msg::PUDQEdge>("pudq_loop_closure", 10);

    timer_ = this->create_wall_timer(1s, std::bind(&PUDQKeyframeGenerator::timer_callback, this));
}

void PUDQKeyframeGenerator::generate_keyframe(Eigen::Vector3d map_pose) {
    size_t edge_i = num_odom_edges_;
    size_t edge_j = num_odom_edges_ + 1;

    //Convert measurement to PUDQ
    Eigen::Vector4d x_j = pose_to_pudq(map_pose);

    //Compute true delta-pose in PUDQ space
    Eigen::Vector4d x_i = pose_to_pudq(map_pose_prev_);
    Eigen::Vector4d z_ij_true = pudq_compose(pudq_inv(x_i), x_j);

    // Compute noisy delta-pose
    // Todo: Randomize covariance matrix (currently isotropic distribution)
    Eigen::Vector3d eta_ij;
    eta_ij << normrnd_theta_(gen_), normrnd_t_(gen_), normrnd_t_(gen_);
    Eigen::Vector4d z_ij_pudq = pudq_compose(z_ij_true, pudq_lib::Lie_Exp_1(eta_ij));

    //Todo: Change to correlated noise
    Eigen::Matrix3d pudq_cov = Eigen::Matrix3d::Zero();
    pudq_cov(0,0) = sigma_theta_*sigma_theta_;
    pudq_cov(1,1) = sigma_t_*sigma_t_;
    pudq_cov(2,2) = sigma_t_*sigma_t_;

    // Eigen::Vector3d z_ij_eucl = pudq_to_pose(z_ij_pudq);
    // ROS_WARN_STREAM("x_i: [" << map_pose_prev_(0) << ", "<< map_pose_prev_(1) << ", "<< map_pose_prev_(2) << "]");
    // ROS_WARN_STREAM("x_j: [" << pose(0) << ", "<< pose(1) << ", "<< pose(2) << "]");
    // ROS_WARN_STREAM("delta_pose: [" << delta_pose(0) << ", "<< delta_pose(1) << ", "<< delta_pose(2) << "]");
    // ROS_WARN_STREAM("Computed delta-pose: (" << delta_pose(0) << ", " << delta_pose(1) << ", " << delta_pose(2) << std::endl);

    std::array<double, 4> z_ij_pudq_msg{z_ij_pudq(0), z_ij_pudq(1), z_ij_pudq(2), z_ij_pudq(3)};
    std::array<double, 9> cov_pudq_msg{pudq_cov(0,0), 0.0, 0.0, 0.0, pudq_cov(1,1), 0.0, 0.0, 0.0, pudq_cov(2,2)};

    //Publish PUDQ edge message
    rclcpp::Time now = this->get_clock()->now();
    pudq_msgs::msg::PUDQEdge pudq_edge_msg;
    pudq_edge_msg.header.stamp = now;
    pudq_edge_msg.edge_i = edge_i;
    pudq_edge_msg.edge_j = edge_j;
    pudq_edge_msg.delta_pose = z_ij_pudq_msg;
    pudq_edge_msg.covariance = cov_pudq_msg;
    pudq_edge_publisher_->publish(pudq_edge_msg);

    //Publish identity vertex if this is the first edge to be published
    if (num_odom_edges_ == 0) {
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

    // Increment edge count
    num_odom_edges_++;
    map_pose_prev_ = map_pose;

    // Update true vertices and check for loop closures as well
    vertices_true_pudq_.push_back(x_j);
    vertices_true_eucl_.push_back(pudq_to_pose(x_j));

    num_vertices_++;
}

void PUDQKeyframeGenerator::detect_loop_closure() {

    // Get the last true vertex and its index
    size_t lc_i = num_vertices_-1;
    Eigen::Vector3d pose_i = vertices_true_eucl_[lc_i];

    //Save position vector from pose j
    Eigen::Vector2d v_i = pose_i.segment(0,2);
 
    //Construct unit vector to represent line-of-sight from vertex j
    Eigen::Vector2d u_i;
    u_i << 1.0, 0.0;
    u_i = pudq_lib::R_theta(pose_i(2))*u_i;

    //Search graph (more than 2 indices away) for virtual loop closusres with vertex x_j
    for (size_t lc_j = 0; lc_j < num_vertices_-2; lc_j++) {
        //Shift vertex k back to origin frame
        Eigen::Vector2d v_ij = vertices_true_eucl_[lc_j].segment(0,2) - v_i;

        //Get angle between vectors u_j and v_k
        double u_dot_v = u_i.transpose() * v_ij;
        double theta_u_v = acos(u_dot_v/(u_i.norm()*v_ij.norm()));

        //Check edges within FOV and range
        if (u_dot_v > 0 && theta_u_v <= intra_lc_fov_/2.0 && v_ij.norm() <= intra_lc_range_) {

            //Add edge if random draw is within the desired probability
            double prob = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX));
            if (prob <= intra_lc_prob_) {
                // Add loop closure!
                RCLCPP_WARN(this->get_logger(), "Adding loop closure (%ld->%ld)", lc_i, lc_j);

                Eigen::Vector4d x_i = vertices_true_pudq_[lc_i];
                Eigen::Vector4d x_j = vertices_true_pudq_[lc_j];
                Eigen::Vector4d z_ij_true = pudq_lib::pudq_compose(pudq_inv(x_i), x_j);

                //Compute noisy delta-pose
                Eigen::Vector3d eta_ij;
                eta_ij << normrnd_theta_(gen_), normrnd_t_(gen_), normrnd_t_(gen_);
                Eigen::Vector4d z_ij_pudq = pudq_lib::pudq_compose(z_ij_true, Lie_Exp_1(eta_ij));

                Eigen::Matrix3d pudq_cov = Eigen::Matrix3d::Zero();
                pudq_cov(0,0) = (intra_sigma_theta_*intra_sigma_theta_);
                pudq_cov(1,1) = (intra_sigma_t_*intra_sigma_t_);
                pudq_cov(2,2) = (intra_sigma_t_*intra_sigma_t_);

                //Convert to message format
                std::array<double, 4> z_ij_pudq_msg{z_ij_pudq(0), z_ij_pudq(1), z_ij_pudq(2), z_ij_pudq(3)};
                std::array<double, 9> cov_pudq_msg{pudq_cov(0,0), 0.0, 0.0, 0.0, pudq_cov(1,1), 0.0, 0.0, 0.0, pudq_cov(2,2)};

                //Publish PUDQ edge message
                rclcpp::Time now = this->get_clock()->now();
                pudq_msgs::msg::PUDQEdge pudq_lc_edge_msg;
                pudq_lc_edge_msg.header.stamp = now;
                pudq_lc_edge_msg.edge_i = lc_i;
                pudq_lc_edge_msg.edge_j = lc_j;
                pudq_lc_edge_msg.delta_pose = z_ij_pudq_msg;
                pudq_lc_edge_msg.covariance = cov_pudq_msg;
                pudq_lc_publisher_->publish(pudq_lc_edge_msg);

                // Only add 1 loop closure per vertex
                return;
            }
        }
    }
}

void PUDQKeyframeGenerator::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg) {
    double yaw = quat_to_yaw(pose_msg->pose.orientation);

    //Extract 2D translation and rotation
    Eigen::Vector3d pose;
    pose << pose_msg->pose.position.x, pose_msg->pose.position.y, yaw;

    //Initialize previous pose if not done already
    if (!initialized_) {

        // Initialize map transform PUDQ and TF
        map_pudq_ = pose_to_pudq(pose);

        // Initialize pose in map frame
        map_pose_prev_ = pudq_to_pose(pudq_identity());

        vertices_true_pudq_.push_back(pudq_identity());
        vertices_true_eucl_.push_back(map_pose_prev_);
        num_vertices_ = 1;

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

    //If delta from previous keyframe is big enough, generate a new delta-pose "measurement"
    if (dt >= t_threshold_) {
        //Generate a keyframe for this pose (expressed in the local map frame)
        generate_keyframe(map_pose);

        // Check for simulated loop closure
        detect_loop_closure();
    }
}

void PUDQKeyframeGenerator::timer_callback() {
    if (initialized_) {
        map_tf_.header.stamp = this->get_clock()->now();
        map_tf_broadcaster_->sendTransform(map_tf_);
    }
}
