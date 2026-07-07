#include "PUDQGraphManager.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

using namespace pudq_lib;

PUDQGraphManager::PUDQGraphManager() : Node("pudq_graph_manager_node") {
    RCLCPP_INFO(this->get_logger(), "Initializing PUDQ Graph Manager Node");

    //Seed the RNG
    srand (static_cast <unsigned> (time(0)));

    robot_name_ = std::string(this->get_namespace()).substr(1);

    // private_nh_.param<bool>("use_g2o", use_g2o_, false);

    //Get parameters
    std::string map_frame_noprefix;
    fixed_frame_id_ = this->declare_parameter<std::string>("fixed_frame", "world");
    map_frame_noprefix = this->declare_parameter<std::string>("map_frame", "map");
    map_frame_id_ = std::string(robot_name_).append("_").append(map_frame_noprefix);

    // private_nh_.param<std::string>("robot_name", robot_name_, "turtlebot3");
    // private_nh_.param<std::string>("map_frame", map_frame_noprefix, "map");
    // private_nh_.param<std::string>("fixed_frame", fixed_frame_id_, "mocap");
    // map_frame_id_ = robot_name_ + "/" + map_frame_noprefix;

    RCLCPP_INFO(this->get_logger(), "PUDQGraphManager %s: Map frame set to \'%s\'.", robot_name_.c_str(), map_frame_id_.c_str());

    //Initialize graph object
    initialize_graph();

    //Initialize graph visualizer
    initialize_viz();

    //Initialize tf listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    //Distributed stuff
    // map_pudq_ = pudq_identity();
    // map_frame_init_ = false;

    print_graph_service_ = this->create_service<std_srvs::srv::Empty>("print_graph", std::bind(&PUDQGraphManager::print_graph, this, _1, _2));
    print_cost_service_ = this->create_service<std_srvs::srv::Empty>("print_cost", std::bind(&PUDQGraphManager::print_cost, this, _1, _2));

    //Subscribe to true (e.g., mocap) vertices
    vertex_subscriber_ = this->create_subscription<pudq_msgs::msg::PUDQVertex>("pudq_vertex_true", 10, std::bind(&PUDQGraphManager::vertex_callback, this, _1));

    //Subsribe to odom and loop closure edges
    // lc_subscriber_ = nh_.subscribe<pudq_msgs::PUDQEdge>("pudq_loop_closure", 10, boost::bind(&PUDQGraphManager::lc_edge_callback, this, _1));

    edge_subscriber_ = this->create_subscription<pudq_msgs::msg::PUDQEdge>("pudq_edge", 10, std::bind(&PUDQGraphManager::edge_callback, this, _1));
    // lc_subscriber_ = this->create_subscription<pudq_msgs::msg::PUDQEdge>("pudq_loop_closure", 10, std::bind(&PUDQGraphManager::lc_edge_callback, this, _1));

    //Graph visualization publishers
    vertices_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("vertices", 10);
    edge_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("edge_viz", 10);
    odom_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("odom_viz", 10);
    edge_true_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("true_edge_viz", 10);

    // legend_publisher_.publish(legend_markers_);
}

// t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);

void PUDQGraphManager::initialize_graph() {
    G.clear();

    //Always initialize graph with fixed identity vertex
    G.add_vertex(pudq_identity());
    
    //Initialize odom to identity as well
    odom_vertices_.push_back(pudq_to_pose(pudq_identity()));

    RCLCPP_WARN(this->get_logger(),  "Pose graph initialized");
}

void PUDQGraphManager::initialize_viz() {
    double line_width = 0.01;

    edge_color_ = color(0.0, 0.0, 1.0, 1.0);     //Blue
    lc_color_ = color(1.0, 1.0, 0.0, 1.0);       //Yellow
    true_color_ = color(0.0, 0.75, 0.0, 1.0);    //Green
    odom_color_ = color(1.0, 0.0, 1.0, 1.0);     //Pink
    multi_lc_color_ = color(1.0, 0.0, 0.0, 1.0); //Red
    
    edges_marker_.header.frame_id = map_frame_id_;
    edges_marker_.ns = robot_name_ + "/edges";
    edges_marker_.id = 0;
    edges_marker_.type = visualization_msgs::msg::Marker::LINE_LIST;
    edges_marker_.color = edge_color_;
    edges_marker_.scale = vector3(line_width, 0.0, 0.0);
    edges_marker_.pose.position = point3d(0.0, 0.0, 0.0);
    edges_marker_.pose.orientation = yaw_to_quat(0.0);

    edges_true_marker_.header.frame_id = map_frame_id_;
    edges_true_marker_.ns = robot_name_ + "/edges_true";
    edges_true_marker_.id = 0;
    edges_true_marker_.type = visualization_msgs::msg::Marker::LINE_LIST;
    edges_true_marker_.color = edge_color_;
    edges_true_marker_.scale = vector3(line_width, 0.0, 0.0);
    edges_true_marker_.pose.position = point3d(0.0, 0.0, 0.0);
    edges_true_marker_.pose.orientation = yaw_to_quat(0.0);

    //Odom edges
    odom_marker_.header.frame_id = map_frame_id_;
    odom_marker_.ns = robot_name_ + "/odom_edges";
    odom_marker_.id = 0;
    odom_marker_.type = visualization_msgs::msg::Marker::LINE_LIST;
    odom_marker_.color = odom_color_;
    odom_marker_.scale = vector3(line_width, 0.0, 0.0);
    odom_marker_.pose.position = point3d(0.0, 0.0, 0.0);
    odom_marker_.pose.orientation = yaw_to_quat(0.0);

    //Multi-lc edges
    // multi_lc_marker_.header.frame_id = map_frame_id_;
    // multi_lc_marker_.ns = robot_name_ + "/multi_lc_edges";
    // multi_lc_marker_.id = 0;
    // multi_lc_marker_.type = visualization_msgs::Marker::LINE_LIST;
    // multi_lc_marker_.color = multi_lc_color_;
    // multi_lc_marker_.scale = vector3(0.0075, 0.0, 0.0);
    // multi_lc_marker_.pose.position = point3d(0.0, 0.0, 0.0);
    // multi_lc_marker_.pose.orientation = yaw_to_quat(0.0);
    // multi_lc_marker_.lifetime = ros::Duration(0.0);

    //Initialize color-coded RVIZ legend
    // legend_markers_.markers.clear();
    // visualization_msgs::Marker truth_legend_text, lc_legend_text, inter_lc_legend_text, odom_legend_text, est_legend_text;

    // int legend_id = 0;
    // double legend_x_pos = -0.05;
    // truth_legend_text.header.frame_id = fixed_frame_id_;
    // truth_legend_text.ns = "legend";
    // truth_legend_text.id = legend_id;
    // truth_legend_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    // truth_legend_text.action = visualization_msgs::Marker::ADD;
    // truth_legend_text.color = true_color_;
    // truth_legend_text.pose.position.x = legend_x_pos;
    // truth_legend_text.pose.position.y = 1.0;
    // truth_legend_text.pose.position.z = 0.0;
    // truth_legend_text.pose.orientation = yaw_to_quat(0.0);
    // truth_legend_text.scale = vector3(0.15, 0.15, 0.15);
    // truth_legend_text.lifetime = ros::Duration(0);
    // truth_legend_text.text = "True Trajectory";
    // legend_markers_.markers.push_back(truth_legend_text);

    // legend_id++;
    // legend_x_pos -= 0.2;
    // lc_legend_text.header.frame_id = fixed_frame_id_;
    // lc_legend_text.ns = "legend";
    // lc_legend_text.id = legend_id;
    // lc_legend_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    // lc_legend_text.action = visualization_msgs::Marker::ADD;
    // lc_legend_text.color = lc_color_;
    // lc_legend_text.pose.position.x = legend_x_pos;
    // lc_legend_text.pose.position.y = 1.0;
    // lc_legend_text.pose.position.z = 0.0;
    // lc_legend_text.pose.orientation = yaw_to_quat(0.0);
    // lc_legend_text.scale = vector3(0.15, 0.15, 0.15);
    // lc_legend_text.lifetime = ros::Duration(0);
    // lc_legend_text.text = "Intra-agent Loop Closures";
    // legend_markers_.markers.push_back(lc_legend_text);

    // legend_id++;
    // legend_x_pos -= 0.2;
    // inter_lc_legend_text.header.frame_id = fixed_frame_id_;
    // inter_lc_legend_text.ns = "legend";
    // inter_lc_legend_text.id = legend_id;
    // inter_lc_legend_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    // inter_lc_legend_text.action = visualization_msgs::Marker::ADD;
    // inter_lc_legend_text.color = color(1.0, 0.0, 0.0, 1.0);
    // inter_lc_legend_text.pose.position.x = legend_x_pos;
    // inter_lc_legend_text.pose.position.y = 1.0;
    // inter_lc_legend_text.pose.position.z = 0.0;
    // inter_lc_legend_text.pose.orientation = yaw_to_quat(0.0);
    // inter_lc_legend_text.scale = vector3(0.15, 0.15, 0.15);
    // inter_lc_legend_text.lifetime = ros::Duration(0);
    // inter_lc_legend_text.text = "Inter-agent Loop Closures";
    // legend_markers_.markers.push_back(inter_lc_legend_text);

    // legend_id++;
    // legend_x_pos -= 0.2;
    // odom_legend_text.header.frame_id = fixed_frame_id_;
    // odom_legend_text.ns = "legend";
    // odom_legend_text.id = legend_id;
    // odom_legend_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    // odom_legend_text.action = visualization_msgs::Marker::ADD;
    // odom_legend_text.color = odom_color_;
    // odom_legend_text.pose.position.x = legend_x_pos;
    // odom_legend_text.pose.position.y = 1.0;
    // odom_legend_text.pose.position.z = 0.0;
    // odom_legend_text.pose.orientation = yaw_to_quat(0.0);
    // odom_legend_text.scale = vector3(0.15, 0.15, 0.15);
    // odom_legend_text.lifetime = ros::Duration(0);
    // odom_legend_text.text = "Odom Trajectory";
    // legend_markers_.markers.push_back(odom_legend_text);

    // legend_id++;
    // legend_x_pos -= 0.2;
    // est_legend_text.header.frame_id = fixed_frame_id_;
    // est_legend_text.ns = "legend";
    // est_legend_text.id = legend_id;
    // est_legend_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    // est_legend_text.action = visualization_msgs::Marker::ADD;
    // est_legend_text.color = edge_color_;
    // est_legend_text.pose.position.x = legend_x_pos;
    // est_legend_text.pose.position.y = 1.0;
    // est_legend_text.pose.position.z = 0.0;
    // est_legend_text.pose.orientation = yaw_to_quat(0.0);
    // est_legend_text.scale = vector3(0.15, 0.15, 0.15);
    // est_legend_text.lifetime = ros::Duration(0);
    // est_legend_text.text = "PUDQ Optimized Trajectory";
    // legend_markers_.markers.push_back(est_legend_text);

    // if (use_g2o_) {
    //     legend_id++;
    //     legend_x_pos -= 0.2;
    //     visualization_msgs::Marker g2o_legend_text;
    //     g2o_legend_text.header.frame_id = fixed_frame_id_;
    //     g2o_legend_text.ns = "legend";
    //     g2o_legend_text.id = legend_id;
    //     g2o_legend_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    //     g2o_legend_text.action = visualization_msgs::Marker::ADD;
    //     g2o_legend_text.color = color(1.0, 0.5, 0.0, 1.0); // Orange
    //     g2o_legend_text.pose.position.x = legend_x_pos;
    //     g2o_legend_text.pose.position.y = 1.0;
    //     g2o_legend_text.pose.position.z = 0.0;
    //     g2o_legend_text.pose.orientation = yaw_to_quat(0.0);
    //     g2o_legend_text.scale = vector3(0.15, 0.15, 0.15);
    //     g2o_legend_text.lifetime = ros::Duration(0);
    //     g2o_legend_text.text = "G2O Optimized Trajectory";
    //     legend_markers_.markers.push_back(g2o_legend_text);
    // }

    est_marker_init_  = false;
    odom_marker_init_ = false;
    true_marker_init_ = false;
    multi_lc_viz_init_ = false;
}

void PUDQGraphManager::update_estimate_viz() {
    rclcpp::Time now = this->get_clock()->now();

    geometry_msgs::msg::PoseArray vertices_msg;
    vertices_msg.header.stamp = now;
    vertices_msg.header.frame_id = map_frame_id_;

    for (size_t i = 0; i < G.vertices_.size(); i++) {
        geometry_msgs::msg::Pose vertex_msg;
        vertex_msg.position.x = G.vertices_[i](0);
        vertex_msg.position.y = G.vertices_[i](1);
        double theta_i = G.vertices_[i](2);
        vertex_msg.orientation.w = cos(theta_i/2);
        vertex_msg.orientation.z = sin(theta_i/2);
        vertices_msg.poses.push_back(vertex_msg);
    }
    vertices_publisher_->publish(vertices_msg);

    if (G.vertices_.size() > 1) {
        edges_marker_.header.stamp = now;

        if (est_marker_init_) {
            edges_marker_.action = visualization_msgs::msg::Marker::MODIFY;
            edges_marker_.points.clear();
            edges_marker_.colors.clear();
        } else {
            //Initialize markers
            edges_marker_.action = visualization_msgs::msg::Marker::ADD;
            est_marker_init_ = true;
        }

        //Plot edges (odom and intra-lc) and loop closures
        for (auto it0 = G.edges_.begin(); it0 != G.edges_.end(); ++it0) {

            //Prepare edge i vertex
            geometry_msgs::msg::Point edge_vertex_i;
            edge_vertex_i.x = G.vertices_[it0->first](0);
            edge_vertex_i.y = G.vertices_[it0->first](1);
            edge_vertex_i.z = 0.0;

            for (auto it1 = it0->second.begin(); it1 != it0->second.end(); ++it1) {

                bool is_lc = abs((int)it0->first - (int)it1->first) > 1;

                //Draw line between 2 vertices
                edges_marker_.points.push_back(edge_vertex_i);

                geometry_msgs::msg::Point edge_vertex_j;
                edge_vertex_j.x = G.vertices_[it1->first](0);
                edge_vertex_j.y = G.vertices_[it1->first](1);
                edge_vertex_j.z = 0.0;
                edges_marker_.points.push_back(edge_vertex_j);

                if (is_lc) {
                    edges_marker_.colors.push_back(lc_color_);
                    edges_marker_.colors.push_back(lc_color_);
                } else {
                    edges_marker_.colors.push_back(edge_color_);
                    edges_marker_.colors.push_back(edge_color_);
                }
            }
        }

        edge_publisher_->publish(edges_marker_);
    }
}

void PUDQGraphManager::update_odom_viz() {
    //Draw true trajectory minus loop closures
    if (odom_vertices_.size() > 1) {
        odom_marker_.header.stamp = this->get_clock()->now();

        if (odom_marker_init_) {
            odom_marker_.action = visualization_msgs::msg::Marker::MODIFY;
            odom_marker_.points.clear();
            odom_marker_.colors.clear();
        } else {
            odom_marker_.action = visualization_msgs::msg::Marker::ADD;
            odom_marker_init_ = true;
        }

        for (unsigned int i = 1; i < odom_vertices_.size(); i++) {
            geometry_msgs::msg::Point odom_vertex_i;
            odom_vertex_i.x = odom_vertices_[i-1](0);
            odom_vertex_i.y = odom_vertices_[i-1](1);
            odom_vertex_i.z = 0.0;
            odom_marker_.points.push_back(odom_vertex_i);

            geometry_msgs::msg::Point odom_vertex_j;
            odom_vertex_j.x = odom_vertices_[i](0);
            odom_vertex_j.y = odom_vertices_[i](1);
            odom_vertex_j.z = 0.0;
            odom_marker_.points.push_back(odom_vertex_j);

            odom_marker_.colors.push_back(odom_color_);
            odom_marker_.colors.push_back(odom_color_);
        }

        odom_publisher_->publish(odom_marker_);
    }
}

void PUDQGraphManager::update_truth_viz() {
    //Draw true trajectory minus loop closures
    if (G.vertices_true_.size() > 1) {
        edges_true_marker_.header.stamp = this->get_clock()->now();

        if (true_marker_init_) {
            edges_true_marker_.action = visualization_msgs::msg::Marker::MODIFY;
            edges_true_marker_.points.clear();
            edges_true_marker_.colors.clear();
        } else {
            edges_true_marker_.action = visualization_msgs::msg::Marker::ADD;
            true_marker_init_ = true;
        }

        for (unsigned int i = 1; i < G.vertices_true_.size(); i++) {
            geometry_msgs::msg::Point edge_vertex_true_i;
            edge_vertex_true_i.x = G.vertices_true_[i-1](0);
            edge_vertex_true_i.y = G.vertices_true_[i-1](1);
            edge_vertex_true_i.z = 0.0;
            edges_true_marker_.points.push_back(edge_vertex_true_i);
            edges_true_marker_.colors.push_back(true_color_);

            geometry_msgs::msg::Point edge_vertex_true_j;
            edge_vertex_true_j.x = G.vertices_true_[i](0);
            edge_vertex_true_j.y = G.vertices_true_[i](1);
            edge_vertex_true_j.z = 0.0;
            edges_true_marker_.points.push_back(edge_vertex_true_j);
            edges_true_marker_.colors.push_back(true_color_);
        }

        edge_true_publisher_->publish(edges_true_marker_);
    }
}

void PUDQGraphManager::vertex_callback(const pudq_msgs::msg::PUDQVertex::SharedPtr vertex_msg) {
    //First, make sure vertex is valid
    if (vertex_msg->id != G.vertices_true_.size()) {
        RCLCPP_ERROR(this->get_logger(), "Invalid vertex %lu", vertex_msg->id);
        return;
    }

    Eigen::Vector4d vertex_pudq;
    vertex_pudq << vertex_msg->pose[0], vertex_msg->pose[1], vertex_msg->pose[2], vertex_msg->pose[3];

    G.add_vertex_true(vertex_pudq);
    update_truth_viz();
}

//Add odom edge to graph
void PUDQGraphManager::edge_callback(const pudq_msgs::msg::PUDQEdge::SharedPtr edge_msg) {

    //First, make sure both vertices are valid
    if (!(edge_msg->edge_i < G.get_num_vertices() && edge_msg->edge_j == G.get_num_vertices())) {
        RCLCPP_ERROR(this->get_logger(), "Invalid odom edge (%lu, %lu)", edge_msg->edge_i, edge_msg->edge_j);
        return;
    }

    Eigen::Vector4d z_ij;
    z_ij << edge_msg->delta_pose[0], edge_msg->delta_pose[1], edge_msg->delta_pose[2], edge_msg->delta_pose[3];

    Eigen::Matrix3d cov_pudq;
    int k = 0;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            cov_pudq(i,j) = edge_msg->covariance[k++];
        }
    }
    Eigen::Matrix3d info_pudq = cov_pudq.inverse();

    //Compute the associated odometry vertex and add it to the graph
    Eigen::Vector4d x_j = pudq_compose(G.vertices_pudq_[edge_msg->edge_i], z_ij);
    G.add_vertex(x_j);

    //Save odom vertex permanently
    Eigen::Vector4d x_j_odom = pudq_compose(pose_to_pudq(odom_vertices_[edge_msg->edge_i]), z_ij);
    odom_vertices_.push_back(pudq_to_pose(x_j_odom));
    update_odom_viz();

    //Add odom edge to the graph
    G.add_edge(edge_msg->edge_i, edge_msg->edge_j, z_ij, info_pudq);
    update_estimate_viz();
}

// Add loop closure edge to graph
void PUDQGraphManager::lc_edge_callback(const pudq_msgs::msg::PUDQEdge::SharedPtr &lc_edge_msg) {
    // First, make sure both vertices are valid
    if (!(lc_edge_msg->edge_i < G.get_num_vertices() && lc_edge_msg->edge_j < G.get_num_vertices())) {
        RCLCPP_ERROR(this->get_logger(), "PUDQGraphManager %s: Invalid loop closure edge (%lu, %lu)", robot_name_.c_str(), lc_edge_msg->edge_i, lc_edge_msg->edge_j);
        return;
    }

    Eigen::Vector4d z_ij;
    z_ij << lc_edge_msg->delta_pose[0], lc_edge_msg->delta_pose[1], lc_edge_msg->delta_pose[2], lc_edge_msg->delta_pose[3];

    Eigen::Matrix3d cov_pudq;
    int k = 0;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            cov_pudq(i,j) = lc_edge_msg->covariance[k++];
        }
    }
    Eigen::Matrix3d info_pudq = cov_pudq.inverse();

    //Add loop closure edge to the graph
    G.add_edge(lc_edge_msg->edge_i, lc_edge_msg->edge_j, z_ij, info_pudq);
    update_estimate_viz();

    // Optimize
    pudq_pgo_lib::optimize_rgn_fast(&G, 1e-5, 10);

    //Publish new vertices
    update_estimate_viz();

    // CHEATING - publish new vertices as public every time we optimize
    // publish_public_poses();
}

void PUDQGraphManager::print_graph(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response) {
    (void)request;
    (void)response;

    G.print_graph();
}

void PUDQGraphManager::print_cost(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response) {
    (void)request;
    (void)response;
    
    double F = pudq_pgo_lib::F_G_pudq(&G);
    std::cout << "F(X) = " << F << std::endl;
}