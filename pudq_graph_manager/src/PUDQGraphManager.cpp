#include "PUDQGraphManager.hpp"

#include <fstream>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

using namespace pudq_lib;

PUDQGraphManager::PUDQGraphManager() : Node("pudq_graph_manager_node") {
    RCLCPP_INFO(this->get_logger(), "Initializing PUDQ Graph Manager Node");

    //Seed the RNG
    srand(static_cast <unsigned> (time(0)));

    robot_name_ = std::string(this->get_namespace()).substr(1);

    //Get parameters
    std::string map_frame_noprefix;
    fixed_frame_id_ = this->declare_parameter<std::string>("fixed_frame", "world");
    map_frame_noprefix = this->declare_parameter<std::string>("map_frame", "map");
    map_frame_id_ = std::string(robot_name_).append("_").append(map_frame_noprefix);

    g2o_mode_ = this->declare_parameter<bool>("g2o_mode", false);
    if (g2o_mode_) {

        // Read g2o filename parameter
        g2o_file_ = this->declare_parameter<std::string>("g2o_file", "");
        if (g2o_file_.length() > 0) {
            RCLCPP_INFO(this->get_logger(), "Optimizing g2o file: %s", g2o_file_.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "No g2o file provided!");
            return;
        }
    }

    // private_nh_.param<std::string>("robot_name", robot_name_, "turtlebot3");
    // private_nh_.param<std::string>("map_frame", map_frame_noprefix, "map");
    // private_nh_.param<std::string>("fixed_frame", fixed_frame_id_, "mocap");
    // map_frame_id_ = robot_name_ + "/" + map_frame_noprefix;

    RCLCPP_INFO(this->get_logger(), "PUDQGraphManager %s: Map frame set to \'%s\'.", robot_name_.c_str(), map_frame_id_.c_str());

    //Initialize graph visualizer
    initialize_viz();

    //Graph visualization publishers
    vertices_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("vertices", rclcpp::ServicesQoS());
    edge_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("edge_viz", rclcpp::ServicesQoS());
    odom_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("odom_viz", rclcpp::ServicesQoS());
    edge_true_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("true_edge_viz", 10);
    // legend_publisher_.publish(legend_markers_);

    //Initialize tf listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    print_graph_service_ = this->create_service<std_srvs::srv::Empty>("print_graph", std::bind(&PUDQGraphManager::print_graph, this, _1, _2));
    print_cost_service_ = this->create_service<std_srvs::srv::Empty>("print_cost", std::bind(&PUDQGraphManager::print_cost, this, _1, _2));

    if (g2o_mode_) {
        read_g2o_file();

        // Start a 1 second viz update timer
        // viz_timer_ = this->create_wall_timer(1s, std::bind(&PUDQGraphManager::viz_timer_callback, this));

        pudq_pgo_lib::optimize_rlm(G, 1e-4, 100);

    } else {
        // Initialize graph to identity vertex
        initialize_graph();

        // Subscribe to true (e.g., mocap) vertices
        vertex_subscriber_ = this->create_subscription<pudq_msgs::msg::PUDQVertex>("pudq_vertex_true", 10, std::bind(&PUDQGraphManager::vertex_callback, this, _1));

        // Subsribe to odom and loop closure edges
        // lc_subscriber_ = nh_.subscribe<pudq_msgs::PUDQEdge>("pudq_loop_closure", 10, boost::bind(&PUDQGraphManager::lc_edge_callback, this, _1));
        edge_subscriber_ = this->create_subscription<pudq_msgs::msg::PUDQEdge>("pudq_edge", 10, std::bind(&PUDQGraphManager::edge_callback, this, _1));
        lc_subscriber_ = this->create_subscription<pudq_msgs::msg::PUDQEdge>("pudq_loop_closure", 10, std::bind(&PUDQGraphManager::lc_edge_callback, this, _1));

        // Start timer for visualization
        viz_timer_ = this->create_wall_timer(500ms, std::bind(&PUDQGraphManager::viz_timer_callback, this));
    }

    saved_g2o_ = false;

    // Distributed stuff
    // map_pudq_ = pudq_identity();
    // map_frame_init_ = false;
}

void PUDQGraphManager::initialize_graph() {
    G.clear();

    //Always initialize graph with fixed identity vertex
    G.add_vertex(pudq_lib::pudq_identity());

    //Initialize odom to identity as well
    odom_vertices_.push_back(pudq_to_pose(pudq_lib::pudq_identity()));

    RCLCPP_WARN(this->get_logger(),  "Pose graph initialized");
}

int PUDQGraphManager::read_g2o_file() {

    // Open g2o file for reading
    std::ifstream g2ofile(g2o_file_);

    // Check if the file opened successfully
    if (!g2ofile.is_open()) {   
        RCLCPP_ERROR(this->get_logger(), "Error opening file %s!\n", g2o_file_.c_str());
        return -1;
    }

    RCLCPP_INFO(this->get_logger(), "Processing g2o file");

    std::map<int, Eigen::Vector4d> init_vertices;
    int max_vertex = 0;

    std::string line;
    while (std::getline(g2ofile, line)) {
        // Todo: Catch g2o formatting errors
        std::string prefix;
        std::istringstream line_stream(line);
        line_stream >> prefix;

        if (prefix == std::string("EDGE_SE2")) {

            // The g2o format specifies a 3D relative pose measurement in the following form:
            // EDGE_SE2 id1 id2 dx dy dtheta, I11, I12, I13, I22, I23, I33
            int i, j;
            double dx, dy, dtheta;
            double I11, I12, I13, I22, I23, I33;
            line_stream >> i >> j >> dx >> dy >> dtheta >> I11 >> I12 >> I13 >> I22 >> I23 >> I33;

            Eigen::Vector3d dp_ij;
            dp_ij << dx, dy, dtheta;
            Eigen::Vector4d z_ij = pudq_lib::pose_to_pudq(dp_ij);

            Eigen::MatrixXd Omega_ij(3,3);
            Omega_ij << I11, I12, I13, I12, I22, I23, I13, I23, I33;

            // No self-loops, but we do allow edges where j >> i
            if (i != j) {
                G.add_edge(i, j, z_ij, Omega_ij);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Invalid self-loop from %d->%d\n", i, j);
            }

            //Update maximum vertex
            max_vertex = std::max(max_vertex, std::max(i, j));

        } else if (prefix == std::string("VERTEX_SE2")) {
            int i;
            double tx, ty, theta;

            line_stream >> i >> tx >> ty >> theta;

            Eigen::Vector3d pose_i;
            pose_i << tx, ty, theta;

            Eigen::Vector4d x_i;
            x_i = pudq_lib::pose_to_pudq(pose_i);

            //Check for duplicate vertices
            if (init_vertices.count(i) == 0) {
                init_vertices[i] = x_i;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Duplicate vertex %d in %s! Exiting.\n", i, g2o_file_.c_str());
                return -1;
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Encountered invalid line prefix \"%s\".\n", prefix.c_str());
            return -1;
        }
    }

    size_t num_vertices = max_vertex+1;
    G.init_vertices(num_vertices);

    // If vertices were included, initialize the graph with them
    if (init_vertices.size() > 0) {
        std::cout << "Reading initial vertices from " << g2o_file_ << std::endl;

        if (init_vertices.size() == num_vertices) {
            //Now, loop through and add each vertex in order. If any are missing, ignore and use odom
            for (size_t i = 0; i < num_vertices; i++) {
                if (init_vertices.count(i) > 0) {
                    G.set_vertex(i, init_vertices[i]);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Vertex %ld is missing from %s! Initializing from odometry.\n", i, g2o_file_.c_str());
                    break;
                }
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid number of vertices in %s! Initializing from odometry.\n", g2o_file_.c_str());
        }
    } else {
        // Not included, initialize vertices from odometry
        std::cout << "Vertices not included... initializing from odometry.\n" << std::endl;
        G.odom_init();
    }

    std::cout << "Read pose graph with " << G.get_num_vertices() << " vertices and " << G.get_num_edges() << " edges from " << g2o_file_ << "." << std::endl;

    g2ofile.close();

    G.odom_init();

    // Update odom vertices at initialization
    std::vector<Eigen::Vector3d> vertices_eucl = G.get_vertices_eucl();
    for (size_t i = 0; i < vertices_eucl.size(); i++) {
        odom_vertices_.push_back(vertices_eucl[i]);
    }

    // Note: this part is temporary, for demonstration purposes!
    // update_odom_viz();
    // update_estimate_viz();

    // update_odom_viz();
    // update_estimate_viz();

    // double F = pudq_pgo_lib::F_G_pudq(G);
    // std::cout << "F(X) = " << F << std::endl;
    // G.print_graph();

    return 0;
}

// Export G2O file format (NOTE: uses PUDQ information matrix format)
int PUDQGraphManager::write_g2o_file(std::string filename) {

    // If we have ground truth, make sure the Euclidean information matrices are correct
    // if (pg.get_num_true_vertices() == pg.get_num_vertices()) {
    //     pg.update_eucl_info();
    // } else {
    //     std::cout << "Warning: Exporting .g2o file without updating Euclidean information matrices!" << std::endl;
    // }
    
    // Open g2o file for writing
    std::ofstream g2ofile(filename);

    //Set desired precision to 16 decimal places to preserve covariances
    g2ofile << std::fixed << std::setprecision(16);

    // Check if the file opened successfully
    if (!g2ofile.is_open()) {
        fprintf(stderr, "Error opening file!\n");
        return -1;
    }

    std::vector<PUDQGraph::Edge> edge_vec = G.get_edges();
    for (size_t ij = 0; ij < edge_vec.size(); ij++) {

        size_t edge_i = edge_vec[ij].i;
        size_t edge_j = edge_vec[ij].j;

        Eigen::Matrix3d Omega_ij = edge_vec[ij].Omega_ij_pudq;
        Eigen::Vector3d p_ij = edge_vec[ij].z_ij_eucl;

        // g2o edge format: i j dtx dty dtheta I11..I33
        g2ofile << "EDGE_SE2 " << edge_i << " " << edge_j << " " << p_ij(0) << " " << p_ij(1) << " " << p_ij(2) << " ";

        // Loop through upper triangle of Omega_ij
        for (size_t i = 0; i < 3; i++) {
            for (size_t j = i; j < 3; j++) {
                g2ofile << Omega_ij(i,j);

                if (i < 2 || i < 2) {
                    g2ofile << " ";
                }
            }
        }
        g2ofile << std::endl;
    }

    // Write current vertices to g2o file
    std::vector<Eigen::Vector3d> vertices_eucl = G.get_vertices_eucl();
    for (size_t i = 0; i < vertices_eucl.size(); i++) {
        Eigen::Vector3d p_i = vertices_eucl[i];

        // g2o vertex format: id tx ty theta
        g2ofile << "VERTEX_SE2 " << i << " " << p_i(0) << " " << p_i(1) << " " << p_i(2) << std::endl;
    }

    // Close the file
    g2ofile.close();

    std::cout << "Saved " << filename << std::endl;
    return 0;
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

    std::vector<Eigen::Vector3d> vertices = G.get_vertices_eucl();

    for (size_t i = 0; i < vertices.size(); i++) {
        geometry_msgs::msg::Pose vertex_msg;
        vertex_msg.position.x = vertices[i](0);
        vertex_msg.position.y = vertices[i](1);
        double theta_i = vertices[i](2);
        vertex_msg.orientation.w = cos(theta_i/2);
        vertex_msg.orientation.z = sin(theta_i/2);
        vertices_msg.poses.push_back(vertex_msg);
    }
    vertices_publisher_->publish(vertices_msg);

    if (vertices.size() > 1) {
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

        // Plot edges (odom and intra-lc) and loop closures
        std::map<size_t, std::map<size_t, size_t>> adjacency_list = G.get_adjacency();
        for (auto it0 = adjacency_list.begin(); it0 != adjacency_list.end(); ++it0) {

            //Prepare edge i vertex
            geometry_msgs::msg::Point edge_vertex_i;
            edge_vertex_i.x = vertices[it0->first](0);
            edge_vertex_i.y = vertices[it0->first](1);
            edge_vertex_i.z = 0.0;

            for (auto it1 = it0->second.begin(); it1 != it0->second.end(); ++it1) {

                bool is_lc = abs((int)it0->first - (int)it1->first) > 1;

                //Draw line between 2 vertices
                edges_marker_.points.push_back(edge_vertex_i);

                geometry_msgs::msg::Point edge_vertex_j;
                edge_vertex_j.x = vertices[it1->first](0);
                edge_vertex_j.y = vertices[it1->first](1);
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
            odom_marker_.colors.push_back(odom_color_);

            geometry_msgs::msg::Point odom_vertex_j;
            odom_vertex_j.x = odom_vertices_[i](0);
            odom_vertex_j.y = odom_vertices_[i](1);
            odom_vertex_j.z = 0.0;
            odom_marker_.points.push_back(odom_vertex_j);
            odom_marker_.colors.push_back(odom_color_);
        }

        odom_publisher_->publish(odom_marker_);
    }
}

void PUDQGraphManager::update_truth_viz() {

    std::vector<Eigen::Vector3d> vertices_true_eucl = G.get_vertices_true_eucl();
    // for (size_t i = 1; i < vertices_true_eucl.size(); i++) {
    //     std::cout << i << ") " << vertices_true_eucl[i-1].transpose() << " -> " << vertices_true_eucl[i].transpose() << std::endl;
    // }

    //Draw true trajectory minus loop closures
    if (vertices_true_eucl.size() > 1) {
        edges_true_marker_.header.stamp = this->get_clock()->now();

        if (true_marker_init_) {
            edges_true_marker_.action = visualization_msgs::msg::Marker::MODIFY;
            edges_true_marker_.points.clear();
            edges_true_marker_.colors.clear();
        } else {
            edges_true_marker_.action = visualization_msgs::msg::Marker::ADD;
            true_marker_init_ = true;
        }

        for (size_t i = 1; i < vertices_true_eucl.size(); i++) {
            geometry_msgs::msg::Point edge_vertex_true_i;
            edge_vertex_true_i.x = vertices_true_eucl[i-1](0);
            edge_vertex_true_i.y = vertices_true_eucl[i-1](1);
            edge_vertex_true_i.z = 0.0;
            edges_true_marker_.points.push_back(edge_vertex_true_i);
            edges_true_marker_.colors.push_back(true_color_);

            geometry_msgs::msg::Point edge_vertex_true_j;
            edge_vertex_true_j.x = vertices_true_eucl[i](0);
            edge_vertex_true_j.y = vertices_true_eucl[i](1);
            edge_vertex_true_j.z = 0.0;
            edges_true_marker_.points.push_back(edge_vertex_true_j);
            edges_true_marker_.colors.push_back(true_color_);
        }

        edge_true_publisher_->publish(edges_true_marker_);
    }
}

void PUDQGraphManager::viz_timer_callback() {
    update_odom_viz();
    update_estimate_viz();
}

// Vertex callback only applies to ground truth
void PUDQGraphManager::vertex_callback(const pudq_msgs::msg::PUDQVertex::SharedPtr vertex_msg) {

    // RCLCPP_INFO(this->get_logger(), "vertex callback");

    //First, make sure vertex is valid
    if (vertex_msg->id != G.get_vertices_true().size()) {
        RCLCPP_ERROR(this->get_logger(), "Invalid vertex %lu", vertex_msg->id);
        return;
    }

    Eigen::Vector4d vertex_pudq;
    vertex_pudq << vertex_msg->pose[0], vertex_msg->pose[1], vertex_msg->pose[2], vertex_msg->pose[3];

    G.add_vertex_true(vertex_pudq);
    update_truth_viz();
}

// Add odom edge to graph
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

    // Compute the associated odometry vertex and add it to the graph
    Eigen::Vector4d x_j = pudq_compose(G.get_vertex(edge_msg->edge_i), z_ij);
    G.add_vertex(x_j);

    // Save odom vertex permanently
    Eigen::Vector4d x_j_odom = pudq_compose(pose_to_pudq(odom_vertices_[edge_msg->edge_i]), z_ij);
    odom_vertices_.push_back(pudq_to_pose(x_j_odom));
    update_odom_viz();

    // Add odom edge to the graph
    G.add_edge(edge_msg->edge_i, edge_msg->edge_j, z_ij, info_pudq);
    update_estimate_viz();

    // Check for unprocessed loop closure originating from vertex j
    if (lc_edge_msg_queue.size() > 0) {
        pudq_msgs::msg::PUDQEdge::SharedPtr lc_edge_msg = lc_edge_msg_queue.front();
        if (lc_edge_msg->edge_i == edge_msg->edge_j) {
            // It's a match! Process and pop it off the queue.
            process_lc_edge(lc_edge_msg);
            lc_edge_msg_queue.pop();
        }
    }
}

// Add loop closure edge to graph
void PUDQGraphManager::lc_edge_callback(const pudq_msgs::msg::PUDQEdge::SharedPtr lc_edge_msg) {

    // First, make sure both vertices are valid
    size_t N = G.get_num_vertices();
    if (lc_edge_msg->edge_i < N && lc_edge_msg->edge_j < N) {
        process_lc_edge(lc_edge_msg);
    } else if (lc_edge_msg->edge_i == N && lc_edge_msg->edge_j < N) {
        // LC came in slightly early, so add it to the queue
        lc_edge_msg_queue.push(lc_edge_msg);
    } else {
        // Invalid LC
        RCLCPP_ERROR(this->get_logger(), "PUDQGraphManager %s: Invalid loop closure edge (%lu, %lu)", robot_name_.c_str(), lc_edge_msg->edge_i, lc_edge_msg->edge_j);
    }
}

void PUDQGraphManager::process_lc_edge(const pudq_msgs::msg::PUDQEdge::SharedPtr lc_edge_msg) {
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

    size_t edge_i = lc_edge_msg->edge_i;
    size_t edge_j = lc_edge_msg->edge_j;
    RCLCPP_WARN(this->get_logger(), "Adding loop closure (%ld->%ld)", lc_edge_msg->edge_i, lc_edge_msg->edge_j);

    //Add loop closure edge to the graph
    G.add_edge(lc_edge_msg->edge_i, lc_edge_msg->edge_j, z_ij, info_pudq);
    update_estimate_viz();

    // Optimize
    pudq_pgo_lib::optimize_rlm(G, 1e-5, 100);

    // if (!saved_g2o_) {
    //     saved_g2o_ = true;
    //     write_g2o_file(std::string("/home/corelab/pudq_pgo_ros2_ws/src/pudq_pgo_ros2/pudq_graph_manager/g2o/online_test.g2o"));
    // }

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
    
    double F = pudq_pgo_lib::F_G_pudq(G);
    std::cout << "(print_cost service) F(X) = " << F << std::endl;
}