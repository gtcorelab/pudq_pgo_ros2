#include "pudq_pgo_lib/PUDQGraph.hpp"

#include <iostream>

using namespace pudq_lib;

PUDQGraph::PUDQGraph() {
    num_vertices_ = 0;
    num_edges_ = 0;
}

size_t PUDQGraph::get_num_vertices() {
    return num_vertices_;
}

size_t PUDQGraph::get_num_edges() {
    return num_edges_;
}

Eigen::VectorXd PUDQGraph::get_X() {
    Eigen::VectorXd X = Eigen::VectorXd::Zero(4*num_vertices_);

    for (size_t i = 0; i < num_vertices_; i++) {
        X.segment(4*i, 4) = vertices_pudq_[i];
    }

    return X;
}

void PUDQGraph::clear() {
    vertices_.clear();
    vertices_pudq_.clear();
    edge_vec_.clear();
}

// This initializes all vertices from odometry measurements
void PUDQGraph::odom_init() {
    // Todo: improve this process (works for now)
    if (vertices_udq_.size() != max_vertex_ + 1) {
        // Reinitialize if the edge and vertex dimensions don't line up
        // init_vertices(max_vertex_ + 1);

        std::cout << "Warning: vertices and max_vertex_ don't line up..." << std::endl;
    }

    // Fix the origin to identity
    set_vertex(0, pudq_lib::pudq_identity());
    for (size_t j = 1; j < vertices_pudq_.size(); j++) {
        
        size_t i = j-1;

        // Make sure there exists an odom edge from i to j
        bool odom_edge_exists = false;
        if (adjacency_list.count(i) > 0 || adjacency_list.count(j) > 0) {
            if (adjacency_list[i].count(j) > 0) {
                odom_edge_exists = true;

                // Propogate odom via x_j = x_i comp. z_ij
                DualQuaternion x_i_odom = vertices_pudq_[i];
                DualQuaternion x_j_odom = x_i_odom * edge_vec_[adjacency_list[i][j]].z_ij;
                set_vertex(j, x_j_odom);
            } else if (adjacency_list[j].count(i) > 0) {
                // Account for reverse odom edges in some datasets
                odom_edge_exists = true;

                // Propogate odom via x_j = x_i comp. z_ij^(-1)
                DualQuaternion x_i_odom = vertices_pudq_[i];
                DualQuaternion x_j_odom = x_i_odom * edge_vec_[adjacency_list[i][j]].z_ij.inverse();
                set_vertex(j, x_j_odom);
            }
        }

        if (!odom_edge_exists) {
            // Don't print this warning for now bc some datasets are just FUBARed
            // fprintf(stderr, "Warning: Missing odom edge from %ld->%ld! Assuming identity.\n", i, j);

            //Propogate odom via x_j = x_i comp. z_ij
            DualQuaternion z_ij_id = pudq_lib::pudq_identity();
            DualQuaternion x_i_odom = vertices_udq_[i];
            DualQuaternion x_j_odom = x_i_odom * z_ij_id;
            set_vertex(j, x_j_odom);
        }
    }
}

void PUDQGraph::construct_omega() {
    //Constructs big omega matrix
    Omega = Eigen::SparseMatrix<double, Eigen::RowMajor>(3*num_edges_, 3*num_edges_);

    for (size_t ij = 0; ij < num_edges_; ij++) {
        for (int i = 0; i < edge_vec_[ij].Omega_ij_pudq.rows(); i++) {
            for (int j = 0; j < edge_vec_[ij].Omega_ij_pudq.cols(); j++) {
                Omega.insert(3*ij+i, 3*ij+j) = edge_vec_[ij].Omega_ij_pudq(i,j);
            }
        }
    }
}

void PUDQGraph::set_vertex(size_t i, Eigen::Vector4d vertex_pudq) {
    //Todo:: check for out of bounds
    vertices_pudq_[i] = vertex_pudq;
    vertices_[i] = pudq_to_pose(vertex_pudq);
}

void PUDQGraph::add_vertex(Eigen::Vector4d vertex_pudq) {
    //Add PUDQ and equivalent Euclidean vertices to the graph
    vertices_pudq_.push_back(vertex_pudq);
    vertices_.push_back(pudq_to_pose(vertex_pudq));
    num_vertices_++;
}

void PUDQGraph::add_vertex_true(Eigen::Vector4d vertex_pudq) {
    //Add PUDQ and equivalent Euclidean vertices to the graph
    vertices_true_pudq_.push_back(vertex_pudq);
    vertices_true_.push_back(pudq_to_pose(vertex_pudq));
}

void PUDQGraph::add_edge(size_t i, size_t j, Eigen::Vector4d edge_pudq, Eigen::Matrix3d info_pudq) {

    //Perform necessary checks to make sure this edge can exist
    // if (!(i < vertices_pudq_.size() && j < vertices_pudq_.size())) {
    //     printf("PUDQGraph add_edge: Edge (%ld,%ld) out of range!\n", i, j);
    //     return;
    // }

    if (i == j) {
        printf("PUDQGraph add_edge: No self-loops!\n");
        return;
    }

    // if (edge_exists(i, j)) {
    //     printf("PUDQGraph add_edge: Edge (%ld,%ld) already exists!\n", i, j);
    //     return;
    // }

    //Populate edge struct
    Edge e;
    e.z_ij_pudq = edge_pudq;
    e.Omega_ij_pudq = info_pudq;

    e.z_ij_eucl = pudq_to_pose(edge_pudq);

    // Todo::Convert info mat from PUDQ to SE(2) - NEED TRUE ERROR FOR THIS
    e.Omega_ij_eucl = info_pudq;

    // Add edge to edge vector
    edge_vec_.push_back(e);

    // Add edge to adjacency list (value is index of edge vector)
    adjacency_list[i][j] = edge_vec_.size()-1;

    // Resize and insert new Omega_ij into big Omega matrix
    Omega.conservativeResize(3*(num_edges_+1), 3*(num_edges_+1));
    for (int i = 0; i < info_udq.rows(); i++) {
        for (int j = 0; j < info_udq.cols(); j++) {
            Omega.insert(3*num_edges_+i, 3*num_edges_+j) = info_pudq(i,j);
        }
    }

    // Recompute the max vertex
    max_vertex_ = std::max(max_vertex_, std::max(i, j));

    // Increment edge count
    num_edges_++;
}

// void PUDQGraph::update_multi_vertex(int agent_id, size_t j, Eigen::Vector4d vertex_pudq) {
//     //Note: either adds or updates existing vertex estimate
//     multi_vertices_[agent_id][j] = vertex_pudq;
// }

// void PUDQGraph::add_multi_edge(size_t i, int agent_id, size_t j, Eigen::Vector4d edge_pudq, Eigen::Matrix3d info_pudq) {
//     printf("Added multi-edge (%ld, %d, %ld) to graph.\n", i, agent_id, j);

//     Edge e;
//     e.z_ij_pudq = edge_pudq;
//     e.Omega_ij_pudq = info_pudq;

//     e.z_ij_eucl = pudq_to_pose(edge_pudq);
//     e.Omega_ij_eucl = info_pudq;

//     //Add inter-agent LC edge to graph
//     multi_edges_[i][agent_id][j] = e;
// }

bool PUDQGraph::edge_exists(size_t i, size_t j) {
    // First make sure vertex i has any edges
    if (adjacency_list.count(i) > 0) {
        // Check for an edge from i to j
        return adjacency_list[i].count(j);
    } else {
        // Vertex i has no edges
        return false;
    }
}

void PUDQGraph::print_graph() {
    
    printf("Pose graph with %ld vertices and %ld edges:", num_vertices_, num_edges_);

    for (auto it0 = edges_.begin(); it0 != edges_.end(); ++it0) {
        std::map<size_t, Edge> edge = it0->second;
        for (auto it1 = edge.begin(); it1 != edge.end(); ++it1) {
            std::string edge_str;
            edge_str.append("(" + std::to_string(it0->first) + ", " + std::to_string(it1->first) + "): ");

            double p[3];
            for (size_t i = 0; i < 3; i++) {
                p[i] = it1->second.delta_pose(i);
            }
            edge_str.append("[" + std::to_string(p[0]) + ", " + std::to_string(p[1]) + ", " + std::to_string(p[2]) + "]\n");

            std::cout << edge_str << it1->second.information_pudq << std::endl;
        }
    }
}

