#include "pudq_pgo_lib/PUDQGraph.hpp"

#include <iostream>

using namespace pudq_lib;

PUDQGraph::PUDQGraph() {
    num_vertices_ = 0;
    num_edges_ = 0;
}

size_t PUDQGraph::get_num_vertices() const {
    return num_vertices_;
}

size_t PUDQGraph::get_num_edges() const {
    return num_edges_;
}

Eigen::Vector4d PUDQGraph::get_vertex(size_t i) const {
    if (i < vertices_pudq_.size()) {
        return vertices_pudq_[i];
    } else {
        fprintf(stderr, "Error: Vertex %ld out of range!\n", i);

        return pudq_lib::pudq_identity();
    }
}

const std::vector<Eigen::Vector4d>& PUDQGraph::get_vertices() const {
    return vertices_pudq_;
}

const std::vector<Eigen::Vector4d>& PUDQGraph::get_vertices_true() const {
    return vertices_true_pudq_;
}

const std::vector<Eigen::Vector3d>& PUDQGraph::get_vertices_eucl() const {
    return vertices_eucl_;
}

const std::vector<Eigen::Vector3d>& PUDQGraph::get_vertices_true_eucl() const {
    return vertices_true_eucl_;
}

const std::map<size_t, std::map<size_t, size_t>>& PUDQGraph::get_adjacency() const {
    return adjacency_list_;
}

const std::vector<PUDQGraph::Edge>& PUDQGraph::get_edges() const {
    return edge_vec_;
}

const Eigen::SparseMatrix<double, Eigen::RowMajor>& PUDQGraph::get_Omega() const {
    return Omega_;
}

void PUDQGraph::clear() {
    // Clear all vertices and edges
    vertices_pudq_.clear();
    vertices_true_pudq_.clear();
    vertices_eucl_.clear();
    vertices_true_eucl_.clear();
    edge_vec_.clear();
    adjacency_list_.clear();

    num_vertices_ = 0;
    num_edges_ = 0;
}

// This initializes all vertices to identity
void PUDQGraph::init_vertices(size_t N) {
    //Reserve and initialize all vertices to identity
    vertices_pudq_.reserve(N);
    vertices_true_pudq_.reserve(N);
    vertices_eucl_.clear();
    vertices_true_eucl_.clear();

    // Initalize vertices to identity (leave true vertices unset)
    Eigen::Vector4d p_id = pudq_lib::pudq_identity();
    Eigen::Vector3d e_id = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < N; i++) {
        vertices_pudq_.push_back(p_id);
        vertices_eucl_.push_back(e_id);
    }

    num_vertices_ = vertices_pudq_.size();
}

void PUDQGraph::update_eucl_vertices() {
    for (size_t i = 0; i < num_vertices_; i++) {
        vertices_eucl_[i] = pudq_to_pose(vertices_pudq_[i]);
    }
}

// This initializes all vertices from odometry measurements
void PUDQGraph::odom_init() {
    // Todo: improve this process (works for now)
    if (vertices_pudq_.size() != max_vertex_ + 1) {
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
        if (adjacency_list_.count(i) > 0 || adjacency_list_.count(j) > 0) {
            if (adjacency_list_[i].count(j) > 0) {
                odom_edge_exists = true;

                // Propogate odom via x_j = x_i comp. z_ij
                Eigen::Vector4d x_i_odom = vertices_pudq_[i];
                Eigen::Vector4d x_j_odom = pudq_lib::pudq_compose(x_i_odom, edge_vec_[adjacency_list_[i][j]].z_ij_pudq);
                set_vertex(j, x_j_odom);
            } else if (adjacency_list_[j].count(i) > 0) {
                // Account for reverse odom edges in some datasets
                odom_edge_exists = true;

                // Propogate odom via x_j = x_i comp. z_ij^(-1)
                Eigen::Vector4d x_i_odom = vertices_pudq_[i];
                Eigen::Vector4d x_j_odom = pudq_lib::pudq_compose(x_i_odom, pudq_lib::pudq_inv(edge_vec_[adjacency_list_[i][j]].z_ij_pudq));
                set_vertex(j, x_j_odom);
            }
        }

        if (!odom_edge_exists) {
            // Don't print this warning for now bc some datasets are just FUBARed
            // fprintf(stderr, "Warning: Missing odom edge from %ld->%ld! Assuming identity.\n", i, j);

            //Propogate odom via x_j = x_i comp. z_ij
            Eigen::Vector4d z_ij_id = pudq_lib::pudq_identity();
            Eigen::Vector4d x_i_odom = vertices_pudq_[i];
            Eigen::Vector4d x_j_odom = pudq_lib::pudq_compose(x_i_odom, z_ij_id);
            set_vertex(j, x_j_odom);
        }
    }

    // Make sure Euclidean vertices always match their PUDQ counterparts
    update_eucl_vertices();
}

void PUDQGraph::set_vertex(size_t i, Eigen::Vector4d vertex_pudq) {
    // Todo:: check for out of bounds
    vertices_pudq_[i] = vertex_pudq;
    vertices_eucl_[i] = pudq_to_pose(vertex_pudq);
}

void set_vertices(std::vector<Eigen::Vector4d> &X) {
    vertices_pudq_ = X;
    update_eucl_vertices();
}

void PUDQGraph::add_vertex(Eigen::Vector4d vertex_pudq) {
    // Add PUDQ and equivalent Euclidean vertices to the graph
    vertices_pudq_.push_back(vertex_pudq);
    vertices_eucl_.push_back(pudq_to_pose(vertex_pudq));
    num_vertices_++;
}

void PUDQGraph::add_vertex_true(Eigen::Vector4d vertex_pudq) {
    // Add PUDQ and equivalent Euclidean vertices to the graph
    vertices_true_pudq_.push_back(vertex_pudq);
    vertices_true_eucl_.push_back(pudq_to_pose(vertex_pudq));
}

bool PUDQGraph::edge_exists(size_t i, size_t j) {
    // First make sure vertex i has any edges
    if (adjacency_list_.count(i) > 0) {
        // Check for an edge from i to j
        return adjacency_list_[i].count(j);
    } else {
        // Vertex i has no edges
        return false;
    }
}

void PUDQGraph::add_edge(size_t edge_i, size_t edge_j, Eigen::Vector4d edge_pudq, Eigen::Matrix3d info_pudq) {

    //Perform necessary checks to make sure this edge can exist
    // if (!(i < vertices_pudq_.size() && j < vertices_pudq_.size())) {
    //     printf("PUDQGraph add_edge: Edge (%ld,%ld) out of range!\n", i, j);
    //     return;
    // }

    if (edge_i == edge_j) {
        printf("PUDQGraph add_edge: No self-loops!\n");
        return;
    }

    // if (edge_exists(i, j)) {
    //     printf("PUDQGraph add_edge: Edge (%ld,%ld) already exists!\n", i, j);
    //     return;
    // }

    //Populate edge struct
    Edge e;
    e.i = edge_i;
    e.j = edge_j;
    e.z_ij_pudq = edge_pudq;
    e.Omega_ij_pudq = info_pudq;

    e.z_ij_eucl = pudq_to_pose(edge_pudq);

    // Todo::Convert info mat from PUDQ to SE(2) - NEED TRUE ERROR FOR THIS
    e.Omega_ij_eucl = info_pudq;

    // Add edge to edge vector
    edge_vec_.push_back(e);

    // Add edge to adjacency list (value is index of edge vector)
    adjacency_list_[edge_i][edge_j] = edge_vec_.size()-1;

    // Resize and insert new Omega_ij into big Omega matrix
    Omega_.conservativeResize(3*(num_edges_+1), 3*(num_edges_+1));
    for (int i = 0; i < info_pudq.rows(); i++) {
        for (int j = 0; j < info_pudq.cols(); j++) {
            Omega_.insert(3*num_edges_+i, 3*num_edges_+j) = info_pudq(i,j);
        }
    }

    // Recompute the max vertex
    max_vertex_ = std::max(max_vertex_, std::max(edge_i, edge_j));

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

void PUDQGraph::construct_omega() {
    //Constructs big omega matrix
    Omega_ = Eigen::SparseMatrix<double, Eigen::RowMajor>(3*num_edges_, 3*num_edges_);

    for (size_t ij = 0; ij < num_edges_; ij++) {
        for (int i = 0; i < edge_vec_[ij].Omega_ij_pudq.rows(); i++) {
            for (int j = 0; j < edge_vec_[ij].Omega_ij_pudq.cols(); j++) {
                Omega_.insert(3*ij+i, 3*ij+j) = edge_vec_[ij].Omega_ij_pudq(i,j);
            }
        }
    }
}

void PUDQGraph::print_graph() {
    
    std::cout << "Pose graph with " << num_vertices_ << " vertices and " << num_edges_ << " edges:" << std::endl;

    //Print all vertices
    std::cout << "Vertices:" << std::endl;
    for (size_t i = 0; i < vertices_pudq_.size(); i++) {
        // Eigen::Vector4d x_i = 

        std::cout << "(" << i << "): " << vertices_pudq_[i].transpose() << std::endl;
    }

    std::cout << std::endl << "Edges:" << std::endl;

    //Print all edges
    for (auto it0 = adjacency_list_.begin(); it0 != adjacency_list_.end(); ++it0) {
        std::map<size_t, size_t> edge = it0->second;
        for (auto it1 = edge.begin(); it1 != edge.end(); ++it1) {
            std::string edge_str;
            edge_str.append("(" + std::to_string(it0->first) + "->" + std::to_string(it1->first) + "): ");

            // double p[3];
            // for (int i=0; i<3; i++) {
            //     p[i] = it1->second.delta_pose(i);
            // }
            // edge_str.append("[" + std::to_string(p[0]) + ", " + std::to_string(p[1]) + ", " + std::to_string(p[2]) + "]\n");

            std::cout << edge_str << edge_vec_[it1->second].z_ij_pudq.transpose() << std::endl;
        }
    }
}

