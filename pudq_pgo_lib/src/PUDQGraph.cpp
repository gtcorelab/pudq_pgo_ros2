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

void PUDQGraph::set_vertex(size_t i, Eigen::Vector4d vertex_pudq) {
    //Todo:: check for out of bounds
    vertices_pudq_[i] = vertex_pudq;
    vertices_[i] = pudq_to_pose(vertex_pudq);
}

void PUDQGraph::clear() {
    vertices_.clear();
    vertices_pudq_.clear();

    edges_.clear();
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
    if (!(i < vertices_pudq_.size() && j < vertices_pudq_.size())) {
        printf("PUDQGraph add_edge: Edge (%d,%d) out of range!", i, j);
        return;
    }

    if (i == j) {
        printf("PUDQGraph add_edge: No self-loops!");
        return;
    }

    if (edge_exists(i, j)) {
        printf("PUDQGraph add_edge: Edge (%d,%d) already exists!", i, j);
        return;
    }

    //Populate edge struct
    Edge e;
    e.delta_pose_pudq = edge_pudq;
    e.information_pudq = info_pudq;

    //Todo::Convert info mat from PUDQ to SE(2) - NEED TRUE ERROR FOR THIS
    e.delta_pose = pudq_to_pose(edge_pudq);
    e.information = info_pudq;

    //Add edge to graph
    edges_[i][j] = e;
    num_edges_++;
}

void PUDQGraph::update_multi_vertex(int agent_id, size_t j, Eigen::Vector4d vertex_pudq) {
    //Note: either adds or updates existing vertex estimate
    multi_vertices_[agent_id][j] = vertex_pudq;
}

void PUDQGraph::add_multi_edge(size_t i, int agent_id, size_t j, Eigen::Vector4d edge_pudq, Eigen::Matrix3d info_pudq) {
    printf("Added multi-edge (%d, %d, %d) to graph.", i, agent_id, j);

    Edge e;
    e.delta_pose_pudq = edge_pudq;
    e.information_pudq = info_pudq;

    e.delta_pose = pudq_to_pose(edge_pudq);
    e.information = info_pudq;

    //Add inter-agent LC edge to graph
    multi_edges_[i][agent_id][j] = e;
}

bool PUDQGraph::edge_exists(size_t i, size_t j) {
    return edges_[i].count(j);
}

void PUDQGraph::print_graph() {
    
    printf("Pose graph with %d vertices and %d edges:", num_vertices_, num_edges_);

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

