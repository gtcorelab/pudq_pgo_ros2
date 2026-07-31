#ifndef PUDQ_GRAPH_HPP
#define PUDQ_GRAPH_HPP

#include <map>
#include <vector>

//ROS
#include <rclcpp/rclcpp.hpp>

//Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <pudq_lib/pudq_lib.hpp>

class PUDQGraph {
private:
    size_t num_vertices_;
    size_t num_edges_;

public:
    //Todo: Make these private
    std::vector<Eigen::Vector3d> vertices_;
    std::vector<Eigen::Vector4d> vertices_pudq_;

    std::vector<Eigen::Vector3d> vertices_true_;
    std::vector<Eigen::Vector4d> vertices_true_pudq_;

    struct Edge {
        size_t i, j;
        Eigen::Vector4d z_ij_pudq;
        Eigen::Vector3d z_ij_eucl;
        Eigen::Matrix3d Omega_ij_pudq;
        Eigen::Matrix3d Omega_ij_eucl;
    };
    // std::map<size_t, std::map<size_t, Edge>> edges_;

    std::map<size_t, std::map<size_t, size_t>> adjacency_list;
    std::vector<Edge> edge_vec_;

    Eigen::SparseMatrix<double, Eigen::RowMajor> Omega;

    //Vertices and Edges from inter-agent loop closures
    // std::map<size_t, std::map<size_t, Eigen::Vector4d>> multi_vertices_;
    // std::map<size_t, std::map<size_t, std::map<size_t, PUDQGraph::Edge>>> multi_edges_;

    PUDQGraph();
    
    size_t get_num_vertices();
    size_t get_num_edges();
    Eigen::VectorXd get_X();
    
    void clear();
    void odom_init();
    void construct_omega();
    void set_vertex(size_t i, Eigen::Vector4d vertex_pudq);
    void add_vertex(Eigen::Vector4d vertex_pudq);
    void add_vertex_true(Eigen::Vector4d vertex_pudq);
    void add_edge(size_t i, size_t j, Eigen::Vector4d edge_pudq, Eigen::Matrix3d info_pudq);
    // void update_multi_vertex(int agent_id, size_t j, Eigen::Vector4d vertex_pudq);
    // void add_multi_edge(size_t i, int agent_id, size_t j, Eigen::Vector4d edge_pudq, Eigen::Matrix3d info_pudq);
    void print_graph();
    bool edge_exists(size_t i, size_t j);
};

#endif

