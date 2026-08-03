#ifndef PUDQ_GRAPH_HPP
#define PUDQ_GRAPH_HPP

#include <map>
#include <vector>

//ROS
#include <rclcpp/rclcpp.hpp>

//Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SparseCore>

#include <pudq_lib/pudq_lib.hpp>

class PUDQGraph {
public:
    // Edge data type
    struct Edge {
        size_t i, j;
        Eigen::Vector4d z_ij_pudq;
        Eigen::Vector3d z_ij_eucl;
        Eigen::Matrix3d Omega_ij_pudq;
        Eigen::Matrix3d Omega_ij_eucl;
    };

    // Vertices and Edges from inter-agent loop closures
    // std::map<size_t, std::map<size_t, Eigen::Vector4d>> multi_vertices_;
    // std::map<size_t, std::map<size_t, std::map<size_t, PUDQGraph::Edge>>> multi_edges_;

    PUDQGraph();

    size_t get_num_vertices() const;
    size_t get_num_edges() const;
    Eigen::Vector4d get_vertex(size_t i) const;

    // Todo: Rename to vertices_pudq
    const std::vector<Eigen::Vector4d>& get_vertices() const;
    const std::vector<Eigen::Vector4d>& get_vertices_true() const;

    const std::vector<Eigen::Vector3d>& get_vertices_eucl() const;
    const std::vector<Eigen::Vector3d>& get_vertices_true_eucl() const;

    const std::map<size_t, std::map<size_t, size_t>>& get_adjacency() const;
    const std::vector<Edge>& get_edges() const;
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& get_Omega() const;

    void clear();
    void init_vertices(size_t N);
    void update_eucl_vertices();
    void odom_init();

    void set_vertex(size_t i, Eigen::Vector4d vertex_pudq);
    void add_vertex(Eigen::Vector4d vertex_pudq);
    void add_vertex_true(Eigen::Vector4d vertex_pudq);

    bool edge_exists(size_t i, size_t j);
    void add_edge(size_t edge_i, size_t edge_j, Eigen::Vector4d edge_pudq, Eigen::Matrix3d info_pudq);

    void construct_omega();

    // void update_multi_vertex(int agent_id, size_t j, Eigen::Vector4d vertex_pudq);
    // void add_multi_edge(size_t i, int agent_id, size_t j, Eigen::Vector4d edge_pudq, Eigen::Matrix3d info_pudq);
    void print_graph();

private:
    size_t max_vertex_;
    size_t num_vertices_;
    size_t num_edges_;

    std::vector<Eigen::Vector3d> vertices_eucl_;
    std::vector<Eigen::Vector4d> vertices_pudq_;

    std::vector<Eigen::Vector3d> vertices_true_eucl_;
    std::vector<Eigen::Vector4d> vertices_true_pudq_;

    std::map<size_t, std::map<size_t, size_t>> adjacency_list_;
    std::vector<Edge> edge_vec_;

    Eigen::SparseMatrix<double, Eigen::RowMajor> Omega_;
};

#endif

