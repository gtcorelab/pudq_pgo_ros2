#ifndef PUDQ_PGO_LIB_HPP
#define PUDQ_PGO_LIB_HPP

#include <rclcpp/rclcpp.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/SparseCore>

#include <pthread.h>

#include <pudq_lib/pudq_lib.hpp>
// #include <pudq_msgs/msg/pudq_graph.hpp>

#include <pudq_pgo_lib/PUDQGraph.hpp>

namespace pudq_pgo_lib {

    // Row-major sparse matrix type
    typedef Eigen::SparseMatrix<double, Eigen::RowMajor> SparseMatrix;

    struct J_Edge_data {
        Eigen::Vector4d x_i, x_j;
        PUDQGraph::Edge edge_ij;
    };

    struct J_Edge_ij {
        size_t i, j;
        Eigen::Vector3d eij;
        Eigen::MatrixXd Aij, Bij;
    };

    void *J_ij(void *data);
    std::tuple<std::vector<Eigen::Vector4d>, SparseMatrix, double, Eigen::VectorXd, SparseMatrix> rgn_gradhess(PUDQGraph &pg);

    void pgo_test();
    double f_1(double x);

    double F_G_pudq(const PUDQGraph &G);

    Eigen::Vector3d e_ij(Eigen::Vector4d z_ij, Eigen::Vector4d x_i, Eigen::Vector4d x_j);
    Eigen::MatrixXd A_ij(Eigen::Vector4d z_ij, Eigen::Vector4d x_i, Eigen::Vector4d x_j);
    Eigen::MatrixXd B_ij(Eigen::Vector4d z_ij, Eigen::Vector4d x_i, Eigen::Vector4d x_j);

    // Eigen::VectorXd egrad(PUDQGraph *G);
    // Eigen::MatrixXd gnhess(PUDQGraph *G);

    void optimize_rgn(PUDQGraph &G, double tol, int max_iter);
    // void optimize_rgn_fast(PUDQGraph *G, double tol, int max_iter);
}

#endif

