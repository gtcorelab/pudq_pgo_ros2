#ifndef PUDQ_PGO_LIB_HPP
#define PUDQ_PGO_LIB_HPP

#include <rclcpp/rclcpp.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <pudq_lib/pudq_lib.hpp>
// #include <pudq_msgs/msg/pudq_graph.hpp>

#include <pudq_pgo_lib/PUDQGraph.hpp>

namespace pudq_pgo_lib {

    void pgo_test();

    double grad_term_1(double x);
    double f_1(double x);

    double F_G_pudq(PUDQGraph *G);

    Eigen::Vector3d e_ij(Eigen::Vector4d z_ij, Eigen::Vector4d x_i, Eigen::Vector4d x_j);
    Eigen::MatrixXd A_ij(Eigen::Vector4d z_ij, Eigen::Vector4d x_i, Eigen::Vector4d x_j);
    Eigen::MatrixXd B_ij(Eigen::Vector4d z_ij, Eigen::Vector4d x_i, Eigen::Vector4d x_j);
    Eigen::VectorXd egrad(PUDQGraph *G);
    Eigen::MatrixXd gnhess(PUDQGraph *G);

    void optimize_rgn(PUDQGraph *G, double tol, int max_iter);
    void optimize_rgn_fast(PUDQGraph *G, double tol, int max_iter);
}

#endif

