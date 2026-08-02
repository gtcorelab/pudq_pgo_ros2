#ifndef PUDQ_LIB__PUDQ_LIB_HPP_
#define PUDQ_LIB__PUDQ_LIB_HPP_

#include "pudq_lib/visibility_control.h"

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/color_rgba.hpp>

#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SparseCore>

namespace pudq_lib {
    // Row-major sparse matrix type
    typedef Eigen::SparseMatrix<double, Eigen::RowMajor> SparseMatrix;

    void unit_test();

    void write_sparse_block(Eigen::SparseMatrix<double, Eigen::RowMajor> *M, Eigen::MatrixXd &m, int M_i, int M_j);

    std::string pudq_to_string(Eigen::Vector4d x);

    double sinc(double x);

    Eigen::Matrix4d Q_L(const Eigen::Vector4d &x);
    Eigen::Matrix4d Q_LLM(const Eigen::Vector4d &x);

    Eigen::Vector4d pose_to_pudq(const Eigen::Vector3d &p);
    Eigen::Vector3d pudq_to_pose(const Eigen::Vector4d &q);
    Eigen::Matrix4d P_x(const Eigen::Vector4d &x);
    SparseMatrix P_X_N(const std::vector<Eigen::Vector4d> &X);

    Eigen::Vector4d pudq_identity();
    Eigen::Vector4d random_pudq();
    Eigen::Vector4d pudq_normalize(Eigen::Vector4d q);
    Eigen::Vector4d pudq_inv(Eigen::Vector4d q);
    Eigen::Vector4d pudq_mul(Eigen::Vector4d x, Eigen::Vector4d y);
    Eigen::Vector4d pudq_compose(Eigen::Vector4d x, Eigen::Vector4d y);
    // int pudq_sign(Eigen::Vector4d q);
    double get_phi_atan2(double sin_phi, double cos_phi);

    Eigen::Vector4d Exp_1(Eigen::Vector3d x_t);
    Eigen::Vector4d Exp_x(Eigen::Vector4d x, Eigen::Vector4d y_t);

    Eigen::Vector3d Log_1(Eigen::Vector4d q);
    Eigen::Vector4d Log_x(Eigen::Vector4d x, Eigen::Vector4d y_m);

    Eigen::Matrix3d pudq_info_to_eucl(Eigen::Matrix3d pudq_info_mat, double alpha);
    Eigen::Matrix3d eucl_info_to_pudq(Eigen::Matrix3d eucl_info_mat, double alpha);
    Eigen::Matrix3d pudq_cov_to_eucl(Eigen::Matrix3d pudq_cov, double alpha);
    Eigen::Matrix3d eucl_cov_to_pudq(Eigen::Matrix3d eucl_cov, double alpha);
    Eigen::MatrixXd eucl_cov_2d_to_3d(Eigen::Matrix3d eucl_cov_2d);

    Eigen::Matrix2d R_theta(double theta);
    double dist_theta(double theta0, double theta1);
    double quat_to_yaw(geometry_msgs::msg::Quaternion q);
    geometry_msgs::msg::Quaternion yaw_to_quat(double yaw);

    inline std_msgs::msg::ColorRGBA color(double r, double g, double b, double a) {
        std_msgs::msg::ColorRGBA c; c.r=r; c.g=g; c.b=b; c.a=a; return c; 
    }

    inline geometry_msgs::msg::Vector3 vector3(double x, double y, double z) {
        geometry_msgs::msg::Vector3 v; v.x=x; v.y=y; v.z=z; return v; 
    }

    inline geometry_msgs::msg::Point point3d(double x, double y, double z) {
        geometry_msgs::msg::Point p; p.x=x; p.y=y; p.z=z; return p; 
    }
}  // namespace pudq_lib

#endif  // PUDQ_LIB__PUDQ_LIB_HPP_
