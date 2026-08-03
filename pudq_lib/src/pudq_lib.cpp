#include "pudq_lib/pudq_lib.hpp"

#include <iostream>

namespace pudq_lib {

    void unit_test() {
        srand (static_cast <unsigned> (time(0)));

        Eigen::Vector4d id = pudq_identity();

        Eigen::Vector3d p_x;
        p_x(0) = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/10.0));
        p_x(1) = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/10.0));
        p_x(2) = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/10*M_PI));
        Eigen::Vector4d x = pose_to_pudq(p_x);

        Eigen::Vector3d p_y;
        p_y(0) = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/10.0));
        p_y(1) = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/10.0));
        p_y(2) = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/10*M_PI));
        Eigen::Vector4d y = pudq_mul(id, pose_to_pudq(p_y));

        Eigen::Vector3d log_1_x = Lie_Log_1(x);
        Eigen::Vector4d exp_log_1_x = Lie_Exp_1(log_1_x);

        Eigen::Vector4d log_x_y = Log_x(x, y);
        Eigen::Vector4d px_log_x_y = P_x(x) * log_x_y;

        Eigen::Vector4d exp_log_x_y = Exp_x(x, log_x_y);

        Eigen::Vector4d x_inv_x_l = pudq_mul(pudq_inv(x), x);
        Eigen::Vector4d x_inv_x_r = pudq_mul(x, pudq_inv(x));
        
        //Make sure Exp_1(Log_1(x)) = x
        if ((x - exp_log_1_x).norm() < 1e-5) {
            std::cout << "Passed Exp_1(Log_1(x)) Test" << std::endl;
        } else {
            std::cout << "Failed Exp_1(Log_1(x)) Test" << std::endl;
        }

        //Make sure P_x(x)*v_x = v_x
        if ((log_x_y - px_log_x_y).norm() < 1e-5) {
            std::cout << "Passed P_x(v_x) Test"<< std::endl;
        } else {
            std::cout << "Failed P_x(v_x) Test"<< std::endl;
        }

        if ((y - exp_log_x_y).norm() < 1e-5) {
            std::cout << "Passed Exp_x(Log_x(y)) Test" << std::endl;
        } else {
            std::cout << "Failed Exp_x(Log_x(y)) Test" << std::endl;
        }

        if ((x_inv_x_l - pudq_identity()).norm() < 1e-5 && (x_inv_x_r - pudq_identity()).norm() < 1e-5) {
            std::cout << "Passed x_inv*x Test" << std::endl;
        } else {
            std::cout << "Failed x_inv*x Test" << std::endl;
        }

        Eigen::Vector3d pose_i, pose_j, pose_ij;
        pose_i << 1.87998, 0, 0.819981;
        pose_j << 1.87998, 0, 1.01999;

        Eigen::Vector4d x_i = pose_to_pudq(pose_i);
        Eigen::Vector4d x_j = pose_to_pudq(pose_j);

        Eigen::Vector4d z_ij = pudq_compose(pudq_inv(x_i), x_j);
        pose_ij = pudq_to_pose(z_ij);

        // ROS_WARN_STREAM("x_i: [" << pose_i(0) << ", "<< pose_i(1) << ", "<< pose_i(2) << "]");
        // ROS_WARN_STREAM("x_j: [" << pose_j(0) << ", "<< pose_j(1) << ", "<< pose_j(2) << "]");
        // ROS_WARN_STREAM("delta_pose: [" << pose_ij(0) << ", "<< pose_ij(1) << ", "<< pose_ij(2) << "]");

        //Info mat check
        double phi = 0.1;
        Eigen::Matrix3d eucl_info_mat, pudq_info_check, eucl_info_check;
        eucl_info_mat << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9;
        eucl_info_mat = eucl_info_mat.transpose()*eucl_info_mat + 3*Eigen::Matrix3d::Identity();

        pudq_info_check = eucl_info_to_pudq(eucl_info_mat, phi);
        eucl_info_check = pudq_info_to_eucl(pudq_info_check, phi);

        Eigen::Matrix3d eucl_cov, pudq_cov, pudq_cov_check, eucl_cov_check, pudq_cov_inv_check, eucl_cov_inv_check;
        eucl_cov = eucl_info_mat.inverse();
        pudq_cov_check = eucl_cov_to_pudq(eucl_cov, phi);
        eucl_cov_check = pudq_cov_to_eucl(pudq_cov_check, phi);
        pudq_cov_inv_check = pudq_cov_check.inverse();
        eucl_cov_inv_check = eucl_cov_check.inverse();

        double eucl_info_diff = (eucl_info_mat-eucl_info_check).norm();
        double pudq_info_diff = (pudq_info_check-pudq_cov_inv_check).norm();
        double eucl_cov_diff =  (eucl_cov-eucl_cov_check).norm();

        std::cout << "Original info:" << std::endl << eucl_info_mat << std::endl << std::endl;
        std::cout << "PUDQ info:" << std::endl << pudq_info_check << std::endl << std::endl;
        std::cout << "Eucl info:" << std::endl << eucl_info_check << std::endl << std::endl;

        std::cout << "Original cov:" << std::endl << eucl_cov << std::endl << std::endl;
        std::cout << "PUDQ cov:" << std::endl << pudq_cov_check << std::endl << std::endl;
        std::cout << "Eucl cov:" << std::endl << eucl_cov_check << std::endl << std::endl;
        std::cout << "PUDQ cov inv:" << std::endl << pudq_cov_inv_check << std::endl << std::endl;
        std::cout << "Eucl cov inv:" << std::endl << eucl_cov_inv_check << std::endl << std::endl;

        std::cout << "Eucl info diff: " << eucl_info_diff << std::endl;
        std::cout << "PUDQ info diff: " << pudq_info_diff << std::endl;
        std::cout << "Eucl cov diff: " << eucl_cov_diff << std::endl;

        Eigen::MatrixXd eucl_cov_3d = eucl_cov_2d_to_3d(eucl_cov);
        std::cout << "Eucl cov 3d:" << std::endl << eucl_cov_3d << std::endl << std::endl;
    }

    // Copies a dense matrix into a big sparse one
    void write_sparse_block(Eigen::SparseMatrix<double, Eigen::RowMajor> *M, Eigen::MatrixXd &m, int M_i, int M_j) {
        for (int i = 0; i < m.rows(); i++) {
            for (int j = 0; j < m.cols(); j++) {
                M->insert(M_i+i, M_j+j) = m(i,j);
            }
        }
    }

    std::string pudq_to_string(Eigen::Vector4d x) {
        std::string s;
        s.append("[").append(std::to_string(x(0))).append(",").append(std::to_string(x(1))).append(",").append(std::to_string(x(2))).append(",").append(std::to_string(x(3))).append("]");
        return s;
    }

    double sinc(double x) {
        return x == 0 ? 1.0 : sin(x)/x;
    }

    Eigen::Matrix4d Q_L(const Eigen::Vector4d &x) {
        Eigen::Matrix4d Q;
        Q << x(0), -x(1), 0.0,   0.0,
             x(1),  x(0), 0.0,   0.0,
             x(2),  x(3), x(0), -x(1),
             x(3), -x(2), x(1),  x(0);
        return Q;
    }

    Eigen::Matrix4d Q_LLM(const Eigen::Vector4d &x) {
        Eigen::Matrix4d Q;
        Q << x(0),  x(1),  0.0,   0.0,
             -x(1), x(0),  0.0,   0.0,
             -x(2), -x(3), x(0),  x(1),
             -x(3), x(2),  -x(1), x(0);
        return Q;
    }

    Eigen::Matrix4d P_x(const Eigen::Vector4d &x) {
        Eigen::Matrix4d P = Eigen::Matrix4d::Identity();
        P.topLeftCorner(2,2) = Eigen::Matrix2d::Identity() - x.segment(0,2)*x.segment(0,2).transpose();
        return P;
    }

    //Todo: Figure out upper triangular storage for symmetric matrices
    SparseMatrix P_X_N(const std::vector<Eigen::Vector4d> &X) {

        size_t N = X.size();
        SparseMatrix *P = new SparseMatrix(4*N, 4*N);

        for (size_t i = 0; i < N; i++) {
            Eigen::MatrixXd P_i = P_x(X[i]);
            write_sparse_block(P, P_i, 4*i, 4*i);
        }

        // Todo: Fix this absolute BS
        SparseMatrix P_ret = *P;
        delete P;

        return P_ret;
    }

    Eigen::Vector4d pose_to_pudq(const Eigen::Vector3d &p) {
        Eigen::Vector4d q;

        //Extract rotation angle and compute half-angle sin and cos
        double theta = p(2);
        q(0) = cos(0.5*theta);
        q(1) = sin(0.5*theta);

        //Apply rotation by theta/2 to construct the last 2 elements
        Eigen::Matrix2d Q_r;
        Q_r << q(0), q(1), -q(1), q(0);
        q.segment(2,2) = 0.5 * Q_r * p.segment(0,2);

        return pudq_normalize(q);
    }

    Eigen::Vector3d pudq_to_pose(const Eigen::Vector4d &x) {
        Eigen::Vector3d p;
        
        Eigen::Matrix2d Q_r;
        Q_r << x(0), x(1), -x(1), x(0);

        p.segment(0,2) = 2.0 * Q_r.transpose() * x.segment(2,2);
        p(2) = 2.0*std::atan2(x(1), x(0));

        return p;
    }

    Eigen::Vector4d pudq_identity() {
        Eigen::Vector4d id;
        id << 1.0, 0.0, 0.0, 0.0;
        return id;
    }

    Eigen::Vector4d random_pudq() {
        Eigen::Vector4d x;
        for (int i=0; i < 4; i++) {
            x(i) = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/10.0));
        }
        return pudq_normalize(x);
    }

    Eigen::Vector4d pudq_normalize(Eigen::Vector4d q) {
        Eigen::Vector4d p = q;
        p.segment(0,2) /= p.segment(0,2).norm();
        return p;
    }

    Eigen::Vector4d pudq_inv(Eigen::Vector4d q) {
        Eigen::Vector4d q_inv;
        q_inv << q(0), -q(1), -q(2), -q(3);
        return q_inv;
    }

    Eigen::Vector4d pudq_mul(Eigen::Vector4d x, Eigen::Vector4d y) {
        Eigen::Vector4d q = Q_L(x) * y;
        return q;
    }

    Eigen::Vector4d pudq_compose(Eigen::Vector4d x, Eigen::Vector4d y) {
        Eigen::Vector4d q = pudq_normalize(pudq_mul(x, y));
        return q;
    }

    double get_phi_atan2(double sin_phi, double cos_phi) {
        double phi = atan2(sin_phi, cos_phi);
        if (phi <= -M_PI_2) {
             phi = phi + M_PI;
        } else if (phi > M_PI_2) {
            phi = phi - M_PI;
        }

        return phi;
    }

    // int pudq_sign(Eigen::Vector4d q) {
    //     if (q(0) < 0) {
    //         //q0 is negative
    //         return -1;
    //     } else if (q(0) > 0) {
    //         //q0 is positive
    //         return 1;
    //     } else {
    //         //q0 = 0
    //         return 0;
    //     }
    // }

    Eigen::Vector4d Lie_Exp_1(Eigen::Vector3d x_t) {
        double gamma = sinc(x_t(0));

        Eigen::Vector4d q;
        q(0) = cos(x_t(0));
        q.segment(1,3) = gamma * x_t;

        return pudq_normalize(q);
    }

    Eigen::Vector4d Exp_x(Eigen::Vector4d x, Eigen::Vector4d y_t) {
        //Project y_t into TxM
        Eigen::Vector4d y_TxM = P_x(x) * y_t;

        Eigen::Matrix2d Q_Lx;
        Q_Lx << x(0), -x(1), x(1), x(0);

        double phi_r = -x(1)*y_TxM(0) + x(0)*y_TxM(1);

        Eigen::Vector2d R_phi;
        R_phi << cos(phi_r), sin(phi_r);

        Eigen::Vector2d exp_yr = Q_Lx*R_phi;

        Eigen::Vector4d exp_x;
        exp_x.head(2) = exp_yr;
        exp_x.tail(2) = x.tail(2) + y_TxM.tail(2);

        // Eigen::Vector4d x_inv_y_t = pudq_mul(pudq_inv(x), y_t);
        // Eigen::Vector4d exp_x = pudq_mul(x, Exp_1(x_inv_y_t.segment(1,3)));

        return exp_x;
    }

    std::vector<Eigen::Vector4d> Exp_X_N(std::vector<Eigen::Vector4d> &X, Eigen::VectorXd &Y_t) {
        std::vector<Eigen::Vector4d> exp_x_n;
        for (size_t i = 0; i < X.size(); i++) {
            exp_x_n.push_back(Exp_x(X[i], Y_t.segment(4*i, 4)));
        }

        return exp_x_n;
    }

    Eigen::Vector3d Lie_Log_1(Eigen::Vector4d r) {
        double phi = get_phi_atan2(r(1), r(0));
        double gamma = sinc(phi);
        Eigen::Vector3d x_t = r.segment(1,3)/gamma;

        return x_t;
    }

    Eigen::Vector4d Log_x(Eigen::Vector4d x, Eigen::Vector4d y_m) {

        Eigen::Matrix2d Q_Lx, Q_LMx;
        Q_Lx << x(0), -x(1), x(1), x(0);
        Q_LMx << x(0), x(1), -x(1), x(0);

        Eigen::Vector2d yr_id = Q_LMx*y_m.tail(2);
        double phi_y = get_phi_atan2(yr_id(1), yr_id(0));

        Eigen::Vector2d pad_log_yr;
        pad_log_yr << 0, phi_y;
        Eigen::Vector2d log_yr = Q_Lx*pad_log_yr;

        Eigen::Vector4d log_x;
        log_x.head(2) = log_yr;
        log_x.tail(2) = y_m.tail(2) - x.tail(2);

        return log_x;
    }

    Eigen::Matrix3d pudq_info_to_eucl(Eigen::Matrix3d pudq_info_mat, double alpha) {
        double beta = cos(alpha)/sinc(alpha);

        Eigen::Matrix3d A, B;
        A << 1.0, 0.0, 0.0, 0.0, beta, alpha, 0.0, -alpha, beta;
        B << 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;

        Eigen::Matrix3d M = 0.5*A*B;

        Eigen::Matrix3d eucl_info_mat = M.transpose() * pudq_info_mat * M;
        return eucl_info_mat;
    }

    Eigen::Matrix3d eucl_info_to_pudq(Eigen::Matrix3d eucl_info_mat, double alpha) {
        double beta = cos(alpha)/sinc(alpha);

        Eigen::Matrix3d A, B;
        A << 1.0, 0.0, 0.0, 0.0, beta, alpha, 0.0, -alpha, beta;
        B << 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;

        Eigen::Matrix3d M = 0.5*A*B;
        Eigen::Matrix3d M_inv = M.inverse();

        Eigen::Matrix3d pudq_info_mat = M_inv.transpose() * eucl_info_mat * M_inv;
        return pudq_info_mat;
    }

    Eigen::Matrix3d pudq_cov_to_eucl(Eigen::Matrix3d pudq_cov, double alpha) {
        double beta = cos(alpha)/sinc(alpha);

        Eigen::Matrix3d A, B;
        A << 1.0, 0.0, 0.0, 0.0, beta, alpha, 0.0, -alpha, beta;
        B << 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
        Eigen::Matrix3d M = 0.5*A*B;
        Eigen::Matrix3d M_inv = M.inverse();

        Eigen::Matrix3d eucl_cov = M_inv * pudq_cov * M_inv.transpose();
        return eucl_cov;
    }

    Eigen::Matrix3d eucl_cov_to_pudq(Eigen::Matrix3d eucl_cov, double alpha) {
        double beta = cos(alpha)/sinc(alpha);

        Eigen::Matrix3d A, B;
        A << 1.0, 0.0, 0.0, 0.0, beta, alpha, 0.0, -alpha, beta;
        B << 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
        Eigen::Matrix3d M = 0.5*A*B;

        Eigen::Matrix3d pudq_cov = M * eucl_cov * M.transpose();
        return pudq_cov;
    }

    Eigen::MatrixXd eucl_cov_2d_to_3d(Eigen::Matrix3d eucl_cov_2d) {
        Eigen::MatrixXd eucl_cov_3d = Eigen::MatrixXd::Identity(6,6);

        //XY covariances
        eucl_cov_3d(0,0) = eucl_cov_2d(0,0);
        eucl_cov_3d(0,1) = eucl_cov_2d(0,1);
        eucl_cov_3d(1,0) = eucl_cov_2d(1,0);
        eucl_cov_3d(1,1) = eucl_cov_2d(1,1);

        //Psi covariances
        eucl_cov_3d(0,5) = eucl_cov_2d(0,2);
        eucl_cov_3d(1,5) = eucl_cov_2d(1,2);
        eucl_cov_3d(5,0) = eucl_cov_2d(2,0);
        eucl_cov_3d(5,1) = eucl_cov_2d(2,1);
        eucl_cov_3d(5,5) = eucl_cov_2d(2,2);

        return eucl_cov_3d;
    }

    Eigen::Matrix2d R_theta(double theta) {
        Eigen::Matrix2d R;
        R << cos(theta), -sin(theta), sin(theta), cos(theta);
        return R;
    }

    double dist_theta(double theta0, double theta1) {
        double cos_dist = cos(theta0 - theta1);
        double sin_dist = sin(theta0 - theta1);

        return std::atan2(sin_dist, cos_dist);
    }

    double quat_to_yaw(geometry_msgs::msg::Quaternion q) {
        // yaw (z-axis rotation)
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);

        return yaw;
    }

    geometry_msgs::msg::Quaternion yaw_to_quat(double yaw) {
        geometry_msgs::msg::Quaternion q;
        q.w = cos(yaw/2.0);
        q.x = 0.0;
        q.y = 0.0;
        q.z = sin(yaw/2.0);

        return q;
    }
}  // namespace pudq_lib
