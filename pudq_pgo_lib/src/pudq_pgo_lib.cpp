#include <pudq_pgo_lib/pudq_pgo_lib.hpp>

using namespace pudq_lib;

namespace pudq_pgo_lib { 

    void pgo_test() {
        //Construct test graph
        Eigen::Matrix3d Omega_ij;
        Omega_ij << 0.5554, 0.5941, 0.7355, 0.5941, 1.2578, 1.2955, 0.7355, 1.2955, 1.3914;

        //True poses
        Eigen::Vector3d pose_1, pose_2, pose_3, pose_4;
        pose_1 << 0.0, 0.0, 0.0;
        pose_2 << 1.0, 0.0, M_PI/2;
        pose_3 << 1.0, 2.0, M_PI;
        pose_4 << 0.0, 1.0, -M_PI/2;

        Eigen::Vector4d x_1_true, x_2_true, x_3_true, x_4_true;
        x_1_true = pose_to_pudq(pose_1);
        x_2_true = pose_to_pudq(pose_2);
        x_3_true = pose_to_pudq(pose_3);
        x_4_true = pose_to_pudq(pose_4);

        //Initialize vertices to their true vales
        Eigen::Vector4d x_1, x_2, x_3, x_4;
        // x_1 = pudq_compose(x_1_true, random_pudq());
        // x_2 = pudq_compose(x_2_true, random_pudq());
        // x_3 = pudq_compose(x_3_true, random_pudq());
        // x_4 = pudq_compose(x_4_true, random_pudq());

        x_1 = x_1_true, random_pudq();
        x_2 = x_2_true, random_pudq();
        x_3 = x_3_true, random_pudq();
        x_4 = x_4_true, random_pudq();

        //Noise/perturbations
        Eigen::Vector4d eta_1, eta_2, eta_3, eta_4;
        eta_1 << 0.9626, 0.2708,  0.4392,  0.5215;
        eta_2 << 0.9971, -0.0767, 0.2034,  0.3396;
        eta_3 << 0.8801, 0.4748,  0.0950,  0.2629;
        eta_4 << 0.7973, -0.6036, -0.5813, -0.4460;

        Eigen::Vector4d z_12, z_23, z_34, z_42;
        z_12 = Q_L(eta_1)*pudq_mul(pudq_inv(x_1), x_2);
        z_23 = Q_L(eta_2)*pudq_mul(pudq_inv(x_2), x_3);
        z_34 = Q_L(eta_3)*pudq_mul(pudq_inv(x_3), x_4);
        z_42 = Q_L(eta_4)*pudq_mul(pudq_inv(x_4), x_2);

        //Populate graph object
        PUDQGraph G;
        G.add_vertex(x_1);
        G.add_vertex(x_2);
        G.add_vertex(x_3);
        G.add_vertex(x_4);

        G.add_edge(0, 1, z_12, Omega_ij);
        G.add_edge(1, 2, z_23, Omega_ij);
        G.add_edge(2, 3, z_34, Omega_ij);
        G.add_edge(3, 1, z_42, Omega_ij);

        optimize_rgn(G, 1e-5, 20);
    }

    void *J_ij(void *data) {

        J_Edge_data *edge_data = static_cast<J_Edge_data *>(data);

        x_i = edge_data->edge_ij.x_i;
        x_j = edge_data->edge_ij.x_j;
        z_ij = edge_data->edge_ij.z_ij;

        // Compute residual and Jacobians
        r_ij = pudq_compose(pudq_inv(z_ij), pudq_mul(pudq_inv(x_i), x_j));
    
        // x_i constants
        // mu_i    =  z_ij(1)*x_j(1) + z_ij(2)*x_j(2);
        // omega_i = -z_ij(2)*x_j(1) + z_ij(1)*x_j(2);
        // eta_i   = -z_ij(2)*x_j(1) + z_ij(1)*x_j(2);
        // kappa_i = -x_j(1)*z_ij(1) - x_j(2)*z_ij(2);
        // alpha_1 = -x_j(1)*z_ij(3) - x_j(2)*z_ij(4) + z_ij(1)*x_j(3)+z_ij(2)*x_j(4);
        // beta_1  =  x_j(1)*z_ij(4) - x_j(2)*z_ij(3) - z_ij(2)*x_j(3)+z_ij(1)*x_j(4);
        // xi_1    = -x_j(1)*z_ij(1) + z_ij(2)*x_j(2);
        // zeta_1  = -z_ij(2)*x_j(1) - z_ij(1)*x_j(2);
        // alpha_3 = -x_j(1)*z_ij(4) + x_j(2)*z_ij(3) - x_j(3)*z_ij(2)+z_ij(1)*x_j(4);
        // beta_3  = -x_j(1)*z_ij(3) - x_j(2)*z_ij(4) - x_j(3)*z_ij(1)-z_ij(2)*x_j(4);
        
        // // x_j constants
        // mu_j    =  x_i(1)*z_ij(1) - x_i(2)*z_ij(2);
        // omega_j =  x_i(1)*z_ij(2) + x_i(2)*z_ij(1);
        // eta_j   = -z_ij(2)*x_i(1) - z_ij(1)*x_i(2);
        // kappa_j =  z_ij(1)*x_i(1) - z_ij(2)*x_i(2);
        // alpha_2 = -z_ij(3)*x_i(1) + z_ij(4)*x_i(2) - z_ij(1)*x_i(3) - z_ij(2)*x_i(4);
        // beta_2  = -z_ij(4)*x_i(1) - z_ij(3)*x_i(2) + z_ij(2)*x_i(3) - z_ij(1)*x_i(4);
        
        // // atan2 method for computing phi
        // phi = get_phi_atan2(r_ij(2), r_ij(1));
        // gamma = pudq_sinc(phi);
        
        // // Compute residual
        // eij = [r_ij(2)/gamma; r_ij(3)/gamma; r_ij(4)/gamma];
        
        // // Compute atan2 gradient term
        // f1 = f_1(phi);
        
        // // Initialize Aij jacobian
        // Aij = zeros(3,4);
    
        // // Initialize Bij jacobian
        // Bij = zeros(3,4);

        // // Aij jacobian
        // dphi_dxi0 = eta_i*r_ij(1) - mu_i*r_ij(2);
        // dphi_dxi1 = kappa_i*r_ij(1) - omega_i*r_ij(2);

        // Aij(1,1) = eta_i/gamma   + r_ij(2)*dphi_dxi0*f1;
        // Aij(1,2) = kappa_i/gamma + r_ij(2)*dphi_dxi1*f1;
        // Aij(2,1) = alpha_1/gamma + r_ij(3)*dphi_dxi0*f1;
        // Aij(2,2) = beta_1/gamma  + r_ij(3)*dphi_dxi1*f1;
        // Aij(2,3) = xi_1/gamma;
        // Aij(2,4) = zeta_1/gamma;
        // Aij(3,1) = alpha_3/gamma + r_ij(4)*dphi_dxi0*f1;
        // Aij(3,2) = beta_3/gamma  + r_ij(4)*dphi_dxi1*f1;
        // Aij(3,3) = -zeta_1/gamma;
        // Aij(3,4) = xi_1/gamma;
        
        // // Bij jacobian
        // dphi_dxj0 = eta_j*r_ij(1) - mu_j*r_ij(2);
        // dphi_dxj1 = kappa_j*r_ij(1) - omega_j*r_ij(2);

        // Bij(1,1) = eta_j/gamma + r_ij(2)*dphi_dxj0*f1;
        // Bij(1,2) = kappa_j/gamma + r_ij(2)*dphi_dxj1*f1;
        // Bij(2,1) = alpha_2/gamma + r_ij(3)*dphi_dxj0*f1;
        // Bij(2,2) = beta_2/gamma + r_ij(3)*dphi_dxj1*f1;
        // Bij(2,3) = kappa_j/gamma;
        // Bij(2,4) = -eta_j/gamma;
        // Bij(3,1) = beta_2/gamma + r_ij(4)*dphi_dxj0*f1;
        // Bij(3,2) = -alpha_2/gamma + r_ij(4)*dphi_dxj1*f1;
        // Bij(3,3) = eta_j/gamma;
        // Bij(3,4) = kappa_j/gamma;


        J_Edge_ij *J_blk = new J_Edge_ij;
        J_blk->i = edge_data->edge_ij.i;
        J_blk->j = edge_data->edge_ij.j;
        // J_blk->eij = eij;
        // J_blk->Aij = Aij;
        // J_blk->Bij = Bij;

        // Used to terminate a thread and the return value is passed as a pointer
        pthread_exit(static_cast<void *>(J_blk));
    }

    std::tuple<std::vector<Eigen::VectorXd>, SparseMatrix, double, Eigen::VectorXd, SparseMatrix> rgn_gradhess(PUDQGraph &G) {

        const int N = G.get_num_vertices();
        const int M = G.get_num_edges();

        // std::cout << N << " vertices, " << M << " edges" << std::endl;

        // Get the entire vertex set
        Eigen::VectorXd X = G.get_X();

        // Eigen::VectorXd E_vec = Eigen::VectorXd::Zero(6*M);

        // pthread_t t_id[M];

        // // Allocate data structs to pass to threads
        // J_Edge_data edge_ij_data[M];

        // // Process each edge in a separate thread
        // for (int ij = 0; ij < M; ij++) {

        //     // Todo: check for segfaults (for now assume the graph class handled this properly)
        //     edge_ij_data[ij].x_i = X[pg.edge_vec_[ij].i];
        //     edge_ij_data[ij].x_j = X[pg.edge_vec_[ij].j];
        //     edge_ij_data[ij].edge_ij = pg.edge_vec_[ij];

        //     pthread_create(&t_id[ij], NULL, J_ij, (void *)(&edge_ij_data[ij]));
        // }

        // std::vector<Eigen::Triplet<double>> J_triplet_list;

        // Join all edge threads
        // for (int ij = 0; ij < M; ij++) {
        //     // Get return value ptr from each thread
        //     void *ret;
        //     pthread_join(t_id[ij], &ret);

        //     J_Edge_ij *edge = static_cast<J_Edge_ij *>(ret);

        //     for (int i = 0; i < 6; i++) {
        //         for (int j = 0; j < 8; j++) {
        //             J_triplet_list.push_back(Eigen::Triplet<double>(6*ij+i, 8*edge->i+j, edge->Aij(i,j)));
        //             J_triplet_list.push_back(Eigen::Triplet<double>(6*ij+i, 8*edge->j+j, edge->Bij(i,j)));
        //         }
        //     }

        //     E_vec.segment(6*ij, 6) = edge->eij;

        //     // Free up the memory allocated for this edge
        //     delete edge;
        // }

        SparseMatrix J_mat(6*M, 8*N);
        // J_mat.setFromTriplets(J_triplet_list.begin(), J_triplet_list.end());

        // double F_X = 0.5*E_vec.transpose()*pg.Omega*E_vec;
        double F_X = 0.0;

        Eigen::VectorXd egrad_X = J_mat.transpose()*G.Omega*E_vec;

        SparseMatrix egnhess_X = J_mat.transpose()*G.Omega*J_mat;
        SparseMatrix P_X = P_X_N(X);

        Eigen::VectorXd rgrad_X = P_X*egrad_X;
        SparseMatrix rgnhess_X = P_X*egnhess_X*P_X;

        return std::make_tuple(X, P_X, F_X, rgrad_X, rgnhess_X);
    }

    // void optimize_rgn_fast(PUDQGraph *G, double tol, int max_iter) {

    //     std::cout << "Optimizing over " << G->get_num_vertices() << " vertices, " << G->get_num_edges() << " edges" << std::endl;

    //     //Initialize cost
    //     double F_prev = F_G_pudq(G);

    //     for (int k = 0; k < max_iter; k++) {

    //         //Compute Euclidean gradient and Gauss-Newton Hessian
    //         Eigen::VectorXd egrad_k = Eigen::VectorXd::Zero(4*G->get_num_vertices());
    //         Eigen::MatrixXd gnhess_k = Eigen::MatrixXd::Zero(4*G->get_num_vertices(), 4*G->get_num_vertices());

    //         //Loop through all edges
    //         for (auto it_i = G->edges_.begin(); it_i != G->edges_.end(); ++it_i) {

    //             //Get x_i
    //             Eigen::Vector4d x_i = G->vertices_pudq_[it_i->first];

    //             for (auto it_j = it_i->second.begin(); it_j != it_i->second.end(); ++it_j) {
    //                 //Get x_j and z_ij
    //                 Eigen::Vector4d x_j = G->vertices_pudq_[it_j->first];

    //                 Eigen::Vector4d z_ij = it_j->second.delta_pose_pudq;
    //                 Eigen::Matrix3d Omega_ij = it_j->second.information_pudq;

    //                 //Arcsin method for computing half-angle phi
    //                 Eigen::Vector4d q_ij = pudq_compose(pudq_inv(z_ij), pudq_mul(pudq_inv(x_i), x_j));
    //                 // int s_q0 = pudq_sign(q_ij);
    //                 int s_q0 = 0;
    //                 double phi = asin(s_q0*q_ij(1));
    //                 double gamma = sinc(phi);
    //                 double t_1 = grad_term_1(phi);

    //                 //Compute residual and jacobians
    //                 Eigen::Vector3d eij = Log_1(q_ij);

    //                 //x_i constants
    //                 double mu_i    = x_j(0)*z_ij(0) + x_j(1)*z_ij(1);
    //                 double omega_i = -x_j(0)*z_ij(1) + x_j(1)*z_ij(0);
    //                 double eta_i   = -z_ij(1)*x_j(0)+z_ij(0)*x_j(1);
    //                 double kappa_i = -x_j(0)*z_ij(0)-x_j(1)*z_ij(1);
    //                 double alpha_1 = -x_j(0)*z_ij(2)-x_j(1)*z_ij(3)+x_j(2)*z_ij(0)+x_j(3)*z_ij(1);
    //                 double beta_1  = x_j(0)*z_ij(3)-x_j(1)*z_ij(2)-x_j(2)*z_ij(1)+x_j(3)*z_ij(0);
    //                 double xi_1    = -x_j(0)*z_ij(0)+x_j(1)*z_ij(1);
    //                 double zeta_1  = -z_ij(1)*x_j(0)-z_ij(0)*x_j(1);
    //                 double alpha_3 = -x_j(0)*z_ij(3)+x_j(1)*z_ij(2)-x_j(2)*z_ij(1)+x_j(3)*z_ij(0);
    //                 double beta_3  = -x_j(0)*z_ij(2)-x_j(1)*z_ij(3)-x_j(2)*z_ij(0)-x_j(3)*z_ij(1);

    //                 //x_j constants
    //                 double mu_j    = x_i(0)*z_ij(0) - x_i(1)*z_ij(1);
    //                 double omega_j = x_i(0)*z_ij(1) + x_i(1)*z_ij(0);
    //                 double eta_j   = -z_ij(1)*x_i(0)-z_ij(0)*x_i(1);
    //                 double kappa_j =  z_ij(0)*x_i(0)-z_ij(1)*x_i(1);
    //                 double alpha_2 = -z_ij(2)*x_i(0)+z_ij(3)*x_i(1)-z_ij(0)*x_i(2)-z_ij(1)*x_i(3);
    //                 double beta_2  = -z_ij(3)*x_i(0)-z_ij(2)*x_i(1)+z_ij(1)*x_i(2)-z_ij(0)*x_i(3);

    //                 //Initialize Aij matrix
    //                 Eigen::MatrixXd Aij = Eigen::MatrixXd::Zero(3,4);

    //                 Aij(0,0) = eta_i/q_ij(0);
    //                 Aij(0,1) = kappa_i/q_ij(0);
                    
    //                 Aij(1,0) = s_q0*alpha_1/gamma + eta_i*q_ij(2)*t_1;
    //                 Aij(1,1) = s_q0*beta_1/gamma + kappa_i*q_ij(2)*t_1;
    //                 Aij(1,2) = s_q0*xi_1/gamma;
    //                 Aij(1,3) = s_q0*zeta_1/gamma;
                    
    //                 Aij(2,0) = s_q0*alpha_3/gamma + eta_i*q_ij(3)*t_1;
    //                 Aij(2,1) = s_q0*beta_3/gamma + kappa_i*q_ij(3)*t_1;
    //                 Aij(2,2) = -s_q0*zeta_1/gamma;
    //                 Aij(2,3) = s_q0*xi_1/gamma;

    //                 //Initialize Bij matrix
    //                 Eigen::MatrixXd Bij = Eigen::MatrixXd::Zero(3,4);

    //                 Bij(0,0) = eta_j/q_ij(0);
    //                 Bij(0,1) = kappa_j/q_ij(0);
                    
    //                 Bij(1,0) = s_q0*alpha_2/gamma + eta_j*q_ij(2)*t_1;
    //                 Bij(1,1) = s_q0*beta_2/gamma + kappa_j*q_ij(2)*t_1;
    //                 Bij(1,2) = s_q0*kappa_j/gamma;
    //                 Bij(1,3) = -s_q0*eta_j/gamma;
                    
    //                 Bij(2,0) = s_q0*beta_2/gamma + eta_j*q_ij(3)*t_1;
    //                 Bij(2,1) = -s_q0*alpha_2/gamma + kappa_j*q_ij(3)*t_1;
    //                 Bij(2,2) = s_q0*eta_j/gamma;
    //                 Bij(2,3) = s_q0*kappa_j/gamma;

    //                 //Compute gradient blocks
    //                 Eigen::Vector4d grad_i = Aij.transpose() * Omega_ij * eij;
    //                 Eigen::Vector4d grad_j = Bij.transpose() * Omega_ij * eij;

    //                 //Compute Hessian blocks
    //                 Eigen::Matrix4d Hii_tilde = Aij.transpose() * Omega_ij * Aij;
    //                 Eigen::Matrix4d Hij_tilde = Aij.transpose() * Omega_ij * Bij;
    //                 Eigen::Matrix4d Hji_tilde = Hij_tilde.transpose();
    //                 Eigen::Matrix4d Hjj_tilde = Bij.transpose() * Omega_ij * Bij;

    //                 //Update gradient blocks
    //                 egrad_k.segment(4*it_i->first, 4) += grad_i;
    //                 egrad_k.segment(4*it_j->first, 4) += grad_j;

    //                 gnhess_k.block(4*it_i->first,4*it_i->first,4,4) += Hii_tilde;
    //                 gnhess_k.block(4*it_i->first,4*it_j->first,4,4)  = Hij_tilde;
    //                 gnhess_k.block(4*it_j->first,4*it_i->first,4,4)  = Hji_tilde;
    //                 gnhess_k.block(4*it_j->first,4*it_j->first,4,4) += Hjj_tilde;
    //             }
    //         }

    //         //Todo: Multi-edges here?

    //         //Compute projection matrix
    //         Eigen::MatrixXd P_X = P_X_N(G->get_X());

    //         //Compute Riemannian gradient
    //         Eigen::VectorXd rgrad_k = P_X * egrad_k;

    //         Eigen::MatrixXd rhess_k = P_X * gnhess_k * P_X;
    //         rhess_k.block(0,0,4,4) += Eigen::Matrix4d::Identity();

    //         //Solve LLS system with BCDSVD solver
    //         // Eigen::VectorXd alpha_k = rhess_k.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(-rgrad_k);
    //         Eigen::VectorXd alpha_k = rhess_k.fullPivLu().solve(-rgrad_k);

    //         for (int i = 1; i < G->get_num_vertices(); i++) {
    //             Eigen::Vector4d x_i = G->vertices_pudq_[i];

    //             //RGN
    //             G->set_vertex(i, Exp_x(x_i, alpha_k.segment(4*i, 4)));

    //             //RGD
    //             // G.set_vertex(i, Exp_x(x_i, -rgd_stepsize*rgrad_k.segment(4*i,4)));
    //         }

    //         //Compute the new gradient norm
    //         double gradnorm = (P_X_N(G->get_X()) * egrad(G)).norm();
            
    //         //Compute the new cost
    //         double F_k = F_G_pudq(G);

    //         printf("Iteration %d:\nGrad norm = %f\nCost = %f\n\n", k+1, gradnorm, F_k);

    //         double delta_F = abs(F_k - F_prev);
    //         if (delta_F < tol) {
    //             printf("RGN converged to within tolerance of %f\n", tol);
    //             return;
    //         }

    //         F_prev = F_k;
    //     }

    //     std::cout << "RGN max iterations reached." << std::endl;
    // }

    void optimize_rgn(PUDQGraph &G, double tol, int max_iter) {

        std::cout << G->get_num_vertices() << " vertices, " << G->get_num_edges() << " edges" << std::endl;

        //Initialize cost
        double F_prev = F_G_pudq(G);

        for (int k = 0; k < max_iter; k++) {

            //Compute Euclidean gradient and Gauss-Newton Hessian


            // Eigen::VectorXd egrad_k = egrad(G);
            // Eigen::MatrixXd gnhess_k = gnhess(G);

            std::tie(X_k, P_X, F_k, rgrad_k, rgnhess_k) = udq_pgo_lib::rgn_gradhess(pg);

            //Compute projection matrix
            Eigen::MatrixXd P_X = P_X_N(G.get_X());

            //Compute Riemannian gradient
            Eigen::VectorXd rgrad_k = P_X * egrad_k;

            Eigen::MatrixXd rhess_k = P_X * gnhess_k * P_X;
            rhess_k.block(0,0,4,4) += Eigen::Matrix4d::Identity();

            // Solve LLS system with BCDSVD solver
            // Eigen::VectorXd alpha_k = rhess_k.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(-rgrad_k);
            Eigen::VectorXd alpha_k = rhess_k.fullPivLu().solve(-rgrad_k);

            for (int i = 1; i < G->get_num_vertices(); i++) {
                Eigen::Vector4d x_i = G->vertices_pudq_[i];

                //RGN
                G->set_vertex(i, Exp_x(x_i, alpha_k.segment(4*i, 4)));

                //RGD
                // G.set_vertex(i, Exp_x(x_i, -rgd_stepsize*rgrad_k.segment(4*i,4)));
            }

            //Compute the new gradient norm
            double gradnorm = (P_X_N(G->get_X()) * egrad(G)).norm();
            
            //Compute the new cost
            double F_k = F_G_pudq(G);

            printf("Iteration %d:\nGrad norm = %f\nCost = %f\n\n", k+1, gradnorm, F_k);

            double delta_F = abs(F_k - F_prev);
            if (delta_F < tol) {
                printf("RGN converged to within tolerance of %f\n", tol);
                return;
            }

            F_prev = F_k;
        }

        printf("RGN max iterations reached.\n");
    }

    double F_G_pudq(PUDQGraph &G) {
        double F = 0.0;

        //Loop through all edges
        for (auto it0 = G->edges_.begin(); it0 != G->edges_.end(); ++it0) {

            //Get x_i
            Eigen::Vector4d x_i = G->vertices_pudq_[it0->first];

            for (auto it1 = it0->second.begin(); it1 != it0->second.end(); ++it1) {
                //Get x_j and z_ij
                Eigen::Vector4d x_j = G->vertices_pudq_[it1->first];

                Eigen::Vector4d z_ij = it1->second.delta_pose_pudq;
                Eigen::Matrix3d Omega_ij = it1->second.information_pudq;

                //Compute residual
                Eigen::Vector3d eij = e_ij(z_ij, x_i, x_j);

                F += eij.transpose() * Omega_ij * eij;
            }
        }

        return F / 2.0;
    }

    double grad_term_1(double x) {
        if (x == 0) {
            return 0.0;
        } else {
            return (sin(x)-x*cos(x))/(sin(x)*sin(x)*cos(x));
        }
    }

    double f_1(double x) {
        if (x == 0) {
            return 0.0;
        } else {
            return (sin(x)-x*cos(x))/(sin(x)*sin(x));
        }
    }

    Eigen::Vector3d e_ij(Eigen::Vector4d z_ij, Eigen::Vector4d x_i, Eigen::Vector4d x_j) {
        Eigen::Vector4d q_ij = pudq_compose(pudq_inv(z_ij), pudq_mul(pudq_inv(x_i), x_j));
        return Log_1(q_ij);
    }

    //Jacobians
    Eigen::MatrixXd A_ij(Eigen::Vector4d z_ij, Eigen::Vector4d x_i, Eigen::Vector4d x_j) {
        //x_i constants
        double mu_i    = x_j(0)*z_ij(0) + x_j(1)*z_ij(1);
        double omega_i = -x_j(0)*z_ij(1) + x_j(1)*z_ij(0);
        double eta_i   = -z_ij(1)*x_j(0)+z_ij(0)*x_j(1);
        double kappa_i = -x_j(0)*z_ij(0)-x_j(1)*z_ij(1);
        double alpha_1 = -x_j(0)*z_ij(2)-x_j(1)*z_ij(3)+x_j(2)*z_ij(0)+x_j(3)*z_ij(1);
        double beta_1  = x_j(0)*z_ij(3)-x_j(1)*z_ij(2)-x_j(2)*z_ij(1)+x_j(3)*z_ij(0);
        double xi_1    = -x_j(0)*z_ij(0)+x_j(1)*z_ij(1);
        double zeta_1  = -z_ij(1)*x_j(0)-z_ij(0)*x_j(1);
        double alpha_3 = -x_j(0)*z_ij(3)+x_j(1)*z_ij(2)-x_j(2)*z_ij(1)+x_j(3)*z_ij(0);
        double beta_3  = -x_j(0)*z_ij(2)-x_j(1)*z_ij(3)-x_j(2)*z_ij(0)-x_j(3)*z_ij(1);

        //Atan2 method for computing half-angle phi
        Eigen::Vector4d r_ij = pudq_compose(pudq_inv(z_ij), pudq_mul(pudq_inv(x_i), x_j));

        double phi = get_phi_atan2(r_ij(1), r_ij(0));
        double gamma = sinc(phi);
        double f1 = f_1(phi);

        double dphi_dxi0 = eta_i*r_ij(0) - mu_i*r_ij(1);
        double dphi_dxi1 = kappa_i*r_ij(0) - omega_i*r_ij(1);

        //Initialize Aij matrix
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3,4);
        
        A(0,0) = eta_i/gamma   + r_ij(1)*dphi_dxi0*f1;
        A(0,1) = kappa_i/gamma + r_ij(1)*dphi_dxi1*f1;

        A(1,0) = alpha_1/gamma + r_ij(2)*dphi_dxi0*f1;
        A(1,1) = beta_1/gamma  + r_ij(2)*dphi_dxi1*f1;
        A(1,2) = xi_1/gamma;
        A(1,3) = zeta_1/gamma;

        A(2,0) = alpha_3/gamma + r_ij(3)*dphi_dxi0*f1;
        A(2,1) = beta_3/gamma  + r_ij(3)*dphi_dxi1*f1;
        A(2,2) = -zeta_1/gamma;
        A(2,3) = xi_1/gamma;

        return A;
    }

    Eigen::MatrixXd B_ij(Eigen::Vector4d z_ij, Eigen::Vector4d x_i, Eigen::Vector4d x_j) {
        //x_j constants
        double mu_j    = x_i(0)*z_ij(0) - x_i(1)*z_ij(1);
        double omega_j = x_i(0)*z_ij(1) + x_i(1)*z_ij(0);
        double eta_j   = -z_ij(1)*x_i(0)-z_ij(0)*x_i(1);
        double kappa_j =  z_ij(0)*x_i(0)-z_ij(1)*x_i(1);
        double alpha_2 = -z_ij(2)*x_i(0)+z_ij(3)*x_i(1)-z_ij(0)*x_i(2)-z_ij(1)*x_i(3);
        double beta_2  = -z_ij(3)*x_i(0)-z_ij(2)*x_i(1)+z_ij(1)*x_i(2)-z_ij(0)*x_i(3);

        //Atan2 method for computing half-angle phi
        Eigen::Vector4d r_ij = pudq_compose(pudq_inv(z_ij), pudq_mul(pudq_inv(x_i), x_j));

        double phi = get_phi_atan2(r_ij(1), r_ij(0));
        double gamma = sinc(phi);
        double f1 = f_1(phi);

        double dphi_dxj0 = eta_j*r_ij(0) - mu_j*r_ij(1);
        double dphi_dxj1 = kappa_j*r_ij(0) - omega_j*r_ij(1);

        //Initialize Bij matrix
        Eigen::MatrixXd B = Eigen::MatrixXd::Zero(3,4);

        B(0,0) = eta_j/gamma + r_ij(1)*dphi_dxj0*f1;
        B(0,1) = kappa_j/gamma + r_ij(1)*dphi_dxj1*f1;

        B(1,0) = alpha_2/gamma + r_ij(2)*dphi_dxj0*f1;
        B(1,1) = beta_2/gamma + r_ij(2)*dphi_dxj1*f1;
        B(1,2) = kappa_j/gamma;
        B(1,3) = -eta_j/gamma;

        B(2,0) = beta_2/gamma + r_ij(3)*dphi_dxj0*f1;
        B(2,1) = -alpha_2/gamma + r_ij(3)*dphi_dxj1*f1;
        B(2,2) = eta_j/gamma;
        B(2,3) = kappa_j/gamma;

        return B;
    }

    Eigen::VectorXd egrad(PUDQGraph *G) {
        // printf("graph with %d vertices", num_vertices);

        Eigen::VectorXd egrad_F = Eigen::VectorXd::Zero(4*G->get_num_vertices());

        //Loop through all edges
        for (auto it_i = G->edges_.begin(); it_i != G->edges_.end(); ++it_i) {

            //Get x_i
            Eigen::Vector4d x_i = G->vertices_pudq_[it_i->first];

            for (auto it_j = it_i->second.begin(); it_j != it_i->second.end(); ++it_j) {
                //Get x_j and z_ij
                Eigen::Vector4d x_j = G->vertices_pudq_[it_j->first];

                Eigen::Vector4d z_ij = it_j->second.delta_pose_pudq;
                Eigen::Matrix3d Omega_ij = it_j->second.information_pudq;

                //Compute residual and jacobians
                Eigen::Vector3d eij = e_ij(z_ij, x_i, x_j);
                Eigen::MatrixXd Aij = A_ij(z_ij, x_i, x_j);
                Eigen::MatrixXd Bij = B_ij(z_ij, x_i, x_j);

                //Compute gradient blocks
                Eigen::Vector4d grad_i = Aij.transpose() * Omega_ij * eij;
                Eigen::Vector4d grad_j = Bij.transpose() * Omega_ij * eij;

                //Update gradient blocks
                egrad_F.segment(4*it_i->first, 4) += grad_i;
                egrad_F.segment(4*it_j->first, 4) += grad_j;
            }
        }

        return egrad_F;
    }

    Eigen::MatrixXd gnhess(PUDQGraph *G) {
        //Initialize Hessian approximation
        Eigen::MatrixXd H_tilde = Eigen::MatrixXd::Zero(4*G->get_num_vertices(), 4*G->get_num_vertices());

        //Loop through all edges
        for (auto it_i = G->edges_.begin(); it_i != G->edges_.end(); ++it_i) {

            //Get x_i
            Eigen::Vector4d x_i = G->vertices_pudq_[it_i->first];

            for (auto it_j = it_i->second.begin(); it_j != it_i->second.end(); ++it_j) {

                //Get x_j and z_ij
                Eigen::Vector4d x_j = G->vertices_pudq_[it_j->first];
                Eigen::Vector4d z_ij = it_j->second.delta_pose_pudq;
                Eigen::Matrix3d Omega_ij = it_j->second.information_pudq;

                //Compute jacobians
                Eigen::MatrixXd Aij = A_ij(z_ij, x_i, x_j);
                Eigen::MatrixXd Bij = B_ij(z_ij, x_i, x_j);

                //Compute Hessian blocks
                Eigen::Matrix4d Hii_tilde = Aij.transpose() * Omega_ij * Aij;
                Eigen::Matrix4d Hij_tilde = Aij.transpose() * Omega_ij * Bij;
                Eigen::Matrix4d Hji_tilde = Hij_tilde.transpose();
                Eigen::Matrix4d Hjj_tilde = Bij.transpose() * Omega_ij * Bij;

                H_tilde.block(4*it_i->first,4*it_i->first,4,4) += Hii_tilde;
                H_tilde.block(4*it_i->first,4*it_j->first,4,4)  = Hij_tilde;
                H_tilde.block(4*it_j->first,4*it_i->first,4,4)  = Hji_tilde;
                H_tilde.block(4*it_j->first,4*it_j->first,4,4) += Hjj_tilde;
            }
        }

        return H_tilde;
    }
}
