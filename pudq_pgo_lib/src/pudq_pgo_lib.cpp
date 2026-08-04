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

        Eigen::Vector4d x_i = edge_data->x_i;
        Eigen::Vector4d x_j = edge_data->x_j;
        Eigen::Vector4d z_ij = edge_data->edge_ij.z_ij_pudq;

        // Compute residual and Jacobians
        Eigen::Vector4d r_ij = pudq_compose(pudq_inv(z_ij), pudq_mul(pudq_inv(x_i), x_j));
        
        // x_i constants
        double mu_i    =  z_ij(0)*x_j(0) + z_ij(1)*x_j(1);
        double omega_i = -z_ij(1)*x_j(0) + z_ij(0)*x_j(1);
        double eta_i   = -z_ij(1)*x_j(0) + z_ij(0)*x_j(1);
        double kappa_i = -x_j(0)*z_ij(0) - x_j(1)*z_ij(1);
        double alpha_1 = -x_j(0)*z_ij(2) - x_j(1)*z_ij(3) + z_ij(0)*x_j(2)+z_ij(1)*x_j(3);
        double beta_1  =  x_j(0)*z_ij(3) - x_j(1)*z_ij(2) - z_ij(1)*x_j(2)+z_ij(0)*x_j(3);
        double xi_1    = -x_j(0)*z_ij(0) + z_ij(1)*x_j(1);
        double zeta_1  = -z_ij(1)*x_j(0) - z_ij(0)*x_j(1);
        double alpha_3 = -x_j(0)*z_ij(3) + x_j(1)*z_ij(2) - x_j(2)*z_ij(1)+z_ij(0)*x_j(3);
        double beta_3  = -x_j(0)*z_ij(2) - x_j(1)*z_ij(3) - x_j(2)*z_ij(0)-z_ij(1)*x_j(3);
        
        // x_j constants
        double mu_j    =  x_i(0)*z_ij(0) - x_i(1)*z_ij(1);
        double omega_j =  x_i(0)*z_ij(1) + x_i(1)*z_ij(0);
        double eta_j   = -z_ij(1)*x_i(0) - z_ij(0)*x_i(1);
        double kappa_j =  z_ij(0)*x_i(0) - z_ij(1)*x_i(1);
        double alpha_2 = -z_ij(2)*x_i(0) + z_ij(3)*x_i(1) - z_ij(0)*x_i(2) - z_ij(1)*x_i(3);
        double beta_2  = -z_ij(3)*x_i(0) - z_ij(2)*x_i(1) + z_ij(1)*x_i(2) - z_ij(0)*x_i(3);
        
        // atan2 method for computing phi
        double phi = pudq_lib::get_phi_atan2(r_ij(1), r_ij(0));
        double gamma = pudq_lib::sinc(phi);

        Eigen::Vector3d eij;
        eij << r_ij(1)/gamma, r_ij(2)/gamma, r_ij(3)/gamma;

        // Compute atan2 gradient term
        double f1 = f_1(phi);
        
        // Initialize Aij jacobian
        Eigen::MatrixXd Aij(3,4);
    
        // Initialize Bij jacobian
        Eigen::MatrixXd Bij(3,4);

        // Aij jacobian
        double dphi_dxi0 = eta_i*r_ij(0) - mu_i*r_ij(1);
        double dphi_dxi1 = kappa_i*r_ij(0) - omega_i*r_ij(1);

        Aij(0,0) = eta_i/gamma   + r_ij(1)*dphi_dxi0*f1;
        Aij(0,1) = kappa_i/gamma + r_ij(1)*dphi_dxi1*f1;
        Aij(1,0) = alpha_1/gamma + r_ij(2)*dphi_dxi0*f1;
        Aij(1,1) = beta_1/gamma  + r_ij(2)*dphi_dxi1*f1;
        Aij(1,2) = xi_1/gamma;
        Aij(1,3) = zeta_1/gamma;
        Aij(2,0) = alpha_3/gamma + r_ij(3)*dphi_dxi0*f1;
        Aij(2,1) = beta_3/gamma  + r_ij(3)*dphi_dxi1*f1;
        Aij(2,2) = -zeta_1/gamma;
        Aij(2,3) = xi_1/gamma;
        
        // Bij jacobian
        double dphi_dxj0 = eta_j*r_ij(0) - mu_j*r_ij(1);
        double dphi_dxj1 = kappa_j*r_ij(0) - omega_j*r_ij(1);

        Bij(0,0) = eta_j/gamma + r_ij(1)*dphi_dxj0*f1;
        Bij(0,1) = kappa_j/gamma + r_ij(1)*dphi_dxj1*f1;
        Bij(1,0) = alpha_2/gamma + r_ij(2)*dphi_dxj0*f1;
        Bij(1,1) = beta_2/gamma + r_ij(2)*dphi_dxj1*f1;
        Bij(1,2) = kappa_j/gamma;
        Bij(1,3) = -eta_j/gamma;
        Bij(2,0) = beta_2/gamma + r_ij(3)*dphi_dxj0*f1;
        Bij(2,1) = -alpha_2/gamma + r_ij(3)*dphi_dxj1*f1;
        Bij(2,2) = eta_j/gamma;
        Bij(2,3) = kappa_j/gamma;

        J_Edge_ij *J_blk = new J_Edge_ij;
        J_blk->i = edge_data->edge_ij.i;
        J_blk->j = edge_data->edge_ij.j;
        J_blk->eij = eij;
        J_blk->Aij = Aij;
        J_blk->Bij = Bij;

        // Used to terminate a thread and the return value is passed as a pointer
        pthread_exit(static_cast<void *>(J_blk));
    }

    std::tuple<std::vector<Eigen::Vector4d>, SparseMatrix, double, Eigen::VectorXd, SparseMatrix> rgn_gradhess(PUDQGraph &G) {

        const int N = G.get_num_vertices();
        const int M = G.get_num_edges();

        // std::cout << N << " vertices, " << M << " edges" << std::endl;

        // Get the entire vertex set
        std::vector<Eigen::Vector4d> X = G.get_vertices();

        // Initialize edge residual vector
        Eigen::VectorXd E_vec = Eigen::VectorXd::Zero(3*M);

        std::vector<pthread_t> t_id(M);

        // Allocate data structs to pass to threads
        std::vector<J_Edge_data> edge_ij_data(M);

        std::vector<PUDQGraph::Edge> edges = G.get_edges();

        // Process each edge in a separate thread
        for (int ij = 0; ij < M; ij++) {

            // Todo: check for segfaults (for now assume the graph class handled this properly)
            edge_ij_data[ij].x_i = X[edges[ij].i];
            edge_ij_data[ij].x_j = X[edges[ij].j];
            edge_ij_data[ij].edge_ij = edges[ij];

            pthread_create(&t_id[ij], NULL, J_ij, (void *)(&edge_ij_data[ij]));
        }

        std::vector<Eigen::Triplet<double>> J_triplet_list;

        // Join all edge threads
        for (int ij = 0; ij < M; ij++) {
            // Get return value ptr from each thread
            void *ret;
            pthread_join(t_id[ij], &ret);

            J_Edge_ij *edge = static_cast<J_Edge_ij *>(ret);

            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 4; j++) {
                    J_triplet_list.push_back(Eigen::Triplet<double>(3*ij+i, 4*edge->i+j, edge->Aij(i,j)));
                    J_triplet_list.push_back(Eigen::Triplet<double>(3*ij+i, 4*edge->j+j, edge->Bij(i,j)));
                }
            }

            E_vec.segment(3*ij, 3) = edge->eij;

            // Free up the memory allocated for this edge
            delete edge;
        }

        SparseMatrix J_mat(3*M, 4*N);
        J_mat.setFromTriplets(J_triplet_list.begin(), J_triplet_list.end());

        Eigen::SparseMatrix<double, Eigen::RowMajor> Omega = G.get_Omega();

        double F_X = 0.5*E_vec.transpose()*Omega*E_vec;
        
        Eigen::VectorXd egrad_X = J_mat.transpose()*Omega*E_vec;
        SparseMatrix egnhess_X = J_mat.transpose()*Omega*J_mat;

        SparseMatrix P_X = P_X_N(X);
        Eigen::VectorXd rgrad_X = P_X*egrad_X;
        SparseMatrix rgnhess_X = P_X*egnhess_X*P_X;

        // std::cout << "F(X) = " << F_X << std::endl;
        // std::cout << "||J(X)|| = " << J_mat.norm() << std::endl;
        // std::cout << "||gradF(X)|| = " << rgrad_X.norm() << std::endl;
        // std::cout << "||rgnhess(X)|| = " << rgnhess_X.norm() << std::endl;

        return std::make_tuple(X, P_X, F_X, rgrad_X, rgnhess_X);
    }

    void optimize_rgn(PUDQGraph &G, double tol, int max_iter) {

        std::cout << G.get_num_vertices() << " vertices, " << G.get_num_edges() << " edges" << std::endl;

        size_t N = G.get_num_vertices();

        // Initialize opt variables
        std::vector<Eigen::Vector4d> X_k = G.get_vertices();
        SparseMatrix P_X(4*N, 4*N);
        double F_k = F_G_pudq(G);
        Eigen::VectorXd rgrad_k(4*N);
        SparseMatrix rgnhess_k(4*N, 4*N);

        SparseMatrix H_k(4*(N-1), 4*(N-1));
        Eigen::VectorXd b_k(4*(N-1));
    
        Eigen::VectorXd S_k = Eigen::VectorXd::Zero(4*N);
        Eigen::VectorXd S_k_trunc(4*(N-1));

        // Call once to initialize the solver
        // std::tie(X_k, P_X, F_k, rgrad_k, rgnhess_k) = rgn_gradhess(G);
        // H_k = rgnhess_k.block(4, 4, 4*(N-1), 4*(N-1));
        // b_k = rgrad_k.tail(4*(N-1));

        // Eigen::SparseLU<SparseMatrix> solver;
        // Eigen::SimplicialLDLT<SparseMatrix> solver;

        // instantiate the solver
        OsqpEigen::Solver solver;
        solver.data()->setNumberOfVariables(4*(N-1));
        solver.data()->setNumberOfConstraints(0);

        solver.settings()->setVerbosity(false);

        // Set custom convergence tolerances
        solver.settings()->setAbsoluteTolerance(1e-8);
        solver.settings()->setRelativeTolerance(1e-8);
        solver.settings()->setPrimalInfeasibilityTolerance(1e-8);
        solver.settings()->setDualInfeasibilityTolerance(1e-8);

        for (int k = 0; k < max_iter; k++) {

            //Compute Riemannian gradient and Gauss-Newton Hessian
            std::tie(X_k, P_X, F_k, rgrad_k, rgnhess_k) = rgn_gradhess(G);

            // Solve LLS system
            H_k = rgnhess_k.block(4, 4, 4*(N-1), 4*(N-1));
            b_k = rgrad_k.tail(4*(N-1));

            if (k == 0) {
                if (!solver.data()->setHessianMatrix(H_k)) {
                    std::fprintf(stderr, "Error setting Hessian!\n");
                    return;
                }

                if (!solver.data()->setGradient(b_k)) {
                    std::fprintf(stderr, "Error setting Gradient!\n");
                    return;
                }

                // instantiate the solver
                if (!solver.initSolver()) {
                    std::fprintf(stderr, "Error initializing solver!\n");
                    return;
                }
            } else {
                if (!solver.updateHessianMatrix(H_k)) {
                    std::fprintf(stderr, "Error updating Hessian!\n");
                    return;
                }

                if (!solver.updateGradient(b_k)) {
                    std::fprintf(stderr, "Error updating Gradient!\n");
                    return;
                }
            }

            // solve the QP problem
            if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
                std::fprintf(stderr, "Solver failed!\n");
                return;
            }

            S_k_trunc = solver.getSolution();

            double linear_residual = (H_k*S_k_trunc + b_k).norm();
            std::cout << "Linear residual ||H*S-b|| = " << linear_residual << std::endl;

            S_k.tail(4*(N-1)) = S_k_trunc;
            S_k = P_X*S_k;

            for (size_t i = 1; i < G.get_num_vertices(); i++) {

                // RGN
                G.set_vertex(i, Exp_x(X_k[i], S_k.segment(4*i, 4)));

                // RGD
                // G.set_vertex(i, Exp_x(x_i, -rgd_stepsize*rgrad_k.segment(4*i,4)));
            }

            // Super inefficient but do this for now...
            std::tie(X_k, P_X, F_k, rgrad_k, rgnhess_k) = rgn_gradhess(G);

            //Compute the new gradient norm
            // double gradnorm = (P_X_N(G.get_vertices()) * egrad(G)).norm();
            double gradnorm = rgrad_k.norm();

            printf("Iteration %d:\nGrad norm = %f\nCost = %f\n\n", k+1, gradnorm, F_k);

            // double delta_F = abs(F_k - F_prev);
            if (gradnorm < tol) {
                printf("RGN converged to within tolerance of %f\n", tol);
                return;
            }
        }

        printf("RGN max iterations reached.\n");
    }

    void optimize_rlm(PUDQGraph &G, double epsilon_g, int max_iter) {

        std::cout << G.get_num_vertices() << " vertices, " << G.get_num_edges() << " edges" << std::endl;

        size_t N = G.get_num_vertices();

        // Initialize opt variables
        std::vector<Eigen::Vector4d> X_k = G.get_vertices();
        SparseMatrix P_X(4*N, 4*N);
        double F_k = F_G_pudq(G);
        Eigen::VectorXd rgrad_k(4*N);
        SparseMatrix rgnhess_k(4*N, 4*N);

        SparseMatrix H_k(4*(N-1), 4*(N-1));
        Eigen::VectorXd b_k(4*(N-1));
    
        Eigen::VectorXd S_k = Eigen::VectorXd::Zero(4*N);
        Eigen::VectorXd S_k_trunc(4*(N-1));

        SparseMatrix eye(4*(N-1), 4*(N-1));
        eye.setIdentity();

        // Compute initial performance metrics
        std::tie(X_k, P_X, F_k, rgrad_k, rgnhess_k) = rgn_gradhess(G);
        double gradnorm_k = rgrad_k.norm();

        fprintf(stdout, "RLM Initialization: F_k = %.4f, ||g_k|| = %.4f\n", F_k, gradnorm_k);

        // instantiate the solver
        OsqpEigen::Solver solver;
        solver.data()->setNumberOfVariables(4*(N-1));
        solver.data()->setNumberOfConstraints(0);
        solver.settings()->setVerbosity(false);
        solver.settings()->setAbsoluteTolerance(1e-8);
        solver.settings()->setRelativeTolerance(1e-8);
        solver.settings()->setPrimalInfeasibilityTolerance(1e-8);
        solver.settings()->setDualInfeasibilityTolerance(1e-8);
 
        // RLM parameters
        double mu_min = 1e-12;
        double mu_k = mu_min;
        double v = 2.0;
        double eta = 0.1;

        bool converged = false;

        // Set lambda to zero bc it gets set at every iteration inside the loop
        double lambda_k = 0;

        int k = 0;
        bool last_accepted = true;

        for (k = 0; k < max_iter; k++) {

            // If the last step was rejected, then only mu has changed.
            if (last_accepted) {
                std::tie(X_k, P_X, F_k, rgrad_k, rgnhess_k) = pudq_pgo_lib::rgn_gradhess(G);
            }
            
            // Start updating analytics after the first iteration is complete
            if (k > 0) {
                // Compute convergence properties
                double gradnorm_k = rgrad_k.norm();
                fprintf(stdout, "Iteration %d: F_k = %.4f, ||g_k|| = %.4f, lambda = %.4e\n", k, F_k, gradnorm_k, lambda_k);
            }
            
            if (rgrad_k.norm() < epsilon_g) {
                std::cout << "RLM converged in " << k << " iterations." << std::endl;
                converged = true;
                break;
            }

            lambda_k = 2*mu_k*F_k;

            H_k = rgnhess_k.block(4, 4, 4*(N-1), 4*(N-1)) + lambda_k*eye;
            b_k = rgrad_k.tail(4*(N-1));

            if (k == 0) {
                if (!solver.data()->setHessianMatrix(H_k)) {
                    std::fprintf(stderr, "Error setting Hessian!\n");
                    return;
                }

                if (!solver.data()->setGradient(b_k)) {
                    std::fprintf(stderr, "Error setting Gradient!\n");
                    return;
                }

                // instantiate the solver
                if (!solver.initSolver()) {
                    std::fprintf(stderr, "Error initializing solver!\n");
                    return;
                }
            } else {
                if (!solver.updateHessianMatrix(H_k)) {
                    std::fprintf(stderr, "Error updating Hessian!\n");
                    return;
                }

                if (!solver.updateGradient(b_k)) {
                    std::fprintf(stderr, "Error updating Gradient!\n");
                    return;
                }
            }

            // solve the QP problem
            if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
                std::fprintf(stderr, "Solver failed!\n");
                return;
            }

            S_k_trunc = solver.getSolution();

            // Check linear residual
            double linear_residual = (H_k*S_k_trunc + b_k).norm();
            // std::cout << "Linear residual ||H*S-b|| = " << linear_residual << std::endl;

            S_k.tail(4*(N-1)) = S_k_trunc;
            S_k = P_X*S_k;

            std::vector<Eigen::Vector4d> Exp_S_k = pudq_lib::Exp_X_N(X_k, S_k);

            G.set_vertices(Exp_S_k);
            double F_new = F_G_pudq(G);

            // Calculate the predicted reduction from the quadratic model
            double predicted_reduction = -rgrad_k.dot(S_k) - 0.5 * S_k.dot(rgnhess_k * S_k) - 0.5 * lambda_k*S_k.dot(S_k);
            double actual_reduction = F_k - F_new;
            double rho = actual_reduction / predicted_reduction;

            // std::cout << "Predicted = " << predicted_reduction << ", Actual = " <<  actual_reduction << ", rho = " << rho << std::endl;
            
            if (rho > eta) {
                // Accept the step
                mu_k = std::max(mu_k / v, mu_min);

                last_accepted = true;
            } else {
                // Reject the step
                G.set_vertices(X_k);
                mu_k = mu_k * v;

                // We reverted X_k, so there's no need to recompute the gradient
                last_accepted = false;
            }
        }

        if (!converged) {
            std::cerr << "RLM DID NOT CONVERGE! I REPEAT, RLM DID NOT CONVERGE!" << std::endl;
        }
    }

    double F_G_pudq(const PUDQGraph &G) {

        double F = 0.0;

        std::map<size_t, std::map<size_t, size_t>> adjacency_list = G.get_adjacency();
        std::vector<PUDQGraph::Edge> edges = G.get_edges();

        // double F1 = 0.0;
        // for (size_t ij = 0; ij < edges.size(); ij++) {
        //     Eigen::Vector4d x_i = G.get_vertex(edges[ij].i);
        //     Eigen::Vector4d x_j = G.get_vertex(edges[ij].j);
        //     Eigen::Vector4d z_ij = edges[ij].z_ij_pudq;
        //     Eigen::Matrix3d Omega_ij = edges[ij].Omega_ij_pudq;

        //     //Compute residual
        //     Eigen::Vector3d eij = e_ij(z_ij, x_i, x_j);
        //     double f_ij = eij.transpose() * Omega_ij * eij;

        //     // std::cout << "Edge " << ij << ":" << std::endl << "eij=" << eij.transpose() << std::endl << "fij=" << f_ij << std::endl << "Omega_ij=" << std::endl << Omega_ij << std::endl;

        //     F1 += f_ij;
        // }
        // F1 = 0.5*F1;
        // std::cout << "F = " << F1 << std::endl;

        //Loop through all edges
        for (auto it0 = adjacency_list.begin(); it0 != adjacency_list.end(); ++it0) {

            //Get x_i
            int i = it0->first;
            Eigen::Vector4d x_i = G.get_vertex(i);

            for (auto it1 = it0->second.begin(); it1 != it0->second.end(); ++it1) {

                //Get x_j and z_ij
                int j = it1->first;
                Eigen::Vector4d x_j = G.get_vertex(j);
                Eigen::Vector4d z_ij = edges[it1->second].z_ij_pudq;
                Eigen::Matrix3d Omega_ij = edges[it1->second].Omega_ij_pudq;

                //Compute residual
                Eigen::Vector3d eij = e_ij(z_ij, x_i, x_j);

                double f_ij = eij.transpose() * Omega_ij * eij;

                if (f_ij < 0) {
                    std::cout << "fij = " << f_ij << std::endl;
                    std::cout << Omega_ij.determinant() << std::endl;
                    return 0;
                }

                F += f_ij;
            }
        }

        return 0.5*F;
    }

    double f_1(double x) {
        double y = 0.0;
        if (fabs(x) < 1e-2) {
            y = x/3 + (7*std::pow(x,3))/90 + (31*std::pow(x,5))/2520 + (127*std::pow(x,7))/75600 + (73*std::pow(x,9))/342144 + (1414477*std::pow(x,11))/54486432000 + (8191*std::pow(x,13))/2668723200 + (16931177*std::pow(x,15))/47636709120000;
        } else {
            double sin_x = sin(x);
            double csc_x = 1.0/sin_x;
            y = csc_x*csc_x*(sin_x - x*cos(x));
        }

        return y;
    }

    Eigen::Vector3d e_ij(Eigen::Vector4d z_ij, Eigen::Vector4d x_i, Eigen::Vector4d x_j) {
        Eigen::Vector4d q_ij = pudq_compose(pudq_inv(z_ij), pudq_mul(pudq_inv(x_i), x_j));
        return Lie_Log_1(q_ij);
    }

    //Jacobians
    // Eigen::MatrixXd A_ij(Eigen::Vector4d z_ij, Eigen::Vector4d x_i, Eigen::Vector4d x_j) {
    //     //x_i constants
    //     double mu_i    = x_j(0)*z_ij(0) + x_j(1)*z_ij(1);
    //     double omega_i = -x_j(0)*z_ij(1) + x_j(1)*z_ij(0);
    //     double eta_i   = -z_ij(1)*x_j(0)+z_ij(0)*x_j(1);
    //     double kappa_i = -x_j(0)*z_ij(0)-x_j(1)*z_ij(1);
    //     double alpha_1 = -x_j(0)*z_ij(2)-x_j(1)*z_ij(3)+x_j(2)*z_ij(0)+x_j(3)*z_ij(1);
    //     double beta_1  = x_j(0)*z_ij(3)-x_j(1)*z_ij(2)-x_j(2)*z_ij(1)+x_j(3)*z_ij(0);
    //     double xi_1    = -x_j(0)*z_ij(0)+x_j(1)*z_ij(1);
    //     double zeta_1  = -z_ij(1)*x_j(0)-z_ij(0)*x_j(1);
    //     double alpha_3 = -x_j(0)*z_ij(3)+x_j(1)*z_ij(2)-x_j(2)*z_ij(1)+x_j(3)*z_ij(0);
    //     double beta_3  = -x_j(0)*z_ij(2)-x_j(1)*z_ij(3)-x_j(2)*z_ij(0)-x_j(3)*z_ij(1);

    //     //Atan2 method for computing half-angle phi
    //     Eigen::Vector4d r_ij = pudq_compose(pudq_inv(z_ij), pudq_mul(pudq_inv(x_i), x_j));

    //     double phi = get_phi_atan2(r_ij(1), r_ij(0));
    //     double gamma = sinc(phi);
    //     double f1 = f_1(phi);

    //     double dphi_dxi0 = eta_i*r_ij(0) - mu_i*r_ij(1);
    //     double dphi_dxi1 = kappa_i*r_ij(0) - omega_i*r_ij(1);

    //     //Initialize Aij matrix
    //     Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3,4);
        
    //     A(0,0) = eta_i/gamma   + r_ij(1)*dphi_dxi0*f1;
    //     A(0,1) = kappa_i/gamma + r_ij(1)*dphi_dxi1*f1;

    //     A(1,0) = alpha_1/gamma + r_ij(2)*dphi_dxi0*f1;
    //     A(1,1) = beta_1/gamma  + r_ij(2)*dphi_dxi1*f1;
    //     A(1,2) = xi_1/gamma;
    //     A(1,3) = zeta_1/gamma;

    //     A(2,0) = alpha_3/gamma + r_ij(3)*dphi_dxi0*f1;
    //     A(2,1) = beta_3/gamma  + r_ij(3)*dphi_dxi1*f1;
    //     A(2,2) = -zeta_1/gamma;
    //     A(2,3) = xi_1/gamma;

    //     return A;
    // }

    // Eigen::MatrixXd B_ij(Eigen::Vector4d z_ij, Eigen::Vector4d x_i, Eigen::Vector4d x_j) {
    //     //x_j constants
    //     double mu_j    = x_i(0)*z_ij(0) - x_i(1)*z_ij(1);
    //     double omega_j = x_i(0)*z_ij(1) + x_i(1)*z_ij(0);
    //     double eta_j   = -z_ij(1)*x_i(0)-z_ij(0)*x_i(1);
    //     double kappa_j =  z_ij(0)*x_i(0)-z_ij(1)*x_i(1);
    //     double alpha_2 = -z_ij(2)*x_i(0)+z_ij(3)*x_i(1)-z_ij(0)*x_i(2)-z_ij(1)*x_i(3);
    //     double beta_2  = -z_ij(3)*x_i(0)-z_ij(2)*x_i(1)+z_ij(1)*x_i(2)-z_ij(0)*x_i(3);

    //     //Atan2 method for computing half-angle phi
    //     Eigen::Vector4d r_ij = pudq_compose(pudq_inv(z_ij), pudq_mul(pudq_inv(x_i), x_j));

    //     double phi = get_phi_atan2(r_ij(1), r_ij(0));
    //     double gamma = sinc(phi);
    //     double f1 = f_1(phi);

    //     double dphi_dxj0 = eta_j*r_ij(0) - mu_j*r_ij(1);
    //     double dphi_dxj1 = kappa_j*r_ij(0) - omega_j*r_ij(1);

    //     //Initialize Bij matrix
    //     Eigen::MatrixXd B = Eigen::MatrixXd::Zero(3,4);

    //     B(0,0) = eta_j/gamma + r_ij(1)*dphi_dxj0*f1;
    //     B(0,1) = kappa_j/gamma + r_ij(1)*dphi_dxj1*f1;

    //     B(1,0) = alpha_2/gamma + r_ij(2)*dphi_dxj0*f1;
    //     B(1,1) = beta_2/gamma + r_ij(2)*dphi_dxj1*f1;
    //     B(1,2) = kappa_j/gamma;
    //     B(1,3) = -eta_j/gamma;

    //     B(2,0) = beta_2/gamma + r_ij(3)*dphi_dxj0*f1;
    //     B(2,1) = -alpha_2/gamma + r_ij(3)*dphi_dxj1*f1;
    //     B(2,2) = eta_j/gamma;
    //     B(2,3) = kappa_j/gamma;

    //     return B;
    // }

    // Eigen::VectorXd egrad(PUDQGraph *G) {
    //     // printf("graph with %d vertices", num_vertices);

    //     Eigen::VectorXd egrad_F = Eigen::VectorXd::Zero(4*G.get_num_vertices());

    //     //Loop through all edges
    //     for (auto it_i = G.edges_.begin(); it_i != G.edges_.end(); ++it_i) {

    //         //Get x_i
    //         Eigen::Vector4d x_i = G.vertices_pudq_[it_i->first];

    //         for (auto it_j = it_i->second.begin(); it_j != it_i->second.end(); ++it_j) {
    //             //Get x_j and z_ij
    //             Eigen::Vector4d x_j = G.vertices_pudq_[it_j->first];

    //             Eigen::Vector4d z_ij = it_j->second.delta_pose_pudq;
    //             Eigen::Matrix3d Omega_ij = it_j->second.information_pudq;

    //             //Compute residual and jacobians
    //             Eigen::Vector3d eij = e_ij(z_ij, x_i, x_j);
    //             Eigen::MatrixXd Aij = A_ij(z_ij, x_i, x_j);
    //             Eigen::MatrixXd Bij = B_ij(z_ij, x_i, x_j);

    //             //Compute gradient blocks
    //             Eigen::Vector4d grad_i = Aij.transpose() * Omega_ij * eij;
    //             Eigen::Vector4d grad_j = Bij.transpose() * Omega_ij * eij;

    //             //Update gradient blocks
    //             egrad_F.segment(4*it_i->first, 4) += grad_i;
    //             egrad_F.segment(4*it_j->first, 4) += grad_j;
    //         }
    //     }

    //     return egrad_F;
    // }

    // Eigen::MatrixXd gnhess(PUDQGraph *G) {
    //     //Initialize Hessian approximation
    //     Eigen::MatrixXd H_tilde = Eigen::MatrixXd::Zero(4*G.get_num_vertices(), 4*G.get_num_vertices());

    //     //Loop through all edges
    //     for (auto it_i = G.edges_.begin(); it_i != G.edges_.end(); ++it_i) {

    //         //Get x_i
    //         Eigen::Vector4d x_i = G.vertices_pudq_[it_i->first];

    //         for (auto it_j = it_i->second.begin(); it_j != it_i->second.end(); ++it_j) {

    //             //Get x_j and z_ij
    //             Eigen::Vector4d x_j = G.vertices_pudq_[it_j->first];
    //             Eigen::Vector4d z_ij = it_j->second.delta_pose_pudq;
    //             Eigen::Matrix3d Omega_ij = it_j->second.information_pudq;

    //             //Compute jacobians
    //             Eigen::MatrixXd Aij = A_ij(z_ij, x_i, x_j);
    //             Eigen::MatrixXd Bij = B_ij(z_ij, x_i, x_j);

    //             //Compute Hessian blocks
    //             Eigen::Matrix4d Hii_tilde = Aij.transpose() * Omega_ij * Aij;
    //             Eigen::Matrix4d Hij_tilde = Aij.transpose() * Omega_ij * Bij;
    //             Eigen::Matrix4d Hji_tilde = Hij_tilde.transpose();
    //             Eigen::Matrix4d Hjj_tilde = Bij.transpose() * Omega_ij * Bij;

    //             H_tilde.block(4*it_i->first,4*it_i->first,4,4) += Hii_tilde;
    //             H_tilde.block(4*it_i->first,4*it_j->first,4,4)  = Hij_tilde;
    //             H_tilde.block(4*it_j->first,4*it_i->first,4,4)  = Hji_tilde;
    //             H_tilde.block(4*it_j->first,4*it_j->first,4,4) += Hjj_tilde;
    //         }
    //     }

    //     return H_tilde;
    // }
}
