/**
 * @file InverseKinematicSolver.cpp
 * 
 * @brief Implementation of the InverseKinematicSolver class.
 * 
 * The main purpose of this class is to provide methods that help
 * with solving the inverse kinematics of a 3-section constant-curvature robot using analytical 
 * methods that is based on the research paper.
 * 
 * @section Credits
 * This code is based on the research paper "An Efficient Multi-solution Solver for the 
 * Inverse Kinematics of 3-Section Constant-Curvature Robots" 
 * @author Ke Qiu, Jingyu Zhang, Danying Sun, Rong Xiong, Haojian Lu, Yue Wang
 * @code https://sites.google.com/view/micsolver/code
 */

#include "../include/InverseKinematicSolver.h"
#include "../include/NumericalMethods.h"
#include "../include/SolverHelper.h"

/**
 * @brief This method solves the inverse kinematics of a 3-section constant-curvature robot.
 * 
 * This method solves the inverse kinematics of a 3-section constant-curvature robot
 * using the Newton-Raphson method. This method will find all possible solutions to the
 * inverse kinematics problem.
 * 
 * @param L1 - the length of the first section
 * @param L2 - the length of the second section
 * @param L3 - the length of the third section
 * @param q - Quaternion representing the orientation of the EE
 * @param r - position of the EE (or translation vector)
 * @param tol - Error tolerance
 * @param msteps - Maximum number of iterations
 * @param noc - Number of corrections
 *
 * @return a matrix containing all possible solutions to the inverse kinematics problem
 *         the matrix has 6 rows and n columns, where n is the number of solutions
 *         each column represents a solution.
 */

Eigen::MatrixXd InverseKinematicSolver::miscSolver(double L1, double L2, double L3, Eigen::Quaterniond q, Eigen::Vector3d r, double par, double tol, int msteps, Eigen::Vector2d noc){

    int noi = 0;
    int nos = 0;
    Eigen::MatrixXd solns = InverseKinematicSolver::miscSolver__(L1, L2, L3, q, r, par, noc);
    Eigen::MatrixXd sol = Eigen::MatrixXd::Zero(6, solns.cols());

    for (int i = 0; i < solns.cols(); i++) {
        Eigen::Matrix<double, 9, 1> soln = solns.col(i);
        Eigen::Matrix<double, 6, 1> xi_star;
        Eigen::Matrix<double, 6, 1> temp = SolverHelper::soln2xi(L1, L2, L3, soln);

        int iterations;
        double err = NumericalMethods::revise_newton(L1, L2, L3, q, r, temp, msteps, tol, xi_star, iterations);
        noi = noi + iterations;
        if (err < tol) {
            sol.col(nos) = xi_star;
            nos = nos + 1;
        }
    }
    std::cout << "Finally ***** assignment, sol:\n" << nos << std::endl;
    Eigen::MatrixXd sol_trimmed = sol.block(0, 0, sol.rows(), nos);
    std::cout << "noi: " << noi << std::endl;
    return sol_trimmed;
}

/**
 * @brief This method does the Candidate selection // TODO MORE DETAILS 
 * 
 * // TODO DEATILED DOC
 * 
 * @param L1 - the length of the first section
 * @param L2 - the length of the second section
 * @param L3 - the length of the third section
 * @param q - Quaternion representing the orientation of the EE
 * @param r - position of the EE (or translation vector)
 * @param par - Partition size
 * @param noc - correction array
 * 
 * @return // todo
 */
Eigen::MatrixXd InverseKinematicSolver::miscSolver__(double L1, double L2, double L3, Eigen::Quaterniond q, Eigen::Vector3d r, double par, Eigen::Vector2d noc){

    double a = q.w();
    double b = q.x();
    double c = q.y();
    double d = q.z();

    Eigen::Matrix3d B;
    B << d, a, b, -a, d, c, -b, -c, d;

    Eigen::Vector3d n0 = B.transpose() * r;

    Eigen::Vector3d n = n0 / n0.norm();

    Eigen::Vector3d r0 = (1.0/2.0 + 1.0/M_PI) * (L3 * d) / n0.norm() * n;
    
    double norm_r0 = r0.norm();

    double norm_r01 = sqrt(1 - norm_r0*norm_r0);

    Eigen::Vector3d u;
    u << n(1), -n(0), 0;
    u = u / u.norm();

    Eigen::Vector3d v = n.cross(u);

    Eigen::Matrix3d P;
    P.col(0) = u;
    P.col(1) = v;
    P.col(2) = n; 

    // std::cout << P << std::endl;
    int num_elements = static_cast<int>((1 - 0) / par) + 1;
    Eigen::VectorXd zeta = Eigen::VectorXd::LinSpaced(num_elements, 0, 1);
    
    int npar = zeta.size();
    
    Eigen::VectorXd err = Eigen::VectorXd::Constant(npar , std::nan("1"));
    Eigen::VectorXd err_r = Eigen::VectorXd::Constant(npar, std::nan("1"));
    Eigen::VectorXd err_t = Eigen::VectorXd::Constant(npar , std::nan("1"));
    Eigen::MatrixXd solns = Eigen::MatrixXd::Constant(9, npar , std::nan("1"));
    Eigen::VectorXd is = Eigen::VectorXd::Constant(npar , std::nan("1"));
    // std::cout << "npar: " << npar << std::endl;

    int nts = 0;
    Eigen::VectorXd soln(9);
    int i;
    for(i = 0; i < npar; i++){
        double t = zeta(i);
        Eigen::Vector3d temp;
        temp << sin(2 * M_PI * t), cos(2 * M_PI * t), 0.0;
        Eigen::Vector3d r3 = r0 + norm_r01 * (P * temp);        
        if(d != 0){
            for(int cor_idx = 0; cor_idx < noc(0); cor_idx++){
                Eigen::Vector3d tmp; 
                Eigen::Vector3d new_val;
                new_val << 0, 0, L3 * d * (1.0/acos(r3(2)) - 1.0/sqrt(1 - pow(r3(2),2)));
                tmp = r3 -( (n0.transpose() * r3 - SolverHelper::rho(r3(2), L3) * d) / ((n0.transpose() + new_val.transpose()) * r0)) * r0;
                r3 = tmp / tmp.norm();
            }
        }

        // if (d != 0) { // When d == 0, the solution is exact, we do not need corrections.
        //     for (int cor_idx = 0; cor_idx < noc(0); ++cor_idx) {
        //         // One-step correction.
        //         Eigen::Vector3d correction = Eigen::Vector3d(0, 0, L3 * d * (1 / acos(r3(2)) - 1 / sqrt(1 - r3(2) * r3(2))));
        //         Eigen::Vector3d n0_prime = n0 + correction;
        //         double tmp_scale = (n0.transpose() * r3 - SolverHelper::rho(r3(2), L3) * d) / (n0_prime.transpose() * r0);
        //         Eigen::Vector3d tmp = r3 - tmp_scale * r0;
        //         r3 = tmp / tmp.norm();
        //     }
        // }


        Eigen::Vector3d r1 = SolverHelper::solve_r1(L1, q, r, r3, noc(1));
        std::array<Eigen::Vector3d, 2> array_val = SolverHelper::solve_r2(L1, L3, q, r, r3, r1);
        Eigen::Vector3d r2r = array_val[0];
        Eigen::Vector3d r2t = array_val[1];

        double e_r = SolverHelper::get_err(r1, r2r, r3, L1, L2, L3, q, r);
        double e_t = SolverHelper::get_err(r1, r2t, r3, L1, L2, L3, q, r);
 
        
        double e;
        Eigen::Vector3d r2;
        if(e_r < e_t){
            r2 = r2r;
            e = e_r;
        } else {
            r2 = r2t;
            e = e_t;
        }

        if(i == 0){
            // No operation here
        } else if (i == 1) {
            solns.col(nts) = soln;
            is(nts) = 1;
            nts = nts + 1;
        } else {
            if (e > err(i-1) && err(i-1) <= err(i-2)){
                solns.col(nts) = soln;
                is(nts) = i-1;
                nts = nts + 1;
            } 
        }
        soln << r1, r2, r3;
        // std::cout << "i "<< i << std::endl;
        err(i) = e;
        err_r(i) = e_r;
        err_t(i) = e_t; 
    }

    i = npar - 1;
    std::cout << err(i) <<std::endl;
    std::cout << err_r(i) <<std::endl;
    std::cout << err_t(i) <<std::endl;
  
    Eigen::MatrixXd solns_final;
    Eigen::VectorXd is_final;

    if (zeta(i) != 1) {
        if (err(0) > err(i) && err(i) <= err(i - 1)) {
            solns_final = Eigen::MatrixXd(solns.leftCols(nts));
            is_final = Eigen::VectorXd(is.head(nts));
            nts = nts + 1;
        }
        if (err(1) > err(0) && err(0) <= err(i)) {
            solns_final = Eigen::MatrixXd(solns.leftCols(nts));
            is_final = Eigen::VectorXd(is.head(nts));
        } else {
            solns_final = solns.block(0, 1, 9, nts);
            is_final = is.segment(1, nts);
        }
    } else {
        if (err(1) > err(0) && err(0) <= err(i - 1)) {
            solns_final = Eigen::MatrixXd(solns.leftCols(nts));
            is_final = Eigen::VectorXd(is.head(nts));
        } else {
            solns_final = solns.block(0, 1, 9, nts - 1);
            is_final = is.segment(1, nts - 1);
        }
    }

    std::cout <<"is_fina: " << is_final << std::endl;
    std::cout << "sols_f"<<solns_final << std::endl;

    Eigen::VectorXd ts = is_final.unaryExpr([&](int idx) { return zeta(idx); });

// // **************************************
//     Eigen::VectorXd abs_diff = (ts.array() - 0.5).abs();
//     std::vector<int> indices(ts.size());
//     for (int i = 0; i < ts.size(); ++i) {
//         indices[i] = i;
//     }
//     std::sort(indices.begin(), indices.end(), [&](int i, int j) {
//         return abs_diff(i) < abs_diff(j);
//     });

//     std::cout << "Sorted indices: ";
//     for (int idx : indices) {
//         std::cout << idx << " ";
//     }
// // **************************************


    return solns_final;

}
