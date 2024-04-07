/**
 * @file NumericalMethods.cpp
 * 
 * @brief Implementation of the NumericalMethods class.
 * 
 * The main purpose of this class is to provide methods that help with
 * performing numerical methods for solving the inverse kinematics of a 
 * 3-section constant-curvature robot.
 * 
 * This class implements two Numerical Methods: 
 *      - Newton-Raphson
 *      - Damped Least Squares
 * 
 * @section Credits
 * This code is based on the research paper "An Efficient Multi-solution Solver for the 
 * Inverse Kinematics of 3-Section Constant-Curvature Robots" 
 * @author Ke Qiu, Jingyu Zhang, Danying Sun, Rong Xiong, Haojian Lu, Yue Wang
 * @code https://sites.google.com/view/micsolver/code
 * 
 */

#include "../include/NumericalMethods.h"
#include "../include/LieAlgebra.h"
#include "../include/ConversionHelpers.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <Eigen/SVD>

/**
 * @brief This method perfroms the Newton-Raphson
 * 
 * This method performs the Newton-Raphson to solve the inverse kinematics of a 3-section constant-curvature robot.
 * The initial guess or the input parameter for this function is based out of candidate selection 
 * makes sure that the solution is found within a few iterations.
 * 
 * @param L1 - the length of the 1st section
 * @param L2 - the length of the 2nd section
 * @param L3 - the length of the 3rd section
 * @param q - the Quaternion representing the orientation of the EE
 * @param r - the position of the EE (or translation vector)
 * @param xi - the initial guess (NOTE: The actual implementation uses Candidate selection)
 * @param msteps - the maximum number of iterations
 * @param tol - Error tolerance
 * @param xi_star - the solution to the inverse kinematics (@return value)
 * @param iterations - the number of iterations taken to solve the IK (@return value)
 * 
 * @return the error value
 */
double NumericalMethods::revise_newton(double L1, double L2, double L3, Eigen::Quaterniond q, Eigen::Vector3d r, Eigen::Matrix<double, 6, 1> xi, int msteps, double tol, Eigen::Matrix<double, 6, 1>& xi_star, int& iterations) {

    Eigen::Matrix4d Td;
    Td <<  q.toRotationMatrix(), r,
          0, 0, 0, 1;

    // std::cout << Td << std::endl;

    Eigen::VectorXd omg_e = Eigen::VectorXd::Constant(msteps + 1, std::nan("1"));
    Eigen::VectorXd v_e = Eigen::VectorXd::Constant(msteps + 1, std::nan("1"));
    Eigen::VectorXd e = Eigen::VectorXd::Constant(msteps + 1, std::nan("1"));

    int k = 0;
    while(k < msteps){
        Eigen::Matrix4d Tt = LieAlgebra::get_end(L1, L2, L3, xi);
        Eigen::VectorXd V  = LieAlgebra::up_vee((Tt.inverse() * Td).log()); 
        omg_e(k) = V.head(3).norm();
        v_e(k) = V.tail(3).norm();
        e(k) = V.norm();
        
        if (e(k) < tol){
            break;
        } else {
            Eigen::MatrixXd J(6,6);
            J = NumericalMethods::jacobian3cc(L1, L2, L3, xi);
            // xi = xi + (J.transpose() * J) * J.adjoint() * V;
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(J.transpose() * J, Eigen::ComputeThinU | Eigen::ComputeThinV);
            xi = xi + svd.solve(J.adjoint()) * V;
            xi = ConversionHelper::arc2xi(L1, L2, L3, ConversionHelper::xi2arc(L1, L2, L3, xi));
 
            k += 1;
        }
    }

    if(k == msteps){
        Eigen::Matrix4d Tt;
        Tt = LieAlgebra::get_end(L1, L2, L3, xi);
        Eigen::VectorXd V = LieAlgebra::up_vee((Tt.inverse() * Td).log());
        omg_e(k) = V.head(3).norm();
        v_e(k) = V.tail(3).norm();
        e(k) = V.norm();
    }
    
    // std::cout << "K: " << k << std::endl;
    iterations = k;
    xi_star = xi;
    return e(k);
}

/**
 * NOTE: THE IMPLEMENTATION OF INVERSE KINEMATICS SOLVER USES THE REVISE_NEWTON METHOD
 *
 * @brief This method performs the Damped Least Squares
 * 
 * This method performs the Damped Least Squares to solve the inverse kinematics of a 3-section constant-curvature robot.
 * The initial guess or the input parameter for this function is based out of candidate selection 
 * makes sure that the solution is found within a few iterations.
 * 
 * @param L1 - the length of the 1st section
 * @param L2 - the length of the 2nd section
 * @param L3 - the length of the 3rd section
 * @param q - the Quaternion representing the orientation of the EE
 * @param r - the position of the EE (or translation vector)
 * @param xi - the initial guess (NOTE: The actual implementation uses Candidate selection)
 * @param msteps - the maximum number of iterations
 * @param tol - Error tolerance
 * @param xi_star - the solution to the inverse kinematics (@return value)
 * 
 * @return the error value
 */
double NumericalMethods::revise_dls(double L1, double L2, double L3, Eigen::Quaterniond q, Eigen::Vector3d r, Eigen::Matrix<double, 6, 1> xi, int msteps, double tol, Eigen::Matrix<double, 6, 1>& xi_star) {
    Eigen::Matrix4d Td;
    Td << ConversionHelper::q2rot(q), r,
          0, 0, 0, 1;

    Eigen::VectorXd omg_e = Eigen::VectorXd::Constant(msteps + 1, std::nan("1"));
    Eigen::VectorXd v_e = Eigen::VectorXd::Constant(msteps + 1, std::nan("1"));
    Eigen::VectorXd e = Eigen::VectorXd::Constant(msteps + 1, std::nan("1"));

    int k = 0;
   

    while(k < msteps){
        Eigen::Matrix4d Tt = LieAlgebra::get_end(L1, L2, L3, xi);
        Eigen::VectorXd V  = LieAlgebra::up_vee((Tt.inverse() * Td).log()); 
        omg_e(k) = V.head(3).norm();
        v_e(k) = V.tail(3).norm();
        e(k) = V.norm();
        if (e(k) < tol){
            break;
        } else {
            Eigen::MatrixXd J(6,6);
            J = NumericalMethods::jacobian3cc(L1, L2, L3, xi);
            Eigen::MatrixXd M = J.transpose() * J;
            double diagonal_max = M.diagonal().maxCoeff();
            Eigen::MatrixXd eye = Eigen::MatrixXd::Identity(6, 6);
            xi = xi + (M + diagonal_max * eye).inverse() * J.transpose() * V;
            xi = ConversionHelper::arc2xi(L1, L2, L3, ConversionHelper::xi2arc(L1, L2, L3, xi));
            k += 1;
        }
    }

    if(k == msteps){
        Eigen::Matrix4d Tt;
        Tt = LieAlgebra::get_end(L1, L2, L3, xi);
        Eigen::VectorXd V = LieAlgebra::up_vee((Tt.inverse() * Td).log());
        omg_e(k) = V.head(3).norm();
        v_e(k) = V.tail(3).norm();
        e(k) = V.norm();
    }

    xi_star = xi;
    
    return e(k);

}

/**
 * @brief This method computes the Jacobian matrix of a 3-section constant-curvature robot.
 * 
 * @param L1 - the length of the 1st section
 * @param L2 - the length of the 2nd section
 * @param L3 - the length of the 3rd section
 * @param xi - twist in exponential coordinates
 * 
 * @return a 6x6 Jacobian matrix
 */
Eigen::MatrixXd NumericalMethods::jacobian3cc(double L1, double L2, double L3, Eigen::Matrix<double, 6, 1> xi){

    Eigen::MatrixXd j3 = NumericalMethods::jaco_c12(xi(4), xi(5), L3);
    
    Eigen::VectorXd temp(6);
    temp << xi(4), xi(5), 0, 0, 0, L3;
    Eigen::Matrix4d T3 = LieAlgebra::up_hat(temp).exp();
    Eigen::Matrix4d invT3;
    invT3 << T3.block(0, 0, 3, 3).transpose(), -T3.block(0, 0, 3, 3).transpose() * T3.block(0, 3, 3, 1),
        0, 0, 0, 1;
    Eigen::MatrixXd j2 = NumericalMethods::jaco_c12(xi(2), xi(3), L2);
    Eigen::VectorXd j2_c1 = LieAlgebra::up_vee(invT3 * LieAlgebra::up_hat(j2.col(0)) * T3);
    Eigen::VectorXd j2_c2 = LieAlgebra::up_vee(invT3 * LieAlgebra::up_hat(j2.col(1)) * T3);
    j2 << j2_c1, j2_c2;

    Eigen::VectorXd temp2(6);
    temp2 << xi(2), xi(3), 0, 0, 0, L2;
    Eigen::Matrix4d T2 = LieAlgebra::up_hat(temp2).exp();
    Eigen::Matrix4d invT2;
    invT2 << T2.block(0, 0, 3, 3).transpose(), -T2.block(0, 0, 3, 3).transpose() * T2.block(0, 3, 3, 1),
        0, 0, 0, 1;
    Eigen::MatrixXd j1 = NumericalMethods::jaco_c12(xi(0), xi(1), L1);
    Eigen::VectorXd j1_c1 = LieAlgebra::up_vee(invT3 * invT2 * LieAlgebra::up_hat(j1.col(0)) * T2 * T3);
    Eigen::VectorXd j1_c2 = LieAlgebra::up_vee(invT3 * invT2 * LieAlgebra::up_hat(j1.col(1)) * T2 * T3);
    j1 << j1_c1, j1_c2;

    Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Zero();
    J << j1, j2, j3;
    return J;
}


/**
 * @brief The function is responsible for computing the sub-Jacobian matrix for an individual section of the continuum robot.
 *
 * @param w1 -  The angular velocities or bending angles of the section.
 * @param w2 -  The angular velocities or bending angles of the section.
 * @param L - the length of the section
 * 
 * @return a 6x3 Jacobian matrix
 */
Eigen::MatrixXd NumericalMethods::jaco_c12(double w1, double w2, double L) {
    Eigen::Vector3d w;
    w << w1, w2, 0;

    Eigen::Matrix3d eye = Eigen::Matrix3d::Identity();
    double n = w.norm();

    Eigen::Matrix<double, 6, 3> Jc;

    if (n == 0) {
        double ML = (double) L / 2.0;
        Jc << eye,
              0, ML, 0,
              -ML, 0, 0,
              0, 0, 0;
    } else {

        double M = (1.0 - cos(n)) / pow(n, 2);
        double N = (n - sin(n)) / pow(n, 3);
        double p1M = w1 / pow(n, 2) - w1 * N - 2.0 * w1 * M / pow(n, 2);
        double p2M = w2 / pow(n, 2) - w2 * N - 2.0 * w2 * M / pow(n, 2);
        double p1N = w1 * M / pow(n, 2) - 3.0 * w1 * N / pow(n, 2);
        double p2N = w2 * M / pow(n, 2) - 3.0 * w2 * N / pow(n, 2);


        Eigen::Matrix3d pwJleftwv;
        pwJleftwv <<                     p1M * w2,                    p2M * w2 + M,        0,
                                    -p1M * w1 - M,                        -p2M * w1,       0,
                    -p1N * pow(n, 2) - 2 * N * w1,    -p2N * pow(n, 2) - 2 * N * w2,       0;

        pwJleftwv = L * pwJleftwv;

        Eigen::Matrix3d w_hat = LieAlgebra::up_hat(w);

        Jc << Eigen::Matrix3d::Identity() - M * w_hat + N * (w_hat * w_hat),
             LieAlgebra::up_hat(w).exp().transpose() * pwJleftwv;

    }

    return Jc.leftCols(2);
}
