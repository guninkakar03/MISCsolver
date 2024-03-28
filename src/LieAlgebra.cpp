/**
 * @file LieAlgebra.cpp
 * 
 * @brief Implementation of the LieAlgebra class.
 * 
 * The main purpose of this class is to provide methods that help
 * with various Linear Algebra operations.
 * 
 * @section Credits
 * This code is based on the research paper "An Efficient Multi-solution Solver for the 
 * Inverse Kinematics of 3-Section Constant-Curvature Robots" 
 * @author Ke Qiu, Jingyu Zhang, Danying Sun, Rong Xiong, Haojian Lu, Yue Wang
 * @code https://sites.google.com/view/micsolver/code
 * 
 */

#include "../include/LieAlgebra.h"

 
/**
 * @brief This method computes the Lie algebra of a vector.
 * 
 * This method converts a vector to a skew-symmetric matrix.
 * If the input vector is a 3x1 vector, the output is a 3x3 matrix.
 * If the input vector is a 6x1 vector, the output is a 4x4 matrix.
 *
 * @param v - the vector to be converted
 * @return the Lie algebra matrix
 */
Eigen::MatrixXd LieAlgebra::up_hat(Eigen::VectorXd V) {
    if (V.size() == 3) {
        Eigen::Matrix3d M;
        M <<    0, -V(2),  V(1),
            V(2),     0, -V(0),
            -V(1),  V(0),     0;
        return M;
    } else if (V.size() == 6) {
        Eigen::Matrix4d M;
        M <<    0, -V(2),  V(1), V(3),
            V(2),     0, -V(0), V(4),
            -V(1),  V(0),     0, V(5),
                0,     0,     0,    0;
        return M;
    } 
    return Eigen::MatrixXd::Identity(3, 3);
}

/*
* This method computes the Lie algebra of a vector.
* V = UP_VEE(M) is an element of \mathbb{R}^3 or \mathbb{R}^6, where M is
* an element of \mathsf{so}_3 or \mathsf{se}_3, respectively.
*
* NOTE: This method is equivalent to the MATLAB function MISC.up_vee(M)
*
* @param M - the matrix to be converted
* @return the Lie algebra vector
*/
Eigen::VectorXd LieAlgebra::up_vee(Eigen::MatrixXd M) {
    if (M.rows() == 3 && M.cols() == 3) {
        Eigen::Vector3d V;
        V << -M(1, 2), M(0, 2), -M(0, 1);
        return V;
    } else if (M.rows() == 4 && M.cols() == 4) {
        Eigen::VectorXd V(6);
        V << -M(1, 2), M(0, 2), -M(0, 1), M(0, 3), M(1, 3), M(2, 3);
        return V;
    }
    return Eigen::VectorXd::Zero(3);
}

/*
* This method does VEELOG Composition of the matrix logarithm and the vee map.
* V = VEELOG(M) is a vector in \mathbb{R}^3 or \mathbb{R}^4 and is
* computed using Rodrigues' formula. The matrix M is in \mathsf{SO}_3 or
* \mathsf{SE}_3. The vee map sends an element of \mathsf{so}_3 or
* \mathsf{se}_3 to a vector.
*
* NOTE: This method is equivalent to the MATLAB function MISC.veelog(M)
*
* @param M - the matrix to be converted
* @return the VEELOG vector
*/
Eigen::VectorXd LieAlgebra::veelog(Eigen::MatrixXd M) {
    Eigen::Matrix3d R = M.block<3, 3>(0, 0);
    Eigen::Matrix3d eye = Eigen::Matrix3d::Identity();
    if((R - eye).norm() < 2e-8) {
        Eigen::VectorXd V(6);
        V << 0, 0, 0, M(0, 3), M(1, 3), M(2, 3);
        return V;
    } 
    else {
        double theta = acos((R.trace() - 1) / 2.0);
        Eigen::Matrix3d omega_hat = 1.0 / (2 * sin(theta)) * (R - R.transpose());
        Eigen::VectorXd V(6);
        V << LieAlgebra::up_vee(omega_hat) * theta,
           (eye - theta/2*omega_hat + (1 - theta/2.0*(1.0/tan(theta/2.0))) * omega_hat * omega_hat) * M.block<3, 1>(0, 3);

        return V;
    }
    return Eigen::VectorXd::Zero(6);
}

/**
 * @brief This method computes the matrix logarithm of a Lie algebra vector.
* This method computes the matrix exponential of a Lie algebra vector.
* The input vector V is a 6x1 vector in \mathbb{R}^6.
* The output is a 4x4 matrix in \mathsf{SE}_3.
*
* @param V - the Lie algebra vector
* @return the matrix exponential
*/
Eigen::Matrix4d LieAlgebra::exphat(Eigen::Matrix<double, 6, 1> V) {
    Eigen::Matrix4d M = Eigen::Matrix4d::Zero();  // Initialize M to zero
    double theta = V.head(3).norm();
    if (theta < 2e-8) {
        M.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();  // Set the top-left 3x3 block to identity
        M.block<3, 1>(0, 3) = V.tail(3);                    // Set the top-right 3x1 block to the last three elements of V
        M(3, 3) = 1;                                        // Set the bottom-right element to 1
    } else {
        Eigen::Vector3d omega = V.head(3) / theta;
        Eigen::Vector3d v = V.tail(3) / theta;
        Eigen::Matrix3d omega_hat = LieAlgebra::up_hat(omega);
        Eigen::Matrix3d A = Eigen::Matrix3d::Identity() + (sin(theta) * omega_hat) + ((1 - cos(theta)) *(omega_hat * omega_hat));
        Eigen::Vector3d b = (Eigen::Matrix3d::Identity() * theta + (1 - cos(theta)) * omega_hat + (theta - sin(theta)) * (omega_hat * omega_hat)) * v;
        M.block<3, 3>(0, 0) = A;  // Set the top-left 3x3 block to A
        M.block<3, 1>(0, 3) = b;  // Set the top-right 3x1 block to b
        M(3, 3) = 1;              // Set the bottom-right element to 1
    }
    return M;
}

/**
 * @brief This method computes the UP_PLUS operation of a quaternion.
 * 
 * @param q - a unit quaternion
 * @return a 4x4 matrix representing the UP_PLUS operation
 */
Eigen::Matrix4d LieAlgebra::up_plus(Eigen::Quaterniond q){
    double delta = q.w();
    Eigen::Vector3d epsilon;
    epsilon << q.x(), q.y(), q.z();
    Eigen::Matrix4d q_up_plus;
    q_up_plus << delta, -epsilon.transpose(),
        epsilon, delta * Eigen::Matrix3d::Identity() + LieAlgebra::up_hat(epsilon);
    return q_up_plus;
}

/**
 * @brief This method computes the UP_OPLUS operation of a quaternion.
 * The input quaternion q is a unit quaternion.
 * The output is a 4x4 matrix in \mathsf{SE}_3.
 *
 * @param q - a unit quaternion
 * @return the UP_OPLUS matrix
 */
Eigen::Matrix4d LieAlgebra::up_oplus(Eigen::Quaterniond q){
    double delta = q.w();
    Eigen::Vector3d epsilon;
    epsilon << q.x(), q.y(), q.z();
    Eigen::Matrix4d q_up_oplus;
    q_up_oplus << delta, -epsilon.transpose(),
        epsilon, delta * Eigen::Matrix3d::Identity() - LieAlgebra::up_hat(epsilon);
    return q_up_oplus;
}

/**
 * @brief This method computes the UP_STAR operation of a quaternion.
 * The input quaternion q is a unit quaternion.
 * The output is a quaternion in \mathbb{R}^4.
 *
 * @param q - a unit quaternion
 * @return the UP_STAR quaternion
 */
Eigen::Quaterniond LieAlgebra::up_star(Eigen::Quaterniond q){
    return Eigen::Quaterniond(q.w(), -q.x(), -q.y(), -q.z());
}

/**
 * @brief This method computes the end effector transformation matrix given the link lengths and joint velocities.
 *
 * The input xi is a 6x1 vector representing the joint velocities.
 * The output is a 4x4 matrix representing the end effector transformation.
 *
 * @param L1 - length of link 1
 * @param L2 - length of link 2
 * @param L3 - length of link 3
 * @param xi - twist
 * 
 * @return the end effector transformation matrix
 */
Eigen::Matrix4d LieAlgebra::get_end(double L1, double L2, double L3, Eigen::VectorXd xi){
    Eigen::Matrix<double, 6, 1> first(6), second(6), third(6);
    first << xi(0), xi(1), 0, 0, 0, L1;
    second << xi(2), xi(3), 0, 0, 0, L2;
    third << xi(4), xi(5), 0, 0, 0, L3;
    Eigen::Matrix4d T1 = LieAlgebra::exphat(first);
    Eigen::Matrix4d T2 = LieAlgebra::exphat(second);
    Eigen::Matrix4d T3 = LieAlgebra::exphat(third);
    return T1 * T2 * T3;
}
