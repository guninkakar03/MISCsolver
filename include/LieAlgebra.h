#ifndef CSC492_LIEALGEBRA_H
#define CSC492_LIEALGEBRA_H

#include <cmath>
#include <iostream>
#include <Eigen/Dense>


/**
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

class LieAlgebra {
public:

 
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
    static Eigen::MatrixXd up_hat(Eigen::VectorXd V);

    /**
    * @brief This method computes the VEE operation of a skew-symmetric matrix.
    * 
    * This method converts a skew-symmetric matrix to a vector.
    * If the input matrix is a 3x3 matrix, the output is a 3x1 vector.
    * If the input matrix is a 4x4 matrix, the output is a 6x1 vector.
    *
    * @param M - the matrix to be converted
    * @return the VEE vector
    */
    static Eigen::VectorXd up_vee(Eigen::MatrixXd M);

    /**
    * @brief This method computes the Composition of the matrix logarithm.
    * 
    * VEELOG is a function that computes the composition of the matrix logarithm using
    * Rodrigues' formula. The function also VEE maps the an element of \mathsf{so}_3 or
    * \mathsf{se}_3 to a vector.
    *
    * @param M - the matrix to be converted
    * @return the VEE vector
    */
    static Eigen::VectorXd veelog(Eigen::MatrixXd M);

    /**
    * @brief This method computes the Composition of the hat map and the matrix exponential.
    *
    * @param V - the Lie algebra vector
    * @return the matrix exponential
    */ 
    static Eigen::Matrix4d exphat(Eigen::Matrix<double, 6, 1> V);

    /**
    * @brief This method computes the UP_PLUS operation of a quaternion.
    * 
    * @param q - a unit quaternion
    * @return a 4x4 matrix representing the UP_PLUS operation
    */
    static Eigen::Matrix4d up_plus(Eigen::Quaterniond q);

    static Eigen::Matrix4d up_oplus(Eigen::Quaterniond q);

    /**
    * @brief This method computes the UP_STAR operation of a quaternion.
    *
    * @param q - a unit quaternion
    * @return the UP_STAR quaternion
    */
    static Eigen::Quaterniond up_star(Eigen::Quaterniond q);

    /**
    * @brief This method computes the end effector transformation matrix given the link lengths and twist.
    *
    *
    * @param L1 - length of link 1
    * @param L2 - length of link 2
    * @param L3 - length of link 3
    * @param xi - twist
    * 
    * @return the end effector transformation matrix
    */
    static Eigen::Matrix4d get_end(double L1, double L2, double L3, Eigen::VectorXd xi);

};
#endif //CSC492_LIEALGEBRA_H