#ifndef CSC492_NUMERICALMETHODS_H
#define CSC492_NUMERICALMETHODS_H

#include <cmath>
#include <iostream>
#include <Eigen/Dense>

/**
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
class NumericalMethods{
    public:

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
    static double revise_newton(double L1, double L2, double L3, Eigen::Quaterniond q, Eigen::Vector3d r, Eigen::Matrix<double, 6, 1> xi, int msteps, double tol, Eigen::Matrix<double, 6, 1>& xi_star, int& iterations);

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
    static double revise_dls(double L1, double L2, double L3, Eigen::Quaterniond q, Eigen::Vector3d r, Eigen::Matrix<double, 6, 1> xi, int msteps, double tol, Eigen::Matrix<double, 6, 1>& xi_star);

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
    static Eigen::MatrixXd jaco_c12(double w1, double w2, double L);

    /**
    * @brief The function is responsible for computing the sub-Jacobian matrix for an individual section of the continuum robot.
    *
    * @param w1 -  The angular velocities or bending angles of the section.
    * @param w2 -  The angular velocities or bending angles of the section.
    * @param L - the length of the section
    * 
    * @return a 6x3 Jacobian matrix
    */
    static Eigen::MatrixXd jacobian3cc(double L1, double L2, double L3, Eigen::Matrix<double, 6, 1> xi);

};

#endif // CSC492_NUMERICALMETHODS_H