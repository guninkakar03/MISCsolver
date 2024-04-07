#ifndef CSC492_CONVERSIONHELPER_H
#define CSC492_CONVERSIONHELPER_H

#include <array>
#include <cmath>
#include <iostream>
#include <Eigen/Dense>

/**
 * @brief Implementation of the ConversionHelpers class.
 * 
 * The main purpose of this class is to provide methods that help
 * converting between arc parameters, Quaternions, and rotation matrices.
 * 
 * @section Credits
 * This code is based on the research paper "An Efficient Multi-solution Solver for the 
 * Inverse Kinematics of 3-Section Constant-Curvature Robots" 
 * @author Ke Qiu, Jingyu Zhang, Danying Sun, Rong Xiong, Haojian Lu, Yue Wang
 * @code https://sites.google.com/view/micsolver/code
 * 
 */
class ConversionHelper {
public:

    /**
    * @brief Converts a Quaternion to a rotation matrix.
    * 
    * This method converts the Quaternion to a rotation matrix.
    * 
    * @param q - the Quaternion to be converted
    * @return the rotation matrix
    */
    static Eigen::Matrix3d q2rot(Eigen::Quaterniond q);

    /**
    * @brief Converts a rotation matrix to a Quaternion.
    * 
    * This method converts a rotation matrix to a Quaternion.
    * 
    * @param R - the rotation matrix to be converted
    * @return the Quaternion
    */
    static Eigen::Quaterniond rot2q(Eigen::Matrix3d R);

    /**
    * @brief Converts a Quaternion to arc parameters.
    * 
    * This method converts a Quaternion to arc parameters of a 1-section constant-curvature robot.
    * 
    * @param q - the Quaternion to be converted
    * @param L - the length of the 1 section of the Continuum robot
    * @return an array containing the curvature and bending angle
    */
    static std::array<double, 2> q2arc(Eigen::Quaterniond q, double L);

    /**
    * @brief Converts arc parameters to a Quaternion.
    * 
    * This method converts arc parameters to a Quaternion representing the end rotation of a 1-section constant-curvature robot.
    * 
    * @param kappa - the curvature of the 1 section of the Continuum robot
    * @param phi - the bending angle of the 1 section of the Continuum robot
    * @param L - the length of the 1 section of the Continuum robot
    * @return the Quaternion
    */
    static Eigen::Quaterniond arc2q(double kappa, double phi, double L);

    /**
    * @brief Converts arc parameters of 3 sections to the exponential coordinate.
    * 
    * This method converts arc parameters of 3 sections to the exponential coordinate of a 3-section constant-curvature robot.
    * 
    * @param L1 - the length of the 1 section of the Continuum robot
    * @param L2 - the length of the 2 section of the Continuum robot
    * @param L3 - the length of the 3 section of the Continuum robot
    * @param arc - an array containing the curvatures and bending angles of each section
    * @return the exponential coordinate
    */
    static Eigen::Matrix<double, 6, 1> arc2xi(int L1, int L2, int L3, Eigen::Matrix<double, 6, 1> arc);
    
    /**
    * @brief Converts the overall exponential coordinate to arc parameters of 3 sections.
    * 
    * This method converts the overall exponential coordinate to arc parameters of 3 sections of a 3-section constant-curvature robot.
    * 
    * @param L1 - the length of the 1 section of the Continuum robot
    * @param L2 - the length of the 2 section of the Continuum robot
    * @param L3 - the length of the 3 section of the Continuum robot
    * @param xi - the overall exponential coordinate
    * @return the arc parameters
    */
    static Eigen::Matrix<double, 6, 1> xi2arc(int L1, int L2, int L3, Eigen::Matrix<double, 6, 1> xi);

};

#endif
