/**
 * @file ConversionHelpers.cpp
 * 
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

#include "../include/ConversionHelpers.h"

/**
 * @brief Converts a Quaternion to a rotation matrix.
 * 
 * This method converts the Quaternion to a rotation matrix.
 * 
 * @param q - the Quaternion to be converted
 * @return the rotation matrix
 */
Eigen::Matrix3d ConversionHelper::q2rot(Eigen::Quaterniond q) {
    return q.toRotationMatrix();
}

/**
 * @brief Converts a rotation matrix to a Quaternion.
 * 
 * This method converts a rotation matrix to a Quaternion.
 * 
 * @param R - the rotation matrix to be converted
 * @return the Quaternion
 */
Eigen::Quaterniond ConversionHelper::rot2q(Eigen::Matrix3d R) {
    return Eigen::Quaterniond(R);
}

/**
 * @brief Converts a Quaternion to arc parameters.
 * 
 * This method converts a Quaternion to arc parameters of a 1-section constant-curvature robot.
 * 
 * @param q - the Quaternion to be converted
 * @param L - the length of the 1 section of the Continuum robot
 * @return an array containing the curvature and bending angle
 */
std::array<double, 2> ConversionHelper::q2arc(Eigen::Quaterniond q, double L) {
    double kappa = (2.0 / L) * acos(q.w());
    double phi = atan2(-q.x(), q.y());
    std::array<double, 2> result = {kappa, phi};
    return result;
}

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
Eigen::Quaterniond ConversionHelper::arc2q(double kappa, double phi, double L) {
    double w = cos(kappa * (L / 2.0));
    double x = -sin(kappa * (L / 2.0)) * sin(phi);
    double y = sin(kappa * (L / 2.0)) * cos(phi);
    double z = 0.0;
    return Eigen::Quaterniond(w, x, y, z);
}

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
Eigen::Matrix<double, 6, 1> ConversionHelper::arc2xi(int L1, int L2, int L3, Eigen::Matrix<double, 6, 1> arc) {
    Eigen::Matrix<double, 6, 1> xi;
    xi << -L1 * arc(0) * sin(arc(1)),
           L1 * arc(0) * cos(arc(1)),
          -L2 * arc(2) * sin(arc(3)),
           L2 * arc(2) * cos(arc(3)),
          -L3 * arc(4) * sin(arc(5)),
           L3 * arc(4) * cos(arc(5));
    return xi;
}

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
Eigen::Matrix<double, 6, 1> ConversionHelper::xi2arc(int L1, int L2, int L3, Eigen::Matrix<double, 6, 1> xi) {
    double k1 = fmod(sqrt(pow(xi(0), 2.0) + pow(xi(1), 2.)), 2. * M_PI) / L1;
    double k2 = fmod(sqrt(pow(xi(2), 2.0) + pow(xi(3), 2.)), 2. * M_PI) / L2;
    double k3 = fmod(sqrt(pow(xi(4), 2.0) + pow(xi(5), 2.)), 2. * M_PI) / L3;

    double phi1 = atan2(-xi(0), xi(1));
    double phi2 = atan2(-xi(2), xi(3));
    double phi3 = atan2(-xi(4), xi(5));

    Eigen::Matrix<double, 6, 1> result;
    result << k1, phi1, k2, phi2, k3, phi3;
    return result;
}






