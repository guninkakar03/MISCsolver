#ifndef CSC492_CONVERSIONHELPER_H
#define CSC492_CONVERSIONHELPER_H

#include <array>
#include <cmath>
#include <iostream>
#include <Eigen/Dense>
// static std::array<double, 2> q2arc(Eigen::Quaterniond q, double L);

class ConversionHelper {
public:

    /*
    * This method converts the Queaternion to a rotation matrix.
    * R = Q2ROT(Q) returns the rotation matrix that is equivalent to the quaternion.
    * 
    * NOTE: This method is equivalent to the MATLAB function MISC.q2rot(q)
    *
    * @param q - the quteruinin to be converted 
    */
    static Eigen::Matrix3d q2rot(Eigen::Quaterniond q);

    /*
    * This method converts a rotation matrix to Queaternion.
    *  Q = ROT2Q(R) returns the quaternion that is equivalent to the rotation matrix.
    * 
    * NOTE: This method is equivalent to the MATLAB function MISC.rot2q(R)
    *
    * @param R - the rotation matrix to be converted 
    */
    static Eigen::Quaterniond rot2q(Eigen::Matrix3d R);

    /*
    * This method Converts a quaternion to arc parameters.
    * ARC = Q2ARC(Q, L) computes the arc parameters of a 1-section constant-curvature robot,
    * including the curvature KAPPA and bending angle PHI. The section length is L. 
    * 
    * NOTE: This method is equivalent to the MATLAB function MISC.q2arc(q, L)
    *
    * @param q - the quaternion to be converted
    * @param L - the length of the  1 section of Continuum robot
    */
    static std::array<double, 2> q2arc(Eigen::Quaterniond q, double L);

    /*
    * This method converts arc parameters to a quaternion.
    * Q = ARC2Q(KAPPA, PHI, L) computes the quaternion Q representing the end 
    * rotation of a 1-section constant-curvature robot with curvature KAPPA,
    * bending angle PHI, and section length L.
    *
    * NOTE: This method is equivalent to the MATLAB function MISC.arc2q(kappa, phi, L)
    *
    * @param kappa - the curvature of the 1 section of Continuum robot
    * @param phi - the bending angle of the 1 section of Continuum robot
    */
    static Eigen::Quaterniond arc2q(double kappa, double phi, double L);

    /*
    * This method converts the arc parameters of 3 sections to the exponential coordinate.
    * XI = ARC2XI(L1, L2, L3, ARC) computes the exponential coordinate XI
    * a 3-section constant-curvature robot. The section lengths are L1, L2
    * and L3, respectively. The parameter ARC is an array containing
    * curvatures and bending angles of each section.
    *
    * NOTE: This method is equivalent to the MATLAB function MISC.arc2xi(L1, L2, L3, arc)
    *
    * @param L1 - the length of the 1 section of Continuum robot
    * @param L2 - the length of the 2 section of Continuum robot
    * @param L3 - the length of the 3 section of Continuum robot
    */
    static Eigen::Matrix<double, 6, 1> arc2xi(int L1, int L2, int L3, Eigen::Matrix<double, 6, 1> arc);

    /*
    * This method converts the overall exponential coordinate to the arc parameters of 3 sections.
    *   ARC = XI2ARC(L1, L2, L3, XI) computes the arc parameter ARC of a
    *   3-section constant-curvature robot. The section lengths are L1, L2 and
    *   L3, respectively. The parameter XI is the overall exponential
    *   coordinate.
    *
    * NOTE: This method is equivalent to the MATLAB function MISC.xi2arc(L1, L2, L3, xi)
    *
    * @param L1 - the length of the 1 section of Continuum robot
    * @param L2 - the length of the 2 section of Continuum robot
    * @param L3 - the length of the 3 section of Continuum robot
    * @param xi - the overall exponential coordinate
    */
    static Eigen::Matrix<double, 6, 1> xi2arc(int L1, int L2, int L3, Eigen::Matrix<double, 6, 1> xi);

    /*
    * This method converts the concatenated parameter to the actuator lengths.
    * LEN = XI2LEN(XI) computes the actuator lengths LEN of a 3-section
    * constant-curvature robot. The concatenated parameter XI is defined in
    * our article.
    *
    * NOTE: This method is equivalent to the MATLAB function MISC.xi2len(xi)
    *
    * @param xi - the concatenated parameter
    */
    // static Eigen::VectorXd xi2len(Eigen::Matrix<double, 6, 1> xi);


};

#endif
