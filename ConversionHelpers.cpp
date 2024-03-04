#include <array>
#include <cmath>
#include <iostream>
#include <Eigen/Dense>

class ConversionHelper {
public:
    ConversionHelper() {}

    /*
    * This method converts the Queaternion to a rotation matrix.
    * R = Q2ROT(Q) returns the rotation matrix that is equivalent to the quaternion.
    * 
    * NOTE: This method is equivalent to the MATLAB function MISC.q2rot(q)
    *
    * @param q - the quteruinin to be converted 
    */
    Eigen::Matrix3d q2rot(Eigen::Quaterniond q) {
        return q.toRotationMatrix();
    }

    /*
    * This method converts a rotation matrix to Queaternion.
    *  Q = ROT2Q(R) returns the quaternion that is equivalent to the rotation matrix.
    * 
    * NOTE: This method is equivalent to the MATLAB function MISC.rot2q(R)
    *
    * @param R - the rotation matrix to be converted 
    */
    Eigen::Quaterniond rot2q(Eigen::Matrix3d R) {
        return Eigen::Quaterniond(R);
    }

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
    std::array<double, 2> q2arc(Eigen::Quaterniond q, double L) {
        double kappa = (2 * acos(q.w())) / L;
        double phi = atan(-q.x()/q.y()); // Remember to check: This is same as atan2(-b, c) ?
        std::array<double, 2> result = {kappa, phi};
        return result;
    }

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
    Eigen::Quaterniond arc2q(double kappa, double phi, double L) {
        double w = cos(kappa * L / 2);
        double x = sin(kappa * L / 2) * cos(phi);
        double y = sin(kappa * L / 2) * sin(phi);
        double z = 0;
        return Eigen::Quaterniond(w, x, y, z);
    }

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
    // Eigen::Matrix<double, 6, 1> arc2xi(int L1, int L2, int L3, arc // TODO: What is the type of arc?

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
    Eigen::Matrix<double, 6, 1> xi2arc(int L1, int L2, int L3, Eigen::Matrix<double, 6, 1> xi) {
        double k1 = fmod(sqrt(xi(0)*xi(0)+ xi(1)*xi(1)),2*M_PI)/ L1;
        double k2 = fmod(sqrt(xi(2)*xi(2) +xi(3)*xi(3)),2*M_PI) /L2;
        double k3 = fmod( sqrt(xi(4)*xi(4) + xi(5)*xi(5)),2*M_PI)/ L3;

        double phi1 = atan(xi(1)/xi(0));
        double phi2 = atan(xi(3)/xi(2));
        double phi3 = atan(xi(5)/xi(4));

        Eigen::Matrix<double, 6, 1> result;
        result << k1, phi1, k2, phi2, k3, phi3;
        return result;

    }

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
    double xi2len(Eigen::Matrix<double, 6, 1> xi) {
        double r = 84.0/2.0;

        Eigen::Matrix<double, 1, 3> psi_lambda;
        psi_lambda << 0, 4*M_PI/9, 2*M_PI/9;

        Eigen::Matrix<double, 1, 3> psi_mu;
        psi_mu << 0, 2*M_PI/3, 4*M_PI/3;

        Eigen::Matrix<double, 2, 1> delta_1_mu;
        delta_1_mu << r * cos(psi_lambda(0) + psi_mu(0)), r * sin(psi_lambda(0) + psi_mu(0));

        Eigen::Matrix<double, 2, 1> delta_2_mu;
        delta_2_mu << r * cos(psi_lambda(1) + psi_mu(1)), r * sin(psi_lambda(1) + psi_mu(1));

        Eigen::Matrix<double, 2, 1> delta_3_mu;
        delta_3_mu << r * cos(psi_lambda(2) + psi_mu(2)), r * sin(psi_lambda(2) + psi_mu(2));

    }



};
