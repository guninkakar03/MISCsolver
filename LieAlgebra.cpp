#include "LieAlgebra.h"

/*
* This method computes the Lie algebra of a vector.
*  M = UP_HAT(V) is an element of \mathsf{so}_3 or \mathsf{se}_3, where V
*  is an element of \mathbb{R}^3 or \mathbb{R}^6, respectively.
*
* NOTE: This method is equivalent to the MATLAB function MISC.up_hat(v)
*
* @param v - the vector to be converted
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
*/
Eigen::VectorXd LieAlgebra::up_vee(Eigen::MatrixXd M) {
    if (M.rows() == 3) {
        Eigen::Vector3d V;
        V << -M(1, 2), M(0, 2), -M(0, 1);
        return V;
    } else if (M.rows() == 4) {
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
        double theta = acos((R.trace() - 1) / 2);
        Eigen::Matrix3d omega_hat = 1 / (2 * sin(theta)) * (R - R.transpose());
        Eigen::VectorXd V(6);
        V << LieAlgebra::up_vee(omega_hat) * theta,
           (eye - theta/2*omega_hat + (1 - theta/2*(1/tan(theta/2))) * omega_hat * omega_hat) * M.block<3, 1>(0, 3);

        return V;
    }
    return Eigen::VectorXd::Zero(6);
}

/*
* This method does EXPHAT Composition of the hat map and the matrix exponential.
* M = EXPHAT(V) is a matrix in \mathsf{SO}_3 or \mathsf{SE}_3 and is
* computed using Rodrigues' formula. The vector V is in \mathbb{R}^3 or
* \mathbb{R}^4. The hat map sends V to an element of \mathsf{so}_3 or
* \mathsf{se}_3. The vector V is in \mathbb{R}^3 or \mathbb{R}^4.
* The hat map sends V to an element of \mathsf{so}_3 or \mathsf{se}_3.
*
* NOTE: This method is equivalent to the MATLAB function MISC.exphat(V)
*
* @param V - the vector to be converted
*/
Eigen::Matrix4d LieAlgebra::exphat(Eigen::Matrix<double, 6, 1> V) {
    double theta = V.head(3).norm();
    Eigen::Matrix4d M;
    Eigen::Matrix3d eye = Eigen::Matrix3d::Identity();
    if (theta < 2e-8) {
        M << eye, V.tail(3),
            0, 0, 0, 1;
    } else {
        Eigen::Vector3d omega;
        omega = V.head(3) / theta;

        Eigen::Vector3d v;
        v = V.tail(3) / theta;

        Eigen::Matrix3d tempM;
        tempM << eye + sin(theta) * LieAlgebra::up_hat(omega) + (1 - cos(theta)) * (LieAlgebra::up_hat(omega) * LieAlgebra::up_hat(omega));

        Eigen::Vector3d tempV;
        tempV << (eye * theta + (1 - cos(theta)) * LieAlgebra::up_hat(omega) + (theta - sin(theta)) * LieAlgebra::up_hat(omega) * LieAlgebra::up_hat(omega)) * v;
        M << tempM, tempV,
            0, 0, 0, 1;
    }

    return M;
}

// NEED TO CHECK AGAIN IF THESE FUNCTIONS EXIST IN THE EIgen library

Eigen::Matrix4d LieAlgebra::up_plus(Eigen::Quaterniond q){
    double delta = q.w();
    // epsilon = q(2:4);
    Eigen::Vector3d epsilon;
    epsilon << q.x(), q.y(), q.z();
    Eigen::Matrix4d q_up_plus;
    q_up_plus << delta, -epsilon.transpose(),
        epsilon, delta * Eigen::Matrix3d::Identity() + LieAlgebra::up_hat(epsilon);
    return q_up_plus;

}


Eigen::Matrix4d LieAlgebra::up_oplus(Eigen::Quaterniond q){
    double delta = q.w();
    Eigen::Vector3d epsilon;
    epsilon << q.x(), q.y(), q.z();
    Eigen::Matrix4d q_up_oplus;
    q_up_oplus << delta, -epsilon.transpose(),
        epsilon, delta * Eigen::Matrix3d::Identity() - LieAlgebra::up_hat(epsilon);
    return q_up_oplus;
}

Eigen::Quaterniond LieAlgebra::up_star(Eigen::Quaterniond q){
    return Eigen::Quaterniond(q.w(), -q.x(), -q.y(), -q.z());
}

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