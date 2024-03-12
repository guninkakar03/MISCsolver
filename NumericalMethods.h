#ifndef CSC492_NUMERICALMETHODS_H
#define CSC492_NUMERICALMETHODS_H

#include <cmath>
#include <iostream>
#include <Eigen/Dense>

class NumericalMethods{
    public:

    static double revise_newton(double L1, double L2, double L3, Eigen::Quaterniond q, Eigen::Vector3d r, Eigen::Matrix<double, 6, 1> xi, int msteps, double tol, Eigen::Matrix<double, 6, 1>& xi_star);

    static double revise_dls(double L1, double L2, double L3, Eigen::Quaterniond q, Eigen::Vector3d r, Eigen::Matrix<double, 6, 1> xi, int msteps, double tol, Eigen::Matrix<double, 6, 1>& xi_star);

    
    static Eigen::MatrixXd jaco_c12(double w1, double w2, double L);

    static Eigen::MatrixXd jacobian3cc(double L1, double L2, double L3, Eigen::Matrix<double, 6, 1> xi);

};

#endif // CSC492_NUMERICALMETHODS_H