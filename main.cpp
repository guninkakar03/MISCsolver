#include <iostream>
#include <Eigen/Dense>
#include "LieAlgebra.h"
#include "ConversionHelpers.h"
#include "NumericalMethods.h"
#include "SolverHelper.h"
#include <cmath>
#include <array>

int main() {
    double L1 = 1;
    double L2 = 1;
    double L3 = 1;
    double alpha = 15 * M_PI / 16;
    Eigen::Vector3d omega;
    omega << 0.48, sqrt(3) / 10, -0.86;
    Eigen::Quaterniond q(cos(alpha / 2), sin(alpha / 2) * omega(0), sin(alpha / 2) * omega(1), sin(alpha / 2) * omega(2));
    Eigen::Vector3d r;
    r << -0.4, 1.1, 0.8;
    Eigen::Matrix<double, 6, 1> TEMP;
    TEMP << 1, 2, 1, 2, 1, 2;
    Eigen::VectorXd RAND(6);
    RAND << 0.8147,0.9058,0.1270,0.9134,0.6324,0.0975;
    Eigen::Matrix<double, 6, 1> TEMP_times_RAND;
    for (int i = 0; i < 6; ++i) {
    TEMP_times_RAND(i) = TEMP(i) * RAND(i);
    }
    Eigen::Matrix<double, 6, 1> xi_0 = ConversionHelper::arc2xi(L1, L2, L3, M_PI * TEMP_times_RAND);
    Eigen::Matrix<double, 6, 1> xi_star;
    // double e = NumericalMethods::revise_dls(L1, L2, L3, q, r, xi_0, 2000, 1e-2, xi_star);
    double e = NumericalMethods::revise_newton(L1, L2, L3, q, r, xi_0, 200, 1e-2, xi_star);
    std::cout << xi_star << std::endl;
    std::cout << e << std::endl;
    return 0;

}





// TESTTING REVISE_DLS AND REVISE_NEWTON
    // double L1 = 1;
    // double L2 = 1;
    // double L3 = 1;
    // double alpha = 15 * M_PI / 16;
    // Eigen::Vector3d omega;
    // omega << 0.48, sqrt(3) / 10, -0.86;
    // Eigen::Quaterniond q(cos(alpha / 2), sin(alpha / 2) * omega(0), sin(alpha / 2) * omega(1), sin(alpha / 2) * omega(2));
    // Eigen::Vector3d r;
    // r << -0.4, 1.1, 0.8;
    // Eigen::Matrix<double, 6, 1> TEMP;
    // TEMP << 1, 2, 1, 2, 1, 2;
    // Eigen::VectorXd RAND(6);
    // RAND << 0.8147,0.9058,0.1270,0.9134,0.6324,0.0975;
    // for (int i = 0; i < 6; ++i) {
    // TEMP_times_RAND(i) = TEMP(i) * RAND(i);
    // }
    // Eigen::Matrix<double, 6, 1> xi_0 = ConversionHelper::arc2xi(L1, L2, L3, M_PI * TEMP_times_RAND);
    // Eigen::Matrix<double, 6, 1> xi_star;
    // double e = NumericalMethods::revise_dls(L1, L2, L3, q, r, xi_0, 2000, 1e-2, xi_star);
    // double e = NumericalMethods::revise_newton(L1, L2, L3, q, r, xi_0, 200, 1e-2, xi_star);
    // std::cout << xi_star << std::endl;
    // std::cout << e << std::endl;



// NEED CODE A FORMULA BEFORE I CONTINUWE
    // Eigen::Vector3d n1;
    // n1 << 0.1,0.2,0.3;

    // Eigen::Vector3d n2;
    // n2 << 0.4,0.5,0.6;

    // Eigen::Vector3d n3; 
    // n3 << 0.7,0.8,0.9;

    // Eigen::Quaterniond q(0.1, 0.2, 0.3, 0.4);

    // // Eigen::Vector3d r = SolverHelper::solve_r1(1, q, n1, n2, 5);