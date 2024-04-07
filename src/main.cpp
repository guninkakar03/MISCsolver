#include <iostream>
#include "../include/LieAlgebra.h"
#include "../include/ConversionHelpers.h"
#include "../include/NumericalMethods.h"
#include "../include/SolverHelper.h"
#include "../include/InverseKinematicSolver.h"
#include <cmath>
#include <array>
#include <unsupported/Eigen/MatrixFunctions>




int main() {
    // 6.201305, 0.011481, -0.939049, -1.738908,
    double L1 = 1;
    double L2 =  1;
    double L3 = 1;
    // double alpha = 15 * M_PI / 16;
    double alpha = 6.201305;
    Eigen::Vector3d omega;
    omega << 0.011481, -0.939049,-1.738908;


    Eigen::Quaterniond q(cos(alpha / 2), sin(alpha / 2) * omega(0), sin(alpha / 2) * omega(1), sin(alpha / 2) * omega(2));
    // std::cout << q.toRotationMatrix() << std::endl;
   
    Eigen::Vector2d noc;
    noc << 5, 5;

    Eigen::Vector3d r;
    r << -0.4, 1.1, 0.8;

    Eigen::MatrixXd sol = InverseKinematicSolver::miscSolver(L1, L2, L3,  q, r,  0.01,  0.1,  4, noc);
    std::cout << "sol: " << sol << std::endl;

    return 0;

}
// THESES ARE ALL XI_0
// NOTE: THIS VALUE CONVERGES PERFECLTY FOR 5 iteration
//   -0.4311 
//    -0.6251
//    -0.2234
//     0.2047
//     0.8054
//     2.8478
//
//  NOTE: THIS VALUE CONVERGES PERFECLTY FOR 7 iteration
//    -1.6462
//    -0.4931
//    -2.1695
//     0.8755
//    -1.9720
//     0.8754
// NOTE: This VALUE converge to correct values of xi but not iteration
//   -0.2821
//     0.2860
//    -2.1644
//     0.3984
//    -1.0853
//    -1.8964





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





    // MOST RECENT VERSION OF TEST:
    // double L1 = 1;
    // double L2 = 1;
    // double L3 = 1;
    // double alpha = 15 * M_PI / 16;
    // Eigen::Vector3d omega;
    // omega << 0.48, sqrt(3) / 10, -0.86;
    // Eigen::Quaterniond q(cos(alpha / 2), sin(alpha / 2) * omega(0), sin(alpha / 2) * omega(1), sin(alpha / 2) * omega(2));
    // Eigen::Vector3d r;
    // r << -0.4, 1.1, 0.8;
    // Eigen::VectorXd TEMP(6);
    // TEMP << 1, 2, 1, 2, 1, 2;
    // Eigen::VectorXd RAND(6);
    // RAND << 0.8147,0.9058,0.1270,0.9134,0.6324,0.0975;
    // // Eigen::Matrix<double, 6, 1> TEMP_times_RAND;
    // // // for (int i = 0; i < 6; ++i) {
    // // // TEMP_times_RAND(i) = TEMP(i) * RAND(i);
    // // // }
    // Eigen::Matrix<double, 6, 1> xi_0 = ConversionHelper::arc2xi(L1, L2, L3, M_PI * TEMP.cwiseProduct(RAND));
    // std:: cout <<  M_PI * TEMP.cwiseProduct(RAND) << std::endl;
    // Eigen::Matrix<double, 6, 1> xi_star;
    // // // double e = NumericalMethods::revise_dls(L1, L2, L3, q, r, xi_0, 2000, 1e-2, xi_star);
    // double e = NumericalMethods::revise_newton(L1, L2, L3, q, r, xi_0, 200, 1e-2, xi_star);
    // std::cout << xi_star << std::endl;
    // // std::cout << e << std::endl;




    //     // code to check MISCSOLVER
    //     double L1 = 1;
    // double L2 = 1;
    // double L3 = 1;
    // double alpha = 15 * M_PI / 16;
    // Eigen::Vector3d omega;
    // omega << 0.48, sqrt(3) / 10, -0.86;
    // Eigen::Quaterniond q(cos(alpha / 2), sin(alpha / 2) * omega(0), sin(alpha / 2) * omega(1), sin(alpha / 2) * omega(2));
    // // std::cout << "q : " << q << std::endl;
    // Eigen::Vector3d r;
    // r << -0.4, 1.1, 0.8;
    // Eigen::Vector2d noc;
    // noc << 5, 5;
    // Eigen::MatrixXd sol = InverseKinematicSolver::miscSolver(L1, L2, L3,  q, r,  0.25,  0.01,  4, noc);
    // std::cout << "sol: " << sol<< std::endl;
    // return 0;



   
    //         // MOST RECENT VERSION OF TEST: // SUN MAR 17, 4:35
    // double L1 = 1;
    // double L2 = 1;
    // double L3 = 1;
    // double alpha = 15 * M_PI / 16;
    
   
    // Eigen::Vector3d omega;
    // omega << 0.48, sqrt(3) / 10, -0.86;
    
    // Eigen::Quaterniond q(cos(alpha / 2), sin(alpha / 2) * omega(0), sin(alpha / 2) * omega(1), sin(alpha / 2) * omega(2));
    // Eigen::Vector3d r;
    // r << -0.4, 1.1, 0.8;
    // Eigen::VectorXd TEMP(6);
    // TEMP << 1, 2, 1, 2, 1, 2;
    // Eigen::VectorXd RAND(6);
    // RAND << 0.8147,0.9058,0.1270,0.9134,0.6324,0.0975;
    // // Eigen::Matrix<double, 6, 1> TEMP_times_RAND;
    // // // for (int i = 0; i < 6; ++i) {
    // // // TEMP_times_RAND(i) = TEMP(i) * RAND(i);
    // // // }
    // Eigen::Matrix<double, 6, 1> xi_0 = ConversionHelper::arc2xi(L1, L2, L3, M_PI * TEMP.cwiseProduct(RAND));
    // std:: cout <<  M_PI * TEMP.cwiseProduct(RAND) << std::endl;
    // Eigen::Matrix<double, 6, 1> xi_star;
    // // // double e = NumericalMethods::revise_dls(L1, L2, L3, q, r, xi_0, 2000, 1e-2, xi_star);
    // int k;
    // double e = NumericalMethods::revise_newton(L1, L2, L3, q, r, xi_0, 200, 1e-2, xi_star,k);
    // std::cout << xi_star << std::endl;
    // std::cout << e <<  std::endl;
    // std::cout << k << std::endl;
    // return 0;



    // // DEBUGGING SOLVE_R1
    // double alpha = 15 * M_PI / 16;
    // Eigen::Vector3d omega;
    // omega << 0.48, sqrt(3) / 10, 0.86;
    // Eigen::Quaterniond q(cos(alpha / 2), sin(alpha / 2) * omega(0), sin(alpha / 2) * omega(1), sin(alpha / 2) * omega(2));    
    // Eigen::Vector3d r(0.5, 0.6, 0.7);
    // Eigen::Vector3d r3(0.8, 0.9, 1.0);
    // Eigen::Vector3d r1(1.1, 1.2, 1.3);
    //  Eigen::VectorXd sol = SolverHelper::solve_r1(1,q,r,r3,1);
    // // std::cout << "r2r" << sol[0] << std::endl;
    // std::cout << sol << std::endl;
    // return 0;