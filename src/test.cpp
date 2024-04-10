#include <iostream>
#include <Eigen/Dense>
#include <array>
#include "../include/InverseKinematicSolver.h"
#include "../include/ConversionHelpers.h"
#include "../include/LieAlgebra.h"
#include <chrono>

/**
* @brief This function runs the scenarios.
*
* This function runs the scenarios for the given input parameters.
*
* @param L1 - the length of the first section
* @param L2 - the length of the second section
* @param L3 - the length of the third section
* @param alpha - the angle
* @param r - the position of the EE
* @param omega - A unit vector
* @param tol - Error tolerance
*
* @return A matrix containing all possible solutions to the inverse kinematics problem
*/
Eigen::MatrixXd runScenarios_(double L1, double L2, double L3, double alpha, Eigen::Vector3d r, const Eigen::Vector3d& omega, double tol) {
    Eigen::Quaterniond q(cos(alpha / 2), sin(alpha / 2) * omega(0), sin(alpha / 2) * omega(1), sin(alpha / 2) * omega(2));
    Eigen::Vector2d noc;
    noc << 5, 5;
    Eigen::MatrixXd sol = InverseKinematicSolver::miscSolver(L1, L2, L3,  q, r,  0.01,  0.01,  4, noc);
    return sol;
}
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <algorithm>

void test_assert(Eigen::MatrixXd actual, Eigen::MatrixXd expected, double tolerance) {
    if (actual.rows() != expected.rows() || actual.cols() != expected.cols()) {
        std::cout << "Test failed: matrices have different dimensions" << std::endl;
        return;
    }
    std::vector<bool> foundColumns(actual.cols(), false);
    for (int i = 0; i < actual.cols(); i++) {
        for (int j = 0; j < expected.cols(); j++) {
            if (foundColumns[j]) continue; // Skip already found columns
            if ((actual.col(i) - expected.col(j)).norm() < tolerance) {
                foundColumns[j] = true;
                break;
            }
        }
    }
    if (std::all_of(foundColumns.begin(), foundColumns.end(), [](bool found){ return found; })) {
        std::cout << "Test passed" << std::endl;
    } else {
        std::cout << "Test failed: not all expected columns are approximately present in actual matrix" << std::endl;
    }
}


void test_based_on_alpha() {
    double L1 = 1;
    double L2 =  1;
    double L3 = 1;
    Eigen::Vector3d omega;
    omega  << 0.48, sqrt(3)/10., -0.86;
   
    Eigen::Vector3d r;
    r << -0.4, 1.1, 0.8;

    // array of candidate alpha values
    std::array<double, 3> alphas = {6. * M_PI / 21., 17. * M_PI / 21., 5. * M_PI / 15.};
    std::array<Eigen::MatrixXd, 3> solutions;
    for (int i = 0; i < 3; i++) {
        Eigen::MatrixXd sol = runScenarios_(L1, L2, L3, alphas[i], r, omega, 0.01);
        solutions[i] = sol;
    }
    // array of expected solutions
    std::array<Eigen::MatrixXd, 3> expected_solutions = {
        Eigen::MatrixXd::Constant(6, 1, 0.0),
        Eigen::MatrixXd::Constant(6, 1, 0.0),
        Eigen::MatrixXd::Constant(6, 1, 0.0)
    };
    Eigen::MatrixXd expected_solutions1(6, 6);
    expected_solutions1 << -2.8771, -2.8770, -1.7577, -0.2804, -0.8372, -1.8461,
                            -1.5656, -1.5658, -1.0752, -0.0791, -0.4958, -1.1233,
                            1.8577,  1.8573,  5.4839, -2.8068, -0.8924,  1.9250,
                            2.3394,  2.3398,  0.9107, -1.6197,  0.5438,  2.3673,
                            0.4608,  0.4608,  2.0129,  2.3501, -3.8790, -0.9344,
                            0.5523,  0.5523,  2.3914,  3.1870, -1.6950,  5.6248;

    Eigen::MatrixXd expected_solutions2(6, 4);
    expected_solutions2 << -1.7864, -0.3803, -0.4888, -1.0574,
                        -2.1116, -0.2495, -0.5024, -1.4429,
                        -1.3531, -1.8372, -2.2409, -0.5009,
                        1.9581,  0.9326, -1.5459,  3.0588,
                        -0.0410, -1.9918, -1.3941, -4.3191,
                        1.7566, -2.2271,  3.8307, -1.6250;
    
    Eigen::MatrixXd expected_solutions3(6, 6);
    expected_solutions3 <<  -2.8058, -2.8056, -1.8562, -0.3130, -0.8122, -1.7984,
                            -1.6225, -1.6223, -1.2047, -0.0995, -0.5116, -1.1721,
                            1.5882,  1.5880,  5.5184, -2.7452, -0.9407,  1.7221,
                            2.4505,  2.4504, -1.6380, -1.6593,  0.6544,  2.5103,
                            0.4861,  0.4861,  1.7582,  2.0709, -3.7898, -2.4443,
                            0.6598,  0.6596,  2.4046,  3.4516, -1.7425,  5.1276;

    expected_solutions[0] = expected_solutions1;
    expected_solutions[1] = expected_solutions2;
    expected_solutions[2] = expected_solutions3;

    for (int i = 0; i < 3; i++) {
        test_assert(solutions[i], expected_solutions[i], 1e-2);
    }
    
}


void test_based_on_r(){
//     double L1 = 1;
//     double L2 =  1;
//     double L3 = 1;
//     Eigen::Vector3d omega;
//     omega  << 0.48, sqrt(3)/10, -0.86;
//     double alpha = 15 * M_PI / 16;
   

//     // array of candidate r values
//     std::array<Eigen::Vector3d, 3> rs = {Eigen::Vector3d(-0.4, 1.1, 0.8), Eigen::Vector3d(0.1, 0.2, 0.3), Eigen::Vector3d(0.5, 0.6, 0.7)};
//     std::array<Eigen::MatrixXd, 3> solutions;
//     for (int i = 0; i < 1; i++) {
//         Eigen::MatrixXd sol = runScenarios_(L1, L2, L3, alpha, rs[i], omega, 0.01);
//         solutions[i] = sol;
//     }
//     // array of expected solutions
//     std::array<Eigen::MatrixXd, 3> expected_solutions = {
//         Eigen::MatrixXd::Constant(6, 1, 0.0),
//         Eigen::MatrixXd::Constant(6, 1, 0.0),
//         Eigen::MatrixXd::Constant(6, 1, 0.0)
//     };
//     // Eigen::MatrixXd expected_solution
//     Eigen::MatrixXd expected_solutions1(6, 1);
//     expected_solutions1 <<   -0.4745,
//                             1.6102,
//                             4.9098,
//                         -0.0625,
//                         -1.8902,
//                         -0.7074;

//     expected_solutions[0] = expected_solutions1;
//     // expected_solutions[1] = expected_solutions2;
//     // expected_solutions[2] = expected_solutions3;
//     std::cout << "expected_solutions1:\n" << expected_solutions1 << std::endl;
//     std::cout << "solutions[0]:\n" << solutions[0] << std::endl;

//     for (int i = 0; i < 1; i++) {
//         test_assert(solutions[i], expected_solutions[i], 1e-2);
//     }
    
}

void test_avg_runtime(){
    double L1 = 1.0, L2 = 1.0, L3 = 1.0;
    double avgtime = 0.0;

    Eigen::Vector2d noc;
    noc << 5, 5;

    int n = 10000;
    for (int i = 0; i < n; ++i) {
        Eigen::VectorXd angles = M_PI * Eigen::VectorXd::Random(6).cwiseAbs();
        Eigen::VectorXd xi = ConversionHelper::arc2xi(L1, L2, L3, angles);
        Eigen::Matrix4d T = LieAlgebra::get_end(L1, L2, L3, xi);
        Eigen::Quaterniond q = ConversionHelper::rot2q(T.block<3,3>(0,0));
        Eigen::Vector3d r = T.block<3,1>(0,3);
        
        auto start = std::chrono::high_resolution_clock::now();
        Eigen::MatrixXd sol  = InverseKinematicSolver::miscSolver(L1, L2, L3, q, r, 1e-2, 1e-2, 4, noc);
        auto end = std::chrono::high_resolution_clock::now();
        double rt = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        avgtime += rt;

        // std::cout << "A solution is found in " << rt << " ms." << std::endl;
    }

    std::cout << "Average runtime was " << avgtime / static_cast<double>(n) << " ms." << std::endl;
}