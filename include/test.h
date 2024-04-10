#ifndef TEST_H
#define TEST_H

#include <iostream>
#include <Eigen/Dense>
#include <array>
#include "InverseKinematicSolver.h"  // Adjust the path according to your project structure

// Function declarations
Eigen::MatrixXd runScenarios_(double L1, double L2, double L3, double alpha, Eigen::Vector3d r, const Eigen::Vector3d& omega, double tol);
void test_assert(Eigen::MatrixXd actual, Eigen::MatrixXd expected);
void test_based_on_alpha();
void test_based_on_r();
void test_avg_runtime();

#endif // TEST_H
