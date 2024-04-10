/**
 * @file main.cpp
 * 
 * @brief This file contains the main function that runs the demo.
 * 
 * The main purpose of this file is to demonstrate the usage of the InverseKinematicSolver class 
 * by running a simple example.
 * 
 * @section Credits
 * This code is based on the research paper "An Efficient Multi-solution Solver for the 
 * Inverse Kinematics of 3-Section Constant-Curvature Robots" 
 * @author Ke Qiu, Jingyu Zhang, Danying Sun, Rong Xiong, Haojian Lu, Yue Wang
 * @code https://sites.google.com/view/micsolver/code
 * 
 */



#include <iostream>
#include "../include/LieAlgebra.h"
#include "../include/ConversionHelpers.h"
#include "../include/NumericalMethods.h"
#include "../include/SolverHelper.h"
#include "../include/InverseKinematicSolver.h"
#include <cmath>
#include <array>
#include <unsupported/Eigen/MatrixFunctions>
#include "../include/test.h"

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
Eigen::MatrixXd runScenarios(double L1, double L2, double L3, double alpha, Eigen::Vector3d r, const Eigen::Vector3d& omega, double tol) {
    Eigen::Quaterniond q(cos(alpha / 2), sin(alpha / 2) * omega(0), sin(alpha / 2) * omega(1), sin(alpha / 2) * omega(2));
    Eigen::Vector2d noc;
    noc << 5, 5;
    Eigen::MatrixXd sol = InverseKinematicSolver::miscSolver(L1, L2, L3,  q, r,  0.01,  0.01,  4, noc);
    return sol;
}

/**
 * @brief This function that runs the demo.
 * 
 * This function demonstrates the usage of the InverseKinematicSolver class by running a simple example.
 * This example uses calls the InverseKinematicSolver::miscSolver method to solve the inverse kinematics problem 
 * and finds multiple solutions for the given input parameters.
 */
void main_demo(){
    double L1 = 1;
    double L2 =  1;
    double L3 = 1;
    // double alpha = 6 * M_PI / 21;
    double alpha = 5 * M_PI / 15;
    Eigen::Vector3d omega;
    // omega << 0.011481, -0.939049,-1.738908;
    omega  << 0.48, sqrt(3)/10, -0.86;


    Eigen::Quaterniond q(cos(alpha / 2), sin(alpha / 2) * omega(0), sin(alpha / 2) * omega(1), sin(alpha / 2) * omega(2));
    // std::cout << q.toRotationMatrix() << std::endl;
   
    Eigen::Vector2d noc;
    noc << 5, 5;

    Eigen::Vector3d r;
    r << 0.21, 0.42, 0.8;
    
    Eigen::MatrixXd sol = runScenarios(L1, L2, L3, alpha, r, omega, 0.01);
    std::cout << "sol:\n" << sol << std::endl;
}


void test_code(){
    test_based_on_alpha();
    test_based_on_r();
}

int main() {
    main_demo();
    // test_code();
    return 0;

}
