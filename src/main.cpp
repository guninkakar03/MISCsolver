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


/**
 * @brief This function that runs the demo.
 * 
 * This function demonstrates the usage of the InverseKinematicSolver class by running a simple example.
 * This example uses calls the InverseKinematicSolver::miscSolver method to solve the inverse kinematics problem 
 * and finds multiple solutions for the given input parameters.
 *
 */
void main_demo(){
    double L1 = 1;
    double L2 =  1;
    double L3 = 1;
    double alpha = 16 * M_PI / 16;
    Eigen::Vector3d omega;
    // omega << 0.011481, -0.939049,-1.738908;
    omega  << 0.48, sqrt(3)/10, -0.86;


    Eigen::Quaterniond q(cos(alpha / 2), sin(alpha / 2) * omega(0), sin(alpha / 2) * omega(1), sin(alpha / 2) * omega(2));
    // std::cout << q.toRotationMatrix() << std::endl;
   
    Eigen::Vector2d noc;
    noc << 5, 5;

    Eigen::Vector3d r;
    r << -0.4, 1.1, 0.8;

    Eigen::MatrixXd sol = InverseKinematicSolver::miscSolver(L1, L2, L3,  q, r,  0.01,  0.01,  4, noc);
    std::cout << "sol: " << sol << std::endl;
}



int main() {
    main_demo();
    return 0;

}
