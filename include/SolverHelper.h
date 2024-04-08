#ifndef CSC492_SOLVERHELPER_H
#define CSC492_SOLVERHELPER_H

#include <Eigen/Dense>
#include <array>
#include <cmath>

/**
 * @brief Implementation of the SolverHelper class.
 * 
 * The main purpose of this class is to provide methods that help
 * with various operations in the solver. This class is a placeholder
 * for methods that do not fit in the other classes.
 * 
 * @section Credits
 * This code is based on the research paper "An Efficient Multi-solution Solver for the
 * Inverse Kinematics of 3-Section Constant-Curvature Robots"
 * @author Ke Qiu, Jingyu Zhang, Danying Sun, Rong Xiong, Haojian Lu, Yue Wang
 * @code https://sites.google.com/view/micsolver/code
 */
class SolverHelper {
    public:
        /**
        * @brief Computes the linear distance between two ends of a circular arc.
        * 
        * @param a - The cosine of the central angle of the arc.
        * @param L - the length of the circular arc
        *
        * @return the linear distance between the two ends of the arc
        */
        static double rho(double a, double L);
        
        /**
        * @brief Converts the output of our solver to an exponential coordinate.
        *
        * @param L1 - The length of the first link
        * @param L2 - The length of the second link
        * @param L3 - The length of the third link
        *
        * @param soln - The solution to the inverse kinematics problem
        */
        static Eigen::Matrix<double, 6, 1> soln2xi (int L1, int L2, int L3, Eigen::Matrix<double, 9, 1> soln);
        
        /**
        * @brief Computes the model parameter of the 2nd section.
        *
        * This method uses LEMMA 1 and 2 to solve the 2nd section of the Continuum robot.
        */
        static std::array<Eigen::Vector3d, 2> solve_r2(double L1, double L3, Eigen::Quaterniond q, Eigen::Vector3d r, Eigen::Vector3d r3, Eigen::Vector3d r1);
        
        /**
        * @brief Computes the model parameter of the 1st section.
        * 
        * @param L1 - The length of the first link
        * @param q - The orientation of the end effector
        * @param r - The position of the end effector
        * @param r3 - The position of the third link
        * @param noc - The number of corrections
        */
        static Eigen::VectorXd solve_r1(double L1, Eigen::Quaterniond q, Eigen::Vector3d r, Eigen::Vector3d r3, double noc);
        
        /**
        * @brief Helper function for solving the 1st section.
        *
        * This methos uses LEMMA 1 and 2 to solve the 1st section of the Continuum robot.
        */
        static Eigen::Vector3d spp(Eigen::Vector3d n1, double d, Eigen::Vector3d n2, Eigen::Vector3d rn);
        
        /**
        * @brief Computes the error between the desired and computed end effector positions.
        * 
        * @param r1 - The computed position of the first link
        * @param r2 - The computed position of the second link  
        * @param r3 - The computed position of the third link
        * @param L1 - The length of the first link
        * @param L2 - The length of the second link
        * @param L3 - The length of the third link
        * @param q - desired orientation of the end effector
        * @param r - desired position of the end effector
        */
        static double get_err(Eigen::Vector3d r1, Eigen::Vector3d r2, Eigen::Vector3d r3, double L1, double L2, double L3, Eigen::Quaterniond q, Eigen::Vector3d r);
};
#endif //CSC492_SOLVERHELPER_H