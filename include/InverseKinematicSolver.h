#ifndef CSC492_INVERSEKINEMATICSOLVER_H
#define CSC492_INVERSEKINEMATICSOLVER_H

#include <cmath>
#include <iostream>
#include <Eigen/Dense>

/**
 * @brief Implementation of the InverseKinematicSolver class.
 * 
 * The main purpose of this class is to provide methods that help
 * with solving the inverse kinematics of a 3-section constant-curvature robot using analytical 
 * methods that is based on the research paper.
 * 
 * @section Credits
 * This code is based on the research paper "An Efficient Multi-solution Solver for the 
 * Inverse Kinematics of 3-Section Constant-Curvature Robots" 
 * @author Ke Qiu, Jingyu Zhang, Danying Sun, Rong Xiong, Haojian Lu, Yue Wang
 * @code https://sites.google.com/view/micsolver/code
 */
class InverseKinematicSolver {
    public:

    /**
     * @brief This method solves the inverse kinematics of a 3-section constant-curvature robot.
     * 
     * This method solves the inverse kinematics of a 3-section constant-curvature robot
     * using the Newton-Raphson method. This method will find all possible solutions to the
     * inverse kinematics problem.
     * 
     * @param L1 - the length of the first section
     * @param L2 - the length of the second section
     * @param L3 - the length of the third section
     * @param q - Quaternion representing the orientation of the EE
     * @param r - position of the EE (or translation vector)
     * @param tol - Error tolerance
     * @param msteps - Maximum number of iterations
     * @param noc - Number of corrections
     *
     * @return a matrix containing all possible solutions to the inverse kinematics problem
     *         the matrix has 6 rows and n columns, where n is the number of solutions
     *         each column represents a solution.
     */
    static Eigen::MatrixXd miscSolver(double L1, double L2, double L3, Eigen::Quaterniond q, Eigen::Vector3d r, double par, double tol, int msteps, Eigen::Vector2d noc);

    /**
     * @brief This method does the Candidate selection
     * 
     * @param L1 - the length of the first section
     * @param L2 - the length of the second section
     * @param L3 - the length of the third section
     * @param q - Quaternion representing the orientation of the EE
     * @param r - position of the EE (or translation vector)
     * @param par - Partition size
     * @param noc - correction array
     * 
     * @return a 6 x n matrix containing n possible solutions to the inverse kinematics problem
     */
    static Eigen::MatrixXd miscSolver__(double L1, double L2, double L3, Eigen::Quaterniond q, Eigen::Vector3d r, double par, Eigen::Vector2d noc);
};


#endif //CSC492_INVERSEKINEMATICSOLVER_H
