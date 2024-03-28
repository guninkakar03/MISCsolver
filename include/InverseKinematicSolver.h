#ifndef CSC492_INVERSEKINEMATICSOLVER_H
#define CSC492_INVERSEKINEMATICSOLVER_H

#include <cmath>
#include <iostream>
#include <Eigen/Dense>

class InverseKinematicSolver {
    public:

    static Eigen::MatrixXd miscSolver(double L1, double L2, double L3, Eigen::Quaterniond q, Eigen::Vector3d r, double par, double tol, int msteps, Eigen::Vector2d noc);

    static Eigen::MatrixXd miscSolver__(double L1, double L2, double L3, Eigen::Quaterniond q, Eigen::Vector3d r, double par, Eigen::Vector2d noc);
};


#endif //CSC492_INVERSEKINEMATICSOLVER_H
