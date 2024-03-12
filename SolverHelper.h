#ifndef CSC492_SOLVERHELPER_H
#define CSC492_SOLVERHELPER_H

#include <Eigen/Dense>
#include <array>
#include <cmath>

class SolverHelper {
    public:
        /*
        * This method computes the linear distance between two ends of a circular arc.
        * Furthermore, this method is Lemma 2  in the paper which represents the
        * Translational constraint.(page 4)
        * 
        * NOTE: This method is equivalent to the MATLAB function MISC.rho(a,L)
        * 
        * @param a - The angle of the circular arc
        * @param L - The length of the circular arc
        */
        static double rho(double a, double L);
        
        /*
        * SOLN2XI Converts the output of our solver to an exponential coordinate.
        *
        * NOTE: This method is equivalent to the MATLAB function MISC.soln2xi(L1, L2, L3, soln)
        * 
        * @param L1 - The length of the first link
        * @param L2 - The length of the second link
        * @param L3 - The length of the third link
        * @param soln - The solution to the inverse kinematics problem
        */
        static Eigen::Matrix<double, 6, 1> soln2xi (int L1, int L2, int L3, Eigen::Matrix<double, 9, 1> soln);

        static std::array<Eigen::Vector3d, 2> solve_r2(double L1, double L3, Eigen::Quaterniond q, Eigen::Vector3d r, Eigen::Vector3d r3, Eigen::Vector3d r1);

        static Eigen::VectorXd solve_r1(double L1, Eigen::Quaterniond q, Eigen::Vector3d r, Eigen::Vector3d r3, double noc);

        static Eigen::Vector3d spp(Eigen::Vector3d n1, double d, Eigen::Vector3d n2, Eigen::Vector3d rn);
        static double get_err(Eigen::Vector3d r1, Eigen::Vector3d r2, Eigen::Vector3d r3, double L1, double L2, double L3, Eigen::Quaterniond q, Eigen::Vector3d r);
};
#endif //CSC492_SOLVERHELPER_H