// #include "InverseKinematicSolver.h"
// #include <Eigen/Dense>


// /*
//  * This class is responsible for solving the inverse kinematics
//  * for a 3-Section continuum robot.
//  */
// class InverseKinematicSolver {
// public:
//     // The length of each section of the robot
//     int l1;
//     int l2;
//     int l3;

//     // A qauternion representing the orientation of the end effector
//     Eigen::Quaterniond orientation;

//     // A 3x1 vector representing the position of the end effector
//     Eigen::Vector3d position;

//     /*
//     *  This is the constructor for the InverseKinematicSolver class.
//     *  It takes in the length of each section of the robot, the orientation
//     *  of the end effector, and the position of the end effector.
//     */
//     InverseKinematicSolver(int l1, int l2, int l3, Eigen::Quaterniond orientation, Eigen::Vector3d position) {
//         this->l1 = l1;
//         this->l2 = l2;
//         this->l3 = l3;
//         this->orientation = orientation;
//         this->position = position;
//     }


//     /*
//     * This method acts as a helper function to the inverse kinematics solver.
//     * It takes care of the Canditate Selection step of the inverse kinematics
//     * algorithm.
//     */
//     Eigen::VectorXd candidateSelection(double L1, double L2, double L3, Eigen::Quaterniond q,  Eigen::Vector3d r, double par, array<double, 2> noc) {

//         double a = orientation.w();
//         double b = orientation.x();
//         double c = orientation.y();
//         double d = orientation.z();
        
//         Eigen::Matrix3d B;
//         B << d, a, b, 
//             -a, d, c,
//             -b, -c, d;
        
    
//         Eigen::Vector3d n0 = B.transpose() * r;

        
//         Eigen::Vector3d n = n0 / n0.norm();

//         double scalar = (0.5 + 1.0/M_PI) * (L3 * d) / n0.norm();
//         Eigen::VectorXd r0 = scalar * n;
        
//         double norm_r0 = r0.norm();
//         double norm_r01 = sqrt(1 - norm_r0 * norm_r0);

        
       
//         Eigen::Vector3d u(n(1), -n(0), 0);
//         u = u / u.norm();
//         Eigen::Vector3d v = n.cross(u);
//         Eigen::Vector3d P;
//         P << u, v, n;

        
//         Eigen::VectorXd zeta = partitionInterval(0, 1, partition); // todo: this is not implemented

//         int npar = zeta.size();

//         // todo nan vals to each element of the vector
//         Eigen::VectorXd err(npar), err_r(npar), err_t(npar), is(npar);


//         // solns = nan(9, npar);
//         Eigen::MatrixXd solns(9, npar);

//         int nts = 0;

//         for (int i = 0; i < npar; i++) {
//             double t = zeta(i);
//             // r3 = r0 + norm_r01 .* (P * [sin(2*pi*t); cos(2*pi*t); 0]);
//             Eigen::Vector3d r3 = r0 + norm_r01 * (P * Eigen::Vector3d(sin(2 * M_PI * t), cos(2 * M_PI * t), 0));
//             if (d != 0) { // When d==0, the solution is exact, we do not need corrections.
//                 for (int cor_idx = 0; cor_idx < noc(0); ++cor_idx) {
//                     // One-step correction.
//                    Eigen::Vector3d temp = r3 - (n0.transpose() * r3 - rho(r3(2), L3) * d) / ((n0.transpose() + Eigen::Vector3d(0, 0, L3 * d * (1 / acos(r3(2)) - 1 / sqrt(1 - r3(2) * r3(2))))).transpose() * r0) * r0;
//                     r3 = tmp / tmp.norm();
//                 }
//             }
//         }

//         Eigen::Vector3d r1 = solve_r1(L1, q, r, r3, noc(2));
//         // [r2r, r2t] = solve_r2(L1, L3, q, r, r3, r1);
//         Eigen::Vector3d r2r, r2t;
//         array<Eigen::Vector3d, 2> result = solve_r2(L1, L3, q, r, r3, r1);
//         r2r = result[0];
//         r2t = result[1];

//         // e_r = get_err(r1, r2r, r3, L1, L2, L3, q, r);
//         Eigen::Vector3d e_r = get_err(r1, r2r, r3, L1, L2, L3, q, r);
//         // e_t = get_err(r1, r2t, r3, L1, L2, L3, q, r);
//         Eigen::Vector3d e_t = get_err(r1, r2t, r3, L1, L2, L3, q, r);

//         if(e_r < e_t){
//             // r2 = r2r;
//             // e = e_r;
//             Eigen::Vector3d r2 = r2r;
//             e_t = e_r;
//         } else {
//             // r2 = r2t;
//             // e = e_t;
//             Eigen::Vector3d r2 = r2t;
//             e_t = e_t;
//         }

//         if (i == 0) {
//             // No operation here.
//         } else if (i == 1) {
//             nts++;
//             solns.col(nts - 1) = Eigen::Vector9d(r1(0), r1(1), r1(2), r2(0), r2(1), r2(2), r3(0), r3(1), r3(2));
//             is(nts - 1) = 1;
//         } else { // i > 1
//             if (e > err(i - 1) && err(i - 1) <= err(i - 2)) { // Local minimum.
//                 nts++;
//                 solns.col(nts - 1) = Eigen::Vector9d(r1(0), r1(1), r1(2), r2(0), r2(1), r2(2), r3(0), r3(1), r3(2));
//                 is(nts - 1) = i - 1;
//             }
//         }
//         Eigen::Vector9d soln(r1(0), r1(1), r1(2), r2(0), r2(1), r2(2), r3(0), r3(1), r3(2));
//         err(i) = e;
//         err_r(i) = e_r;
//         err_t(i) = e_t;

//         // todo remaining code
//         // ts = zeta(is);
//         // [~, idx] = sort(abs(ts - 0.5));
//         // solns = solns(:, idx);
//     }


//     /*
//     * This method helps to partition a given interval into some 
//     * number of subintervals.
//     */
//     Eigen::VectorXd partitionInterval(double start, double end, double partition) {
//         Eigen::VectorXd partitionInterval;
//         int j = 0;
//         for (int i = 0; i < end; i+= partition) { // The candidate selection is done for t in [0,1)
//             j++; // todo
//         }
//         return partitionInterval;
//     }
// };
