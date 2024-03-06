// // set up the class 
// //

// #include <Eigen/Dense>
// #include SolverHelper.h

// class SolverHelper {

//     SolverHelper() {
//     }

//     /*
//     * This method computes the linear distance between two ends of a circular arc.
//     * Furthermore, this method is Lemma 2  in the paper which represents the
//     * Translational constraint.(page 4)
//     * 
//     * NOTE: This method is equivalent to the MATLAB function MISC.rho(a,L)
//     * 
//     * @param a - The angle of the circular arc
//     * @param L - The length of the circular arc
//     */
//     double rho(double a, double L) {
//         if (a==1) return L;
//         double a_square = a*a;
//         double numerator = L * sqrt((1 - a_square));
//         double denominator = acos(a);
//         return numerator / denominator;
//     }

//     /*
//     * SOLN2XI Converts the output of our solver to an exponential coordinate.
//     *
//     * NOTE: This method is equivalent to the MATLAB function MISC.soln2xi(L1, L2, L3, soln)
//     * 
//     * @param L1 - The length of the first link
//     * @param L2 - The length of the second link
//     * @param L3 - The length of the third link
//     * @param soln - The solution to the inverse kinematics problem
//     */
//     Eigen::Matrix<double, 6, 1> soln2xi (int L1, int L2, int L3, Eigen::Matrix<double, 9, 1> soln) {
//         double k1 = 2/L1 * acos(soln(2));
//         double k2 = 2/L2 * acos(soln(5));
//         double k3 = 2/L3 * acos(soln(8));
//         double p1 = atan(soln(1)/soln(0));
//         double p2 = atan(soln(4)/soln(3));
//         double p3 = atan(soln(7)/soln(6));
//         Eigen::Matrix<double, 6, 1> xi;
//         xi << -L1*k1*sin(p1), L1*k1*cos(p1), -L2*k2*sin(p2), L2*k2*cos(p2), -L3*k3*sin(p3), L3*k3*cos(p3);
//         return xi;

//     }

//     Eigen::Matrix4d get_end(double L1, double L2, double L3, Eigen::Matrix<double, 6, 1> xi) {
//         Eigen::Matrix<double, 6, 1> xi1, xi2, xi3;
//         xi1 << xi(0), xi(1), 0, 0, 0, L1;
//         xi2 << xi(2), xi(3), 0, 0, 0, L2;
//         xi3 << xi(4), xi(5), 0, 0, 0, L3;

//         Eigen::Matrix4d T1, T2, T3;
//         T1 = exphat(xi1);
//         T2 = exphat(xi2);
//         T3 = exphat(xi3);

//         Eigen::Matrix4d T = T1 * T2 * T3;

//         return T;
//     }

//     // This function retuns a 6x6 matrix
//     Eigen::MatrixXd jacobian3cc(double L1, double L2, double L3, Eigen::Matrix(double, 6, 1) xi){

//         // T3 = expm(up_hat( [xi(5); xi(6); 0; 0; 0; L3] ));
//         Eigen::Matrix(double, 6, 1) temp;
//         temp << xi(4), xi(5), 0, 0, 0, L3;

//         Eigen::Matrix4d T3 = LieAlgebra::up_hat(temp).expm();

//         // invT3 = [T3(1:3, 1:3).', -T3(1:3, 1:3).' * T3(1:3, 4); 0, 0, 0, 1];

//         Eigen::Matrix4d invT3;
//         invT3 << T3.block(0, 0, 3, 3).transpose(), -T3.block(0, 0, 3, 3).transpose() * T3.block(0, 3, 3, 1),
//             0, 0, 0, 1;

//     }

//     /*
//     * GET_ERR Computes the error between desired and current end pose.
//     *
//     * NOTE: This method is equivalent to the MATLAB function MISC.get_err(r1, r2, r3, L1, L2, L3, q, r)
//     *
//     * @param r1 - The position of the first joint
//     * @param r2 - The position of the second joint
//     * @param r3 - The position of the third joint
//     * @param L1 - The length of the first link
//     * @param L2 - The length of the second link
//     * @param L3 - The length of the third link
//     * @param q - The orientation of the end effector
//     * @param r - The position of the end effector
//     */
//     double get_err(Eigen::Vector3d r1, Eigen::Vector3d r2, Eigen::Vector3d r3, double L1, double L2, double L3, Eigen::Quaterniond q, Eigen::Vector3d r) {
//         Eigen::Matrix4d Td;
//         Td << q.toRotationMatrix(), r,
//             0, 0, 0, 1;
        
//         Eigen::Matrix4d T1;
//         Eigen::Quaterniond q1_temp(r1(2), -r1(1), r1(0), 0); 
//         T1 << q1_temp.toRotationMatrix(), rho(r1(2),L1)*r1,
//             0, 0, 0, 1;

//         Eigen::Matrix4d T2;
//         Eigen::Quaterniond q2_temp(r2(2), -r2(1), r2(0), 0);
//         T2 << q2_temp.toRotationMatrix(), rho(r2(2), L2)*r2,
//             0, 0, 0, 1;
        
//         Eigen::Matrix4d T3;
//         Eigen::Quaterniond q3_temp(r3(2), -r3(1), r3(0), 0);
//         T3 << q3_temp.toRotationMatrix(), rho(r3(2), L3)*r3,
//             0, 0, 0, 1;
        
//         Eigen::Matrix4d Tt = T1 * T2 * T3;

//         Eigen::VectorXd V = LieAlgebra::veelog((Tt.inverse() * Td));
//         //  NOTW: (Tt \ Td) in MATLAB is equivalent to calculating the inverse of Tt and 
//         // then multiplying it by Td
//         return V.norm();
//     }

//     /*
//     * SOLVE_R1 Computes the model parameter of the 1st section.
//     *
//     * NOTE: This method is equivalent to the MATLAB function MISC.solve_r1(L1, L2, L3, q, r, par, noc)
//     *
//     */
//    Eigen::VectorXd solve_r1(double L1, Eigen::Quaterniond q, Eigen::Vector3d r, double par, array<double, 2> noc) {
//       // todo
//    }

//     /*
//     * OLVE_R2 Computes the model parameter of the 2nd section using rotational and translational constraints.
//     *
//     * NOTE: This method is equivalent to the MATLAB function MISC.solve_r2(L1, L3, q, r, r3, r1)
//     */
//    array<Eigen::Vector3d, 2> solve_r2(double L1, double L3, Eigen::Quaterniond q, Eigen::Vector3d r, Eigen::Vector3d r3, Eigen::Vector3d r1) {
//        // todo
//     //    a = q(1); b = q(2); c = q(3); d = q(4);
//     double a = q.w();
//     double b = q.x();
//     double c = q.y();
//     double d = q.z();
//     //     B = [d, a, b; -a, d, c; -b, -c, d];
//     Eigen::Matrix3d B;
//     B << d, a, b, 
//         -a, d, c,
//         -b, -c, d;
//     //     m = [c; -b; a];
//     Eigen::Vector3d m(c, -b, a);

//     //     qe = [m.' * r3; B * r3];
//     Eigen::Vector2d qe;
//     qe << m.transpose() * r3, 
//             B * r3;
   

//    }


  


// };