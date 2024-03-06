// #include <Eigen/Dense>
// #include SolverHelper

// class NumericalMethods {

//     NumericalMethods() {
//     }

//     void revise_newton(double L1, double L2, double L3, Eigen::Quaterniond q, Eigen::Vector3d r, Eigen::Matrix<double, 6, 1> xi, int msteps, double tol) {
        
//         Eigen::Matrix4d Td;
//         Td << q.toRotationMatrix(), r,
//             0, 0, 0, 1;
        
        
//         Eigen::VectorXd omg_e(msteps);
//         omg_e.setConstant(NAN);
        
      
//         Eigen::VectorXd v_e(msteps);
//         v_e.setConstant(NAN);


//         Eigen::VectorXd e(msteps);
//         e.setConstant(NAN);

//         int k = 0;

//         while (k < msteps){
//             Eigen::Matrix4d Tt = SolverHelper::get_end(L1, L2, L3, xi);
//             //  V = up_vee(logm(Tt \ Td));
//             Eigen::Matrix4d V = SolverHelper::up_vee((Tt.inverse() * Td)).log();

//             // omg_e(k+1) = norm(V(1: 3));
//             omg_e(k) = V.head(3).norm();

//             // v_e(k+1) = norm(V(4: 6));
//             v_e(k) = V.tail(3).norm();

//             // e(k+1) = norm(V);
//             e(k) = V.norm();

//             if (e(k) < tol) {
//                 break;
//             } else {
//                 // do jacobian
//                 // xi = xi + (J.' * J) \ J' * V;
//                 Eigen::Matrix<double, 6, 6> J = SolverHelper::get_jacobian(L1, L2, L3, xi);
//                 // xi = xi + (J.' * J) \ J' * V;
//                 xi = xi + (J.transpose() * J).inverse() * J.transpose() * V;
//                 // xi = arc2xi(L1, L2, L3, xi2arc(L1, L2, L3, xi));
//                 xi = SolverHelper::arc2xi(L1, L2, L3, SolverHelper::xi2arc(L1, L2, L3, xi));
//                 k += 1;
//             }


//         }

//         if(k == msteps) {
//             Eigen::Matrix4d Tt = SolverHelper::get_end(L1, L2, L3, xi);
//             Eigen::Matrix4d V = SolverHelper::up_vee((Tt.inverse() * Td));
//             omg_e(k) = V.head(3).norm();
//             v_e(k) = V.tail(3).norm();
//             e(k) = V.norm();
//         }


//     }


// };