#include <Eigen/Dense>

#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

Eigen::MatrixXd up_hat(const Eigen::VectorXd& V) {
    Eigen::Matrix4d M = Eigen::Matrix4d::Zero();

    if (V.size() == 3) {
        M <<    0,  -V(2),   V(1),   0,
              V(2),      0,  -V(0),   0,
             -V(1),   V(0),      0,   0,
                 0,      0,      0,   0;
    } else if (V.size() == 6) {
        M <<    0,  -V(2),   V(1),   V(3),
              V(2),      0,  -V(0),   V(4),
             -V(1),   V(0),      0,   V(5),
                 0,      0,      0,      0;
    } else {
        throw std::invalid_argument("Input not in R3/R6");
    }

    return M;
}

Eigen::Matrix4d exphat(const Eigen::VectorXd& V) {
    Eigen::Matrix4d M = Eigen::Matrix4d::Zero();  // Initialize M to zero
    double theta = V.head(3).norm();
    if (theta < 2e-8) {
        M.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();  // Set the top-left 3x3 block to identity
        M.block<3, 1>(0, 3) = V.tail(3);                    // Set the top-right 3x1 block to the last three elements of V
        M(3, 3) = 1;                                        // Set the bottom-right element to 1
    } else {
        Eigen::Vector3d omega = V.head(3) / theta;
        Eigen::Vector3d v = V.tail(3) / theta;
        Eigen::Matrix3d omega_hat = up_hat(omega);
        Eigen::Matrix3d A = Eigen::Matrix3d::Identity() + (sin(theta) * omega_hat) + ((1 - cos(theta)) *(omega_hat * omega_hat));
        Eigen::Vector3d b = (Eigen::Matrix3d::Identity() * theta + (1 - cos(theta)) * omega_hat + (theta - sin(theta)) * (omega_hat * omega_hat)) * v;
        M.block<3, 3>(0, 0) = A;  // Set the top-left 3x3 block to A
        M.block<3, 1>(0, 3) = b;  // Set the top-right 3x1 block to b
        M(3, 3) = 1;              // Set the bottom-right element to 1
    }
    return M;
}

Eigen::Matrix4d get_end(double L1, double L2, double L3, const Eigen::VectorXd& xi) {
    Eigen::Matrix4d T1 = exphat(Eigen::VectorXd::Map(xi.data(), 6, 1) << xi(0), xi(1), 0, 0, 0, L1);
    Eigen::Matrix4d T2 = exphat(Eigen::VectorXd::Map(xi.data() + 2, 6, 1) << xi(2), xi(3), 0, 0, 0, L2);
    Eigen::Matrix4d T3 = exphat(Eigen::VectorXd::Map(xi.data() + 4, 6, 1) << xi(4), xi(5), 0, 0, 0, L3);

    return T1 * T2 * T3;
}



// Eigen::VectorXd up_vee(const Eigen::Matrix4d& M) {
//     if (M.rows() == 3 && M.cols() == 3) {
//         return Eigen::VectorXd(3) << -M(1, 2), M(0, 2), -M(0, 1);
//     } else if (M.rows() == 4 && M.cols() == 4) {
//         return Eigen::VectorXd(6) << -M(1, 2), M(0, 2), -M(0, 1), M(0, 3), M(1, 3), M(2, 3);
//     } else {
//         throw std::invalid_argument("Input not in so/se3");
//     }
// }


Eigen::MatrixXd jaco_c12(double w1, double w2, double L) {
    Eigen::Vector3d w(w1 + w2, 0, 0);
    double n = w.norm();
    Eigen::MatrixXd Jc(6, 2);

    if (n == 0) {
        double ML = L / 2;
        Jc << Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, ML, 0),
              Eigen::Vector3d(-ML, 0, 0), Eigen::Vector3d::Zero().transpose(),
              Eigen::Vector3d::Zero().transpose(), Eigen::Vector3d::Zero().transpose();
    } else {
        double M = (1 - cos(n)) / (n * n);
        double N = (n - sin(n)) / (n * n * n);
        double p1M = w1 / (n * n) - w1 * N - 2 * w1 * M / (n * n);
        double p2M = w2 / (n * n) - w2 * N - 2 * w2 * M / (n * n);
        double p1N = w1 * M / (n * n) - 3 * w1 * N / (n * n);
        double p2N = w2 * M / (n * n) - 3 * w2 * N / (n * n);

        Eigen::Matrix3d pwJleftwv;
        pwJleftwv <<   p1M * w2,   p2M * w2 + M,    0,
                      -p1M * w1 - M,   -p2M * w1,    0,
                      -p1N * n * n - 2 * N * w1,    -p2N * n * n - 2 * N * w2,    0;

        Eigen::Matrix3d w_hat;
        w_hat <<    0,  -w(2),   w(1),
                  w(2),      0,  -w(0),
                 -w(1),   w(0),      0;

        Jc << Eigen::Matrix3d::Identity() - M * w_hat + N * (w_hat * w_hat),
              std::exp(w_hat.transpose()) * pwJleftwv;
    }

    return Jc;
}


// Eigen::MatrixXd jacobian3cc(double L1, double L2, double L3, const Eigen::VectorXd& xi) {
//     Eigen::MatrixXd J(6, 6);

//     Eigen::MatrixXd j3 = jaco_c12(xi(4), xi(5), L3);

//     Eigen::Matrix4d T3 = exphat(xi.segment(4, 2) << xi(4), xi(5), 0, 0, 0, L3);
//     Eigen::Matrix4d invT3;
//     invT3 << T3.block<3, 3>(0, 0).transpose(), -T3.block<3, 3>(0, 0).transpose() * T3.block<3, 1>(0, 3),
//              0, 0, 0, 1;
//     Eigen::MatrixXd j2 = jaco_c12(xi(2), xi(3), L2);
//     Eigen::VectorXd j2_c1 = up_vee(invT3 * up_hat(j2.col(0)) * T3);
//     Eigen::VectorXd j2_c2 = up_vee(invT3 * up_hat(j2.col(1)) * T3);
//     j2.resize(6, 2);
//     j2 << j2_c1, j2_c2;

//     Eigen::Matrix4d T2 = exphat(xi.segment(2, 2) << xi(2), xi(3), 0, 0, 0, L2);
//     Eigen::Matrix4d invT2;
//     invT2 << T2.block<3, 3>(0, 0).transpose(), -T2.block<3, 3>(0, 0).transpose() * T2.block<3, 1>(0, 3),
//              0, 0, 0, 1;
//     Eigen::MatrixXd j1 = jaco_c12(xi(0), xi(1), L1);
//     Eigen::VectorXd j1_c1 = up_vee(invT3 * invT2 * up_hat(j1.col(0)) * T2 * T3);
//     Eigen::VectorXd j1_c2 = up_vee(invT3 * invT2 * up_hat(j1.col(1)) * T2 * T3);
//     j1.resize(6, 2);
//     j1 << j1_c1, j1_c2;

//     J << j1, j2, j3;

//     return J;
// }



// std::tuple<VectorXd, double, int> revise_newton(double L1, double L2, double L3, const VectorXd& q, const VectorXd& r, const VectorXd& xi, int mstep, double tol, const std::string& type) {
//     Matrix4d Td;
//     Td << Quaterniond(q(0), q(1), q(2), q(3)).toRotationMatrix(), r,
//           0, 0, 0, 1;

//     VectorXd omg_e(mstep);
//     VectorXd v_e(mstep);
//     VectorXd e(mstep);
//     int k = 0;
//     VectorXd xi_new = xi;

//     while (k < mstep) {
//         Matrix4d Tt = get_end(L1, L2, L3, xi_new);
//         MatrixXd log_Tt_Td = (Tt.inverse() * Td).topLeftCorner(3, 3);
//         Quaterniond q_log_Tt_Td(log_Tt_Td);
//         VectorXd V(6);
//         V << q_log_Tt_Td.vec(), Tt.inverse() * Td.topRightCorner(3, 1);

//         omg_e(k) = V.segment(0, 3).norm();
//         v_e(k) = V.segment(3, 3).norm();
//         e(k) = V.norm();

//         if (e(k) < tol) {
//             break;
//         } else {
//             // Update
//             MatrixXd J = jacobian3cc(L1, L2, L3, xi_new);
//             xi_new = xi_new + (J.transpose() * J).inverse() * J.transpose() * V;
//             xi_new = arc2xi(L1, L2, L3, xi2arc(L1, L2, L3, xi_new));
//             k++;
//         }
//     }

//     if (k == mstep) {
//         Matrix4d Tt = get_end(L1, L2, L3, xi_new);
//         MatrixXd log_Tt_Td = (Tt.inverse() * Td).topLeftCorner(3, 3);
//         Quaterniond q_log_Tt_Td(log_Tt_Td);
//         VectorXd V(6);
//         V << q_log_Tt_Td.vec(), Tt.inverse() * Td.topRightCorner(3, 1);

//         omg_e(k) = V.segment(0, 3).norm();
//         v_e(k) = V.segment(3, 3).norm();
//         e(k) = V.norm();
//     }

//     return std::make_tuple(xi_new, e(k), k);
// }


int main() {
    double L1 = 1;
    double L2 = 1;
    double L3 = 1;

    // double alpha = 15 * M_PI / 16;
    // Vector3d omega(0.48, sqrt(3) / 10, -0.86);
    // Quaterniond quat(cos(alpha / 2), sin(alpha / 2) * omega(0), sin(alpha / 2) * omega(1), sin(alpha / 2) * omega(2));
    // Vector4d q(quat.w(), quat.x(), quat.y(), quat.z());
    // Vector3d r(-0.4, 1.1, 0.8);

    // VectorXd nex(6);
    // nex << 0.0794, 0.3255, 0.0144, 0.0023, 1.8861, 1.7423;

    // VectorXd xi;
    // double err;
    // int noi;
    // std::tie(xi, err, noi) = revise_newton(L1, L2, L3, q, r, nex, 200, 1e-2, "plot");

    // std::cout << "Final xi:\n" << xi << std::endl;

    return 0;
}

