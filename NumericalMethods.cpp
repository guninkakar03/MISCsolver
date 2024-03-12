#include "NumericalMethods.h"
#include "LieAlgebra.h"
#include "ConversionHelpers.h"
#include <unsupported/Eigen/MatrixFunctions>


double NumericalMethods::revise_newton(double L1, double L2, double L3, Eigen::Quaterniond q, Eigen::Vector3d r, Eigen::Matrix<double, 6, 1> xi, int msteps, double tol, Eigen::Matrix<double, 6, 1>& xi_star) {

    Eigen::Matrix4d Td;
    Td << ConversionHelper::q2rot(q), r,
          0, 0, 0, 1;

    Eigen::VectorXd omg_e = Eigen::VectorXd::Constant(msteps + 1, std::nan("1"));
    Eigen::VectorXd v_e = Eigen::VectorXd::Constant(msteps + 1, std::nan("1"));
    Eigen::VectorXd e = Eigen::VectorXd::Constant(msteps + 1, std::nan("1"));

    int k = 0;

    while(k < msteps){
        Eigen::Matrix4d Tt = LieAlgebra::get_end(L1, L2, L3, xi);
        Eigen::VectorXd V  = LieAlgebra::up_vee((Tt.inverse() * Td).log()); 
        omg_e(k) = V.head(3).norm();
        v_e(k) = V.tail(3).norm();
        e(k) = V.norm();
        if (e(k) < tol){
            break;
        } else {
            Eigen::MatrixXd J(6,6);
            J = NumericalMethods::jacobian3cc(L1, L2, L3, xi);
            xi = xi + ((J.transpose() * J).inverse() * J.transpose()) * V;
            xi = ConversionHelper::arc2xi(L1, L2, L3, ConversionHelper::xi2arc(L1, L2, L3, xi));
            k += 1;
        }
    }

    if(k == msteps){
        Eigen::Matrix4d Tt;
        Tt = LieAlgebra::get_end(L1, L2, L3, xi);
        Eigen::VectorXd V = LieAlgebra::up_vee((Tt.inverse() * Td).log());
        omg_e(k) = V.head(3).norm();
        v_e(k) = V.tail(3).norm();
        e(k) = V.norm();
    }
    xi_star = xi;
    std::cout << "k: " << k << std::endl;
    return e(k);
}

double NumericalMethods::revise_dls(double L1, double L2, double L3, Eigen::Quaterniond q, Eigen::Vector3d r, Eigen::Matrix<double, 6, 1> xi, int msteps, double tol, Eigen::Matrix<double, 6, 1>& xi_star) {
        Eigen::Matrix4d Td;
    Td << ConversionHelper::q2rot(q), r,
          0, 0, 0, 1;

    Eigen::VectorXd omg_e = Eigen::VectorXd::Constant(msteps + 1, std::nan("1"));
    Eigen::VectorXd v_e = Eigen::VectorXd::Constant(msteps + 1, std::nan("1"));
    Eigen::VectorXd e = Eigen::VectorXd::Constant(msteps + 1, std::nan("1"));

    int k = 0;

    while(k < msteps){
        Eigen::Matrix4d Tt = LieAlgebra::get_end(L1, L2, L3, xi);
        Eigen::VectorXd V  = LieAlgebra::up_vee((Tt.inverse() * Td).log()); 
        omg_e(k) = V.head(3).norm();
        v_e(k) = V.tail(3).norm();
        e(k) = V.norm();
        if (e(k) < tol){
            break;
        } else {
            Eigen::MatrixXd J(6,6);
            J = NumericalMethods::jacobian3cc(L1, L2, L3, xi);
            Eigen::MatrixXd M = J.transpose() * J;
            double diagonal_max = M.diagonal().maxCoeff();
            Eigen::MatrixXd eye = Eigen::MatrixXd::Identity(6, 6);
            xi = xi + (M + diagonal_max * eye).inverse() * J.transpose() * V;
            xi = ConversionHelper::arc2xi(L1, L2, L3, ConversionHelper::xi2arc(L1, L2, L3, xi));
            k += 1;
        }
    }

    if(k == msteps){
        Eigen::Matrix4d Tt;
        Tt = LieAlgebra::get_end(L1, L2, L3, xi);
        Eigen::VectorXd V = LieAlgebra::up_vee((Tt.inverse() * Td).log());
        omg_e(k) = V.head(3).norm();
        v_e(k) = V.tail(3).norm();
        e(k) = V.norm();
    }
    xi_star = xi;
    std::cout << "k: " << k << std::endl;
    return e(k);

}


Eigen::MatrixXd NumericalMethods::jacobian3cc(double L1, double L2, double L3, Eigen::Matrix<double, 6, 1> xi){
    
    Eigen::MatrixXd j3 = NumericalMethods::jaco_c12(xi(4, 0), xi(5, 0), L3);
    Eigen::VectorXd temp(6);
    temp << xi(4, 0), xi(5, 0), 0, 0, 0, L3;
    Eigen::Matrix4d T3 = LieAlgebra::up_hat(temp).exp();
    Eigen::Matrix4d invT3;
    invT3 << T3.block(0, 0, 3, 3).transpose(), -(T3.block(0, 0, 3, 3)).transpose() * T3.block(0, 3, 3, 1),
        0, 0, 0, 1;
    Eigen::MatrixXd j2 = NumericalMethods::jaco_c12(xi(2, 0), xi(3, 0), L2);
    Eigen::VectorXd j2_c1 = LieAlgebra::up_vee(invT3 * LieAlgebra::up_hat(j2.col(0)) * T3);
    Eigen::VectorXd j2_c2 = LieAlgebra::up_vee(invT3 * LieAlgebra::up_hat(j2.col(1)) * T3);
    
    j2 << j2_c1, j2_c2;

    Eigen::VectorXd temp2(6);
    temp2 << xi(2), xi(3), 0, 0, 0, L2;
    Eigen::Matrix4d T2 = LieAlgebra::up_hat(temp2).exp();
    Eigen::Matrix4d invT2;
    invT2 << T2.block(0, 0, 3, 3).transpose(), -T2.block(0, 0, 3, 3).transpose() * T2.block(0, 3, 3, 1),
        0, 0, 0, 1;
    Eigen::MatrixXd j1 = NumericalMethods::jaco_c12(xi(0), xi(1), L1);
    Eigen::VectorXd j1_c1 = LieAlgebra::up_vee(invT3 * invT2 * LieAlgebra::up_hat(j1.col(0)) * T2 * T3);
    Eigen::VectorXd j1_c2 = LieAlgebra::up_vee(invT3 * invT2 * LieAlgebra::up_hat(j1.col(1)) * T2 * T3);
    j1 << j1_c1, j1_c2;

    Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Zero();
    J << j1, j2, j3;
    return J;
}



Eigen::MatrixXd NumericalMethods::jaco_c12(double w1, double w2, double L){
    Eigen::Vector3d w;
    w << w1, w2, 0;

    Eigen::Matrix3d eye = Eigen::Matrix3d::Identity();
    double n = w.norm();


    Eigen::Matrix<double, 6, 3> Jc; 

    if (n==0) {
        double ML = L/2;
        
        Jc << eye,
        0,ML,0,
        -ML, 0, 0,
        0, 0, 0;

    }
     else {
        double M = (1 - cos(n)) / (n*n);
        double N = (n - sin(n)) / (n*n*n);
        double p1M = w1/(n*n) - w1*N - 2*w1*M/(n*n);
        double p2M = w2/(n*n) - w2*N - 2*w2*M/(n*n);
        double p1N = w1*M/(n*n) - 3*w1*N/(n*n);
        double p2N = w2*M/(n*n) - 3*w2*N/(n*n);
        Eigen::Matrix3d pwJleftwv;
        pwJleftwv << p1M*w2, p2M*w2 + M, 0,
                    -p1M*w1 - M, -p2M*w1, 0,
                    -p1N*n*n - 2*N*w1,-p2N*n*n - 2*N*w2,0;

        Eigen::Matrix3d w_hat = LieAlgebra::up_hat(w);
        Eigen::Matrix3d exp_w_hat_transpose = w_hat.exp().transpose();
        Jc << eye - M * LieAlgebra::up_hat(w) + N * (LieAlgebra::up_hat(w) * LieAlgebra::up_hat(w)),
            exp_w_hat_transpose * pwJleftwv;
    }

    return Jc.block(0, 0, Jc.rows(), 2);

}
