/**
 * @file SolverHelper.cpp
 * 
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


#include "../include/SolverHelper.h"
#include "../include/LieAlgebra.h"

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
double SolverHelper::rho(double a, double L) {
    double d;
    if (a == 1) {
        d = L;
    } else {
        d = L * sqrt(1 - pow(a, 2)) / acos(a);
    }
    return d;
    
}

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
Eigen::Matrix<double, 6, 1> SolverHelper::soln2xi (int L1, int L2, int L3, Eigen::Matrix<double, 9, 1> soln) {
    // double k1 = 2.0 / L1 * acos(soln(2));
    // double k2 = 2.0 / L2 * acos(soln(5));
    // double k3 = 2.0 / L3 * acos(soln(8));
    // double p1 = atan2(soln(1), soln(0));
    // double p2 = atan2(soln(4), soln(3));
    // double p3 = atan2(soln(7), soln(6));
    // Eigen::Matrix<double, 6, 1> xi;
    // xi << -L1*k1*sin(p1), L1*k1*cos(p1), -L2*k2*sin(p2), L2*k2*cos(p2), -L3*k3*sin(p3), L3*k3*cos(p3);
    // return xi;

    double k1 = 2.0 / L1 * acos(soln(2));  
    double p1 = atan2(soln(1), soln(0));   
    double k2 = 2.0 / L2 * acos(soln(5));  
    double p2 = atan2(soln(4), soln(3));   
    double k3 = 2.0 / L3 * acos(soln(8));  
    double p3 = atan2(soln(7), soln(6));   
    Eigen::Matrix<double, 6, 1> xi;
    xi << -L1 * k1 * sin(p1),
           L1 * k1 * cos(p1),
          -L2 * k2 * sin(p2),
           L2 * k2 * cos(p2),
          -L3 * k3 * sin(p3),
           L3 * k3 * cos(p3);
    return xi;

}



/*
* GET_ERR Computes the error between desired and current end pose.
*
* NOTE: This method is equivalent to the MATLAB function MISC.get_err(r1, r2, r3, L1, L2, L3, q, r)
*
*/
double SolverHelper::get_err(Eigen::Vector3d r1, Eigen::Vector3d r2, Eigen::Vector3d r3, double L1, double L2, double L3, Eigen::Quaterniond q, Eigen::Vector3d r) {
    
    Eigen::Matrix4d Td;
    Td << q.toRotationMatrix(), r,
        0, 0, 0, 1;
    
    Eigen::Matrix4d T1;
    Eigen::Quaterniond q1_temp(r1(2), -r1(1), r1(0), 0); 
    T1 << q1_temp.toRotationMatrix(), SolverHelper::rho(r1(2),L1)*r1,
        0, 0, 0, 1;

    Eigen::Matrix4d T2;
    Eigen::Quaterniond q2_temp(r2(2), -r2(1), r2(0), 0);
    T2 << q2_temp.toRotationMatrix(), SolverHelper::rho(r2(2), L2)*r2,
        0, 0, 0, 1;
    
    Eigen::Matrix4d T3;
    Eigen::Quaterniond q3_temp(r3(2), -r3(1), r3(0), 0);
    T3 << q3_temp.toRotationMatrix(), SolverHelper::rho(r3(2), L3)*r3,
        0, 0, 0, 1;
    
    Eigen::Matrix4d Tt = T1 * T2 * T3;
    // Eigen::Matrix4d vec = Tt.lu().inverse() * Td;
    Eigen::Matrix4d vec = (Tt.inverse() * Td);
    std::cout << vec << std::endl;
    Eigen::VectorXd V = LieAlgebra::veelog(vec);
    // std::cout << V << std::endl;
    return V.norm();

    // Eigen::Matrix4d Td;
    // Td << q.toRotationMatrix(), r,
    //       0, 0, 0, 1;

    // Eigen::Matrix4d T1;
    // Eigen::Quaterniond q1_temp(0, r1(2), -r1(1), r1(0)); 
    // T1 << q1_temp.toRotationMatrix(), SolverHelper::rho(r1(2), L1) * r1,
    //       0, 0, 0, 1;

    // Eigen::Matrix4d T2;
    // Eigen::Quaterniond q2_temp(0, r2(2), -r2(1), r2(0));
    // T2 << q2_temp.toRotationMatrix(), SolverHelper::rho(r2(2), L2) * r2,
    //       0, 0, 0, 1;

    // Eigen::Matrix4d T3;
    // Eigen::Quaterniond q3_temp(0, r3(2), -r3(1), r3(0)); 
    // T3 << q3_temp.toRotationMatrix(), SolverHelper::rho(r3(2), L3) * r3,
    //       0, 0, 0, 1;

    // Eigen::Matrix4d Tt = T1 * T2 * T3;
    // Eigen::VectorXd V = LieAlgebra::veelog(Tt.inverse() * Td);
    // return V.norm();
}

/*
* SOLVE_R1 Computes the model parameter of the 1st section.
*
* NOTE: This method is equivalent to the MATLAB function MISC.solve_r1(L1, L2, L3, q, r, par, noc)
*
*/
// Eigen::VectorXd SolverHelper::solve_r1(double L1, Eigen::Quaterniond q, Eigen::Vector3d r, Eigen::Vector3d r3, double noc) {
//     double a = q.w();
//     double b = q.x();
//     double c = q.y();
//     double d = q.z();

//     Eigen::Matrix3d B;
//     B << d, a, b,
//     -a, d, c,
//     -b, -c, d;

//     Eigen::Vector3d n0 = B.transpose() * r;
//     Eigen::Vector3d r0 = (L1 * d) / (n0.norm() * n0.norm()) * n0;
//     Eigen::Vector3d ne = B * r3;

//     Eigen::Vector3d r1 = SolverHelper::spp(n0, (0.5 + 1/M_PI) * L1 * d, ne, r3);


//     if(d != 0){
//         for(int cor_idx = 1; cor_idx <= noc; cor_idx++){
//             // Eigen::Matrix<double, 3, 2> r0_ne;
//             // r0_ne << r0, ne;
//             // Eigen::MatrixXd A(2, 2);
//             // A << 0, 0, L1 * d * (1 / acos(r1(2)) - 1 / sqrt(1 - r1(2) * r1(2))),
//             //     ne.transpose();
//             // A = (A * r0_ne).inverse();
//             // Eigen::Vector2d B;
//             // B << n0.transpose() * r1 - SolverHelper::rho(r1(2), L1) * d,
//             //     ne.transpose() * r1;
//             // Eigen::Vector3d tmp = r1 - r0_ne * (A * B);
//             // r1 = tmp/tmp.norm();

//         }
//     }
//     return r1;
// }

Eigen::VectorXd SolverHelper::solve_r1(double L1, Eigen::Quaterniond q, Eigen::Vector3d r, Eigen::Vector3d r3, double noc) {
    double a = q.w();
    double b = q.x();
    double c = q.y();
    double d = q.z();

    Eigen::Matrix3d B;
    B << d, a, b,
         -a, d, c,
         -b, -c, d;

    Eigen::Vector3d n0 = B.transpose() * r;
    Eigen::Vector3d r0 = (L1 * d) / (n0.norm() * n0.norm()) * n0;
    Eigen::Vector3d ne = B * r3;
    Eigen::Vector3d r1 = SolverHelper::spp(n0, (0.5 + 1/M_PI) * L1 * d, ne, r3);
    Eigen::Vector3d tmp;
    if (d != 0) {
        for (int cor_idx = 1; cor_idx <= noc; cor_idx++) { 
            Eigen::MatrixXd r0_ne(3, 2);
            r0_ne << r0, ne;
            Eigen::MatrixXd s1(2, 3);
            Eigen::Vector3d new_gunin = n0 + Eigen::Vector3d(0, 0, L1 * d * (1 / acos(r1(2)) - 1 / sqrt(1 - r1(2) * r1(2))));
            s1 << new_gunin.transpose(), ne.transpose();
            Eigen::Matrix2d s2 = s1 * r0_ne;
            Eigen::Vector2d s3;
            s3 << n0.transpose() * r1 - rho(r1(2), L1) * d,
            ne.transpose() * r1;
            Eigen::Vector2d s4;
            s4 = s2.inverse() * s3;
            tmp = r1 - r0_ne * s4;
            r1 = tmp / tmp.norm();

        }
    }
    return r1;
}



Eigen::Vector3d SolverHelper::spp(Eigen::Vector3d n1, double d, Eigen::Vector3d n2, Eigen::Vector3d rn){
    double n11 = n1(0);
    double n12 = n1(1);
    double n13 = n1(2);

    double n21 = n2(0);
    double n22 = n2(1);
    double n23 = n2(2);

    double det0 = n11*n22 - n12*n21;
    double det1 = n12*n23 - n13*n22;
    double det2 = n11*n23 - n13*n21;

    double a = det0*det0 + det1*det1 + det2*det2;

    Eigen::Vector3d soln;
    if (a < std::numeric_limits<double>::epsilon()){
        soln << -rn(0)*rn(2), -rn(1)*rn(2), rn(0)*rn(0) + rn(1)*rn(1);
        soln = soln / (sqrt(rn(0)*rn(0) + rn(1)*rn(1)));

    } else {
        double b = 2 * d * (n22 * det1 + n21 * det2);
        double c = d*d * (n22*n22 + n21*n21) - det0*det0;
        double delta = b*b - 4 * a * c;
        
        if(delta < 0) {
            double r3 = -b / (2 * a);
            double r1 = (det1 * r3 + n22 * d) / det0;
            double r2 = -(det2 * r3 + n21 * d) / det0;
            soln << r1, r2, r3;
            soln = soln / soln.norm();
        } else {
            double r3 = (-b + sqrt(delta)) / (2 * a);
            double r1 = (det1 * r3 + n22 * d) / det0;
            double r2 = -(det2 * r3 + n21 * d) / det0;
            soln << r1, r2, r3;
        }

    }
    return soln;
}

/*
* SOLVE_R2 Computes the model parameter of the 2nd section using rotational and translational constraints.
*
* NOTE: This method is equivalent to the MATLAB function MISC.solve_r2(L1, L3, q, r, r3, r1)
*/
std::array<Eigen::Vector3d, 2> SolverHelper::solve_r2(double L1, double L3, Eigen::Quaterniond q, Eigen::Vector3d r, Eigen::Vector3d r3, Eigen::Vector3d r1) {
    double a = q.w();
    double b = q.x();
    double c = q.y();
    double d = q.z();

    Eigen::Matrix3d B;
    B << d, a, b,
    -a, d, c,
    -b, -c, d;

    Eigen::Vector3d  m;
    m << c, -b, a;

    // qe = [m.' * r3; B * r3];
    Eigen::Vector4d qe_vec;
    qe_vec << m.transpose() * r3, B * r3;
    Eigen::Quaterniond qe(qe_vec(0), qe_vec(1), qe_vec(2), qe_vec(3));
    // re = [0; r] - rho(r3(3), L3) .* up_plus(qe) * up_oplus(up_star(qe)) * [0; r3]; re = re(2:4);
    Eigen::Vector4d r_0;
    r_0 << 0, r;

    Eigen::Vector4d temp;
    temp << 0, r3;

    Eigen::Vector4d re_vec4 = r_0 - SolverHelper::rho(r3(2), L3) * LieAlgebra::up_plus(qe) * LieAlgebra::up_oplus(LieAlgebra::up_star(qe)) * temp;
    // std::cout << "re_vec4:" <<   SolverHelper::rho(r3(2), L3) << std::endl;
    Eigen::Vector3d re = re_vec4.tail(3);
    // std::cout << "re:" << re << std::endl;

    double ae = qe.w();
    double be = qe.x();
    double ce = qe.y();
    double de = qe.z();

    Eigen::Matrix3d Ae;
    Ae << -ae, -de, ce, 
        de, -ae, -be,
        ce, -be, ae;

    Eigen::Vector3d r2r = Ae * r1;

    Eigen::Vector3d rv = re - SolverHelper::rho(r1(2), L1) * r1;
    // std::cout << "rv:" << rv << std::endl;
    Eigen::Matrix3d eye = Eigen::Matrix3d::Identity();
    Eigen::Vector3d negtive_w2 = (2 * (r1 * r1.transpose()) - eye) * rv / (rv.norm());
    // std::cout << "negtive_w2:" << negtive_w2 << std::endl;
    Eigen::Vector3d r2t;
    r2t << -negtive_w2(0), -negtive_w2(1), negtive_w2(2);
    // todo 
    return {r2r, r2t};
}





