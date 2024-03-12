#ifndef CSC492_LIEALGEBRA_H
#define CSC492_LIEALGEBRA_H

#include <cmath>
#include <iostream>
#include <Eigen/Dense>

class LieAlgebra {
public:


    /*
    * This method computes the Lie algebra of a vector.
    *  M = UP_HAT(V) is an element of \mathsf{so}_3 or \mathsf{se}_3, where V
    *  is an element of \mathbb{R}^3 or \mathbb{R}^6, respectively.
    *
    * NOTE: This method is equivalent to the MATLAB function MISC.up_hat(v)
    *
    * @param v - the vector to be converted
    */
    static Eigen::MatrixXd up_hat(Eigen::VectorXd V);

    /*
    * This method computes the Lie algebra of a vector.
    * V = UP_VEE(M) is an element of \mathbb{R}^3 or \mathbb{R}^6, where M is
    * an element of \mathsf{so}_3 or \mathsf{se}_3, respectively.
    *
    * NOTE: This method is equivalent to the MATLAB function MISC.up_vee(M)
    *
    * @param M - the matrix to be converted
    */
    static Eigen::VectorXd up_vee(Eigen::MatrixXd M);

    /*
    * This method does VEELOG Composition of the matrix logarithm and the vee map.
    * V = VEELOG(M) is a vector in \mathbb{R}^3 or \mathbb{R}^4 and is
    * computed using Rodrigues' formula. The matrix M is in \mathsf{SO}_3 or
    * \mathsf{SE}_3. The vee map sends an element of \mathsf{so}_3 or
    * \mathsf{se}_3 to a vector.
    *
    * NOTE: This method is equivalent to the MATLAB function MISC.veelog(M)
    *
    * @param M - the matrix to be converted
    */
    static Eigen::VectorXd veelog(Eigen::MatrixXd M);

    /*
    * This method does EXPHAT Composition of the hat map and the matrix exponential.
    * M = EXPHAT(V) is a matrix in \mathsf{SO}_3 or \mathsf{SE}_3 and is
    * computed using Rodrigues' formula. The vector V is in \mathbb{R}^3 or
    * \mathbb{R}^4. The hat map sends V to an element of \mathsf{so}_3 or
    * \mathsf{se}_3. The vector V is in \mathbb{R}^3 or \mathbb{R}^4.
    * The hat map sends V to an element of \mathsf{so}_3 or \mathsf{se}_3.
    *
    * NOTE: This method is equivalent to the MATLAB function MISC.exphat(V)
    *
    * @param V - the vector to be converted
    */
    static Eigen::Matrix4d exphat(Eigen::Matrix<double, 6, 1> V);


    static Eigen::Matrix4d up_plus(Eigen::Quaterniond q);

    static Eigen::Matrix4d up_oplus(Eigen::Quaterniond q);

    static Eigen::Quaterniond up_star(Eigen::Quaterniond q);

    static Eigen::Matrix4d get_end(double L1, double L2, double L3, Eigen::VectorXd xi);

};
#endif //CSC492_LIEALGEBRA_H