#include <cmath>
#include <iostream>
#include <Eigen/Dense>


class LieAlgebra {
public:
    LieAlgebra() {}
    
    /*
    * This method computes the Lie algebra of a vector.
    *  M = UP_HAT(V) is an element of \mathsf{so}_3 or \mathsf{se}_3, where V
    *  is an element of \mathbb{R}^3 or \mathbb{R}^6, respectively.
    *
    * NOTE: This method is equivalent to the MATLAB function MISC.up_hat(v)
    *
    * @param v - the vector to be converted
    */
    Eigen::MatrixXd up_hat(Eigen::VectorXd V) {
        if (V.size() == 3) {
            Eigen::Matrix3d M;
            M <<    0, -V(2),  V(1),
                V(2),     0, -V(0),
                -V(1),  V(0),     0;
            return M;
        } else if (V.size() == 6) {
            Eigen::Matrix4d M;
            M <<    0, -V(2),  V(1), V(3),
                V(2),     0, -V(0), V(4),
                -V(1),  V(0),     0, V(5),
                    0,     0,     0,    0;
            return M;
        } 
    }

    /*
    * This method computes the Lie algebra of a vector.
    * V = UP_VEE(M) is an element of \mathbb{R}^3 or \mathbb{R}^6, where M is
    * an element of \mathsf{so}_3 or \mathsf{se}_3, respectively.
    *
    * NOTE: This method is equivalent to the MATLAB function MISC.up_vee(M)
    *
    * @param M - the matrix to be converted
    */
    Eigen::VectorXd up_vee(Eigen::MatrixXd M) {
        if (M.rows() == 3) {
            Eigen::Vector3d V;
            V << -M(1, 2), M(0, 2), -M(0, 1);
            return V;
        } else if (M.rows() == 4) {
            Eigen::VectorXd V(6);
            V << -M(1, 2), M(0, 2), -M(0, 1), M(0, 3), M(1, 3), M(2, 3);
            return V;
        }
    }

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
    Eigen::VectorXd veelog(Eigen::MatrixXd M) {
    }

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
    Eigen::MatrixXd exphat(const Eigen::VectorXd& V) {
        // To be implemented;   
        return Eigen::MatrixXd::Zero(3, 3);
    }


};
