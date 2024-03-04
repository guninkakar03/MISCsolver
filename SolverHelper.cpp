// set up the class 
//

#include <Eigen/Dense>


class SolverHelper {

    SolverHelper() {
    }

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
    double rho(double a, double L) {
        if (a==1) return L;
        double a_square = a*a;
        double numerator = L * sqrt((1 - a_square));
        double denominator = acos(a);
        return numerator / denominator;
    }

   
    Eigen::Matrix<double, 6, 1> soln2xi (int L1, int L2, int L3, Eigen::Matrix<double, 9, 1>& soln) {
       
    }

  


};