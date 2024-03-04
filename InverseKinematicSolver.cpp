#include "InverseKinematicSolver.h"
#include <Eigen/Dense>


/*
 * This class is responsible for solving the inverse kinematics
 * for a 3-Section continuum robot.
 */
class InverseKinematicSolver {
public:
    // The length of each section of the robot
    int l1;
    int l2;
    int l3;

    // A qauternion representing the orientation of the end effector
    Eigen::Quaterniond orientation;

    // A 3x1 vector representing the position of the end effector
    Eigen::Vector3d position;

    /*
    *  This is the constructor for the InverseKinematicSolver class.
    *  It takes in the length of each section of the robot, the orientation
    *  of the end effector, and the position of the end effector.
    */
    InverseKinematicSolver(int l1, int l2, int l3, Eigen::Quaterniond orientation, Eigen::Vector3d position) {
        this->l1 = l1;
        this->l2 = l2;
        this->l3 = l3;
        this->orientation = orientation;
        this->position = position;
    }


    /*
    * This method acts as a helper function to the inverse kinematics solver.
    * It takes care of the Canditate Selection step of the inverse kinematics
    * algorithm.
    */
    Eigen::VectorXd candidateSelection(double partition) {
        double a = orientation.w();
        double b = orientation.x();
        double c = orientation.y();
        double d = orientation.z();
        
        Eigen::Matrix3d B;
        B << d, a, b, 
            -a, d, c,
            -b, -c, d;
        
    
        Eigen::Vector3d n0 = B.transpose() * position;

        
        Eigen::Vector3d n = n0 / n0.norm();

       
        Eigen::Vector3d u(n(1), -n(0), 0);
        u = u / u.norm();
        Eigen::Vector3d v = n.cross(u);
        Eigen::Matrix3d P;
        P << u, v, n;

        
        Eigen::VectorXd zeta = partitionInterval(0, 1, partition); 


    }


    /*
    * This method helps to partition a given interval into some 
    * number of subintervals.
    */
    Eigen::VectorXd partitionInterval(double start, double end, double partition) {
        Eigen::VectorXd partitionInterval;
        int j = 0;
        for (int i = 0; i < end; i+= partition) { // The candidate selection is done for t in [0,1)
            j++; // todo
        }
        return partitionInterval;
    }
};
