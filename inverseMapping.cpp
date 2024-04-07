#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
//  g++ -std=c++11 -I/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3 inverseMapping.cpp -o inverseMapping
// Parameters for each segment
struct SegmentParams {
    double length;
    double pitch_radius;
};


bool inverseMapping(const std::vector<SegmentParams>& segments, const std::vector<double>& kappa, const std::vector<double>& phi, Eigen::MatrixXd& q_tendons,Eigen::VectorXd& q_lengths) {

    q_tendons.resize(segments.size() * 3, 1);
    q_lengths.resize(segments.size(), 1);

    double beta = 2 * M_PI / 3;

    for(size_t i = 0; i < segments.size(); ++i) {
        // the angle between tangent to the curve and x axis
        double theta_l = M_PI / 2 - kappa[i] * segments[i].length;
        double dt1, dt2, dt3;

        if(phi[i] == 0) { // this implies that is bending anlge of a section is zero then only 1 tendon need to be actuated
            dt1 = segments[i].pitch_radius * std::cos(phi[i]) * (M_PI / 2 - theta_l);
            dt2 = dt3 = -dt1 / 2;
        } else {// the segment is bending, so the displacements of all three tendons need to be calculated to achieve the desired curvature and bending angle.
            dt1 = segments[i].pitch_radius * std::cos(phi[i]) * (M_PI / 2 - theta_l);
            dt2 = segments[i].pitch_radius * std::cos(phi[i] - beta) * (M_PI / 2 - theta_l);
            dt3 = -dt1 - dt2;
        }

        q_tendons.block(i * 3, 0, 3, 1) = Eigen::Vector3d(dt1, dt2, dt3);
        q_lengths(i, 0) = 0;
    }

    return true;
}

int main() {
    std::vector<SegmentParams> segments = {
        {0.1, 0.01}, 
        {0.1, 0.01},
        {0.1, 0.01}  
    };

    std::vector<double> kappa = {24.8174 ,  23.2046, 20.9087};
    std::vector<double> phi = { 2.5952 ,  1.0686, 0.2511};

    Eigen::MatrixXd q_tendons;
    Eigen::VectorXd q_lengths;

    inverseMapping(segments, kappa, phi, q_tendons, q_lengths) 
    std::cout << "Tendon displacements:\n" << q_tendons << std::endl;


    return 0;
}
