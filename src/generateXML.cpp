#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sstream>
#include "../include/ConversionHelpers.h"

std::string generateXML(double L1, double L2, double L3, double config1, double config2, double config3, double config4, double config5, double config6) {
    std::stringstream xmlStream;
    xmlStream << "<entity entityType=\"TDCR\" name=\"TDCR1\">\n";
    xmlStream << "    <base roll=\"0\" yaw=\"0\" z=\"0\" x=\"0\" y=\"0\" pitch=\"0\"/>\n";
    xmlStream << "    <parameters numTendon=\"3\" modelType=\"Kinematic\" extensible=\"false\" tubular=\"false\"/>\n";
    xmlStream << "    <segment length=\"" << L1 << "\" backboneInnerDiameter=\"0\" poissonRatio=\"0.3\" numDisk=\"10\" youngsModulus=\"5.4e+10\" backboneOuterDiameter=\"0.001\" weightBackbone=\"0\" pitchRadius=\"0.01\" weightDisk=\"0\" diskRadius=\"0.0106\" diskHeight=\"0.002\">\n";
    xmlStream << "        <color>[1 1 1 1 1]</color>\n";
    xmlStream << "    </segment>\n";
    xmlStream << "    <segment length=\"" << L2 << "\" backboneInnerDiameter=\"0\" poissonRatio=\"0.3\" numDisk=\"10\" youngsModulus=\"5.4e+10\" backboneOuterDiameter=\"0.001\" weightBackbone=\"0\" pitchRadius=\"0.01\" weightDisk=\"0\" diskRadius=\"0.0106\" diskHeight=\"0.002\">\n";
    xmlStream << "        <color>[1 1 1 1 1]</color>\n";
    xmlStream << "    </segment>\n";
    xmlStream << "    <segment length=\"" << L3 << "\" backboneInnerDiameter=\"0\" poissonRatio=\"0.3\" numDisk=\"10\" youngsModulus=\"5.4e+10\" backboneOuterDiameter=\"0.001\" weightBackbone=\"0\" pitchRadius=\"0.01\" weightDisk=\"0\" diskRadius=\"0.0106\" diskHeight=\"0.002\">\n";
    xmlStream << "        <color>[1 1 1 1 1]</color>\n";
    xmlStream << "    </segment>\n";
    xmlStream << "    <configuration>[" << config1 << " " << config2 << "][" << config3 << " " << config4 << "][" << config5 << " " << config6 << "]</configuration>\n";
    xmlStream << "</entity>";

    return xmlStream.str();
}

void writeXMLToFile(std::string filename, std::string xmlContent) {
    std::ofstream outputFile(filename);
    if (outputFile.is_open()) {
        outputFile << "<ENTITIES>\n";
        outputFile << xmlContent;
        outputFile << "\n</ENTITIES>";
        outputFile.close();
    } else {
        std::cerr << "Failed to open file for writing\n";
    }
}

Eigen::MatrixXd inverse(Eigen::VectorXd kappa, Eigen::VectorXd phi, double L1, double L2, double L3, double d) {
    double beta = 2 * M_PI / 3;
    double t11 = L1 * d * kappa(0) * std::cos(phi(0));
    double t12 = L1 * d * kappa(0) * std::cos(beta - phi(0));
    double t21 = L2 * d * kappa(1) * std::cos(phi(1));
    double t22 = L2 * d * kappa(1) * std::cos(beta - phi(1));
    double t31 = L3 * d * kappa(2) * std::cos(phi(2));
    double t32 = L3 * d * kappa(2) * std::cos(beta - phi(2));
    Eigen::MatrixXd tendons(3,2);
    tendons << t11, t12,
               t21, t22,
               t31, t32;
    return tendons;
}

void generate(double L1, double L2, double L3, Eigen::MatrixXd solution){

    int num_of_sol = solution.cols();
    std::string dataXML = "";
    for (int i = 0; i < num_of_sol; i++){
        Eigen::VectorXd arc_parameters = ConversionHelper::xi2arc(L1, L2, L3, solution.col(i));
        Eigen::Vector3d kappa;
        kappa << arc_parameters(0), arc_parameters(2), arc_parameters(4);
        Eigen::Vector3d phi;
        phi << arc_parameters(1), arc_parameters(3), arc_parameters(5);
        Eigen::MatrixXd delta_t = inverse(kappa, phi, L1, L2, L3, 0.01);
        std::cout << delta_t << std::endl;
        dataXML += generateXML(L1/5, L2/5, L3/5, delta_t(0,0), delta_t(0,1), delta_t(1,0), delta_t(1,1), delta_t(2,0), delta_t(2,1)); // scaling the length for visualization
    }
    std::cout << dataXML << std::endl;
    writeXMLToFile("output.xml", dataXML);
}