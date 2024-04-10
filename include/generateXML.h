#ifndef GENERATEXML_H
#define GENERATEXML_H

#include <iostream>
#include <string>
#include <fstream>

std::string generateXML(int length1, int length2, int length3, int config1, int config2, int config3, int config4, int config5, int config6);
void writeXMLToFile(const std::string& filename, const std::string& xmlContent);
Eigen::MatrixXd inverse(const Eigen::VectorXd& kappa, const Eigen::VectorXd& phi, double L1, double L2, double L3, double d);
void generate(double L1, double L2, double L3, Eigen::MatrixXd delta_t);

#endif // GENERATEXML_H
