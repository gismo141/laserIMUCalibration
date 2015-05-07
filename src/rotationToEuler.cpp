/**
 * Copyright 2015 <michael.riedel@dlr.de>
 */

#include <Eigen/Geometry>
#include <iostream>

int main(int argc, char const* argv[]) {
    Eigen::Matrix3f m;
    m = Eigen::AngleAxisf(0.25 * M_PI, Eigen::Vector3f::UnitX())
        * Eigen::AngleAxisf(0.5 * M_PI, Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(0.33 * M_PI, Eigen::Vector3f::UnitZ());

    std::cout << "Original rotation:" << std::endl;
    std::cout << m << std::endl;

    Eigen::Vector3f ea = m.eulerAngles(0, 1, 2);

    std::cout << "to Euler:" << std::endl;
    std::cout << ea << std::endl;

    Eigen::Matrix3f n;
    n = Eigen::AngleAxisf(ea[0] * M_PI, Eigen::Vector3f::UnitX())
        * Eigen::AngleAxisf(ea[1] * M_PI, Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(ea[2] * M_PI, Eigen::Vector3f::UnitZ());

    std::cout << "Recalculated rotation:" << std::endl;
    std::cout << n << std::endl;

    return 0;
}
