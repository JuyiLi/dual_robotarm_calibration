#ifndef DUAL_ROBOTARM_CALIBRATION_H
#define DUAL_ROBOTARM_CALIBRATION_H
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class dual_robotarm_calibration {

public:

    dual_robotarm_calibration() = default;
    ~dual_robotarm_calibration() = default;

    bool Three_points_calibration(const std::vector<Eigen::Vector3d>& master_point, const std::vector<Eigen::Vector3d>& slave_point, Eigen::Isometry3d &master2slave);

private:

    double eps = 1e-6;
};

#endif
