#pragma once

#include "../trj_planer.h"

class CubicSpline: public TrjPlaner {
private:
    std::vector<Eigen::VectorXd> _poly;
    Eigen::MatrixXd _vronsk_matrix;
    Eigen::VectorXd _control_points;
public:
    CubicSpline(double time_delay, double max_accel, double max_velocity, double default_speed, int dim);
    bool generate_trj(Path const &path, Path const &speeds) override;
    Point point(double t);
};
