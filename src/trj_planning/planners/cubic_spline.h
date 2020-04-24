#pragma once

#include "../trj_planer.h"

class CubicSpline: public TrjPlaner {
private:
    std::vector<Eigen::VectorXd> _poly;
    Eigen::MatrixXd _vronsk_matrix;
    Eigen::VectorXd _control_points;
public:
    CubicSpline(double time_delay,double *max_accel, double *default_speed, int dim);
    bool generate_trj(const Eigen::VectorXd & start_pose, const Eigen::VectorXd finish_pose, const Eigen::VectorXd start_speed, const Eigen::VectorXd finish_speed) override;
    Point point(double t) override ;
    double end_t() override;
};
