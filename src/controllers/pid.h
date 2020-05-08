#pragma once

#include "math.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>

class PID {
public:
    PID(const Eigen::VectorXd kp, const Eigen::VectorXd ki, const Eigen::VectorXd kd, const Eigen::VectorXd control_limit);
    void loop(const Eigen::VectorXd & err, const int64_t time_usec, Eigen::VectorXd & control);
    void reset();
private:
    int _dim;
    Eigen::MatrixXd _kp;
    Eigen::MatrixXd _ki;
    Eigen::MatrixXd _kd;
    Eigen::VectorXd _control_limit;
    Eigen::VectorXd _integral_sum;
    Eigen::VectorXd _last_err;
    int64_t _last_time_usec;
};

