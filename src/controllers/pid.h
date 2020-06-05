#pragma once

#include "math.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>

struct PIDParams{
public:
    PIDParams(int dim){
        _dim = dim;
        kp = Eigen::VectorXd::Zero(_dim);
        ki = Eigen::VectorXd::Zero(_dim);
        kd = Eigen::VectorXd::Zero(_dim);
        control_limit = Eigen::VectorXd::Zero(_dim);

    }
    Eigen::VectorXd kp;
    Eigen::VectorXd ki;
    Eigen::VectorXd kd;
    Eigen::VectorXd control_limit;
    int dim(){return _dim;}
private:
    int _dim;
};

class PID {
public:
    PID();
    PID(PIDParams & params);
    void set_params(PIDParams & params);
    void loop(const Eigen::VectorXd & err, const int64_t time_usec, Eigen::VectorXd & control);
    void reset();
private:
    int _dim;
    PIDParams _params;
    Eigen::VectorXd _integral_sum;
    Eigen::VectorXd _last_err;
    int64_t _last_time_usec;
};

