
#include "pid.h"


PID::PID(const Eigen::VectorXd kp, const Eigen::VectorXd ki, const Eigen::VectorXd kd, const Eigen::VectorXd control_limit) {
    _dim =kp.size();
    _kp = kp.asDiagonal();
    _ki = ki.asDiagonal();
    _kd = kd.asDiagonal();
    _control_limit = control_limit;
    _integral_sum = Eigen::VectorXd::Zero(_dim);
    _last_err = Eigen::VectorXd::Zero(_dim);
}

void PID::loop(const Eigen::VectorXd & err, int64_t time_usec, Eigen::VectorXd & control) {

    if(_last_time_usec == 0) _last_time_usec = time_usec;

    double dt =((time_usec - _last_time_usec)*1.0e-6);
    _integral_sum = (err + _last_err)/2*((time_usec - _last_time_usec)*1e-6);
    control = _kp*err + _ki*_integral_sum + _kd* (err - _last_err)/dt;

    _last_err = err;
    _last_time_usec = time_usec;
}

void PID::reset() {
    _last_time_usec = 0;
    _integral_sum = Eigen::VectorXd::Zero(_dim);
    _last_err = Eigen::VectorXd::Zero(_dim);
}