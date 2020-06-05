
#include "pid.h"

PID::PID():_params(1){
    _dim =1;
    _integral_sum = Eigen::VectorXd::Zero(_dim);
    _last_err = Eigen::VectorXd::Zero(_dim);
    _last_time_usec =0;
}

PID::PID(PIDParams & params):_params(params) {
    _dim = params.dim();
    _integral_sum = Eigen::VectorXd::Zero(_dim);
    _last_err = Eigen::VectorXd::Zero(_dim);
    _last_time_usec =0;
}

void PID::set_params(PIDParams &params) {
    _params = params;
    _dim = params.dim();
}

void PID::loop(const Eigen::VectorXd & err, int64_t time_usec, Eigen::VectorXd & control) {

    if(_last_time_usec == 0) _last_time_usec = time_usec;
    else {
        double dt = (((time_usec - _last_time_usec) + ((time_usec - _last_time_usec)==0)*10) * 1.0e-6);
        _integral_sum = (err + _last_err) / 2 * ((time_usec - _last_time_usec) * 1e-6);
    control = _params.kp.asDiagonal()*err + _params.ki.asDiagonal()*_integral_sum + _params.kd.asDiagonal()* (err - _last_err)/dt;

//        control = _params.kp.asDiagonal() * err + _params.ki.asDiagonal() * _integral_sum;
        std::cout << control << std::endl;
        _last_err = err;
        _last_time_usec = time_usec;
    }
}

void PID::reset() {
    _last_time_usec = 0;
    _integral_sum = Eigen::VectorXd::Zero(_dim);
    _last_err = Eigen::VectorXd::Zero(_dim);
}