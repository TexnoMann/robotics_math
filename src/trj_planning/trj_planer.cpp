#include "trj_planer.h"

TrjPlaner::~TrjPlaner(){}

TrjPlaner::TrjPlaner(double time_delay,double *max_accel, double *default_speed, int dim){
    _dim = dim;
    _time_delay = time_delay;
    _max_accel = max_accel;
    _default_speed = default_speed;
}

Point TrjPlaner::point(double t){
    return Point(t, Eigen::VectorXd::Zero(_dim));
}