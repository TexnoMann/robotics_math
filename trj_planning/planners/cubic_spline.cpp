

#include "cubic_spline.h"


CubicSpline::CubicSpline(double time_delay, double max_accel, double max_velocity, double default_speed, int dim)
: TrjPlaner(time_delay, max_accel, max_velocity, default_speed, dim){
}

bool CubicSpline::generate_trj(Path const &path, Path const  &speeds) {
    if(path.size() != speeds.size())
        return false;
    int32_t path_size = path.size()-1;
    _poly.reserve(path_size);
    _vronsk_matrix = Eigen::MatrixXd::Zero(4, 4);
    _control_points = Eigen::MatrixXd::Zero(4);
    for( int32_t i = 0; i < path_size; i++){
        for(int d = 0; d < _dim; d++){
            _poly[i] = Eigen::VectorXd::Zero(4);
            _poly[i]<< path[i](d), path[i+1](d), speeds[i](d), speeds[i+1](d);
        }
    }
    return true;
}

Point CubicSpline::point(double t) {

}

