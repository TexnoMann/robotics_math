
#include "lspb.h"

Lspb::~Lspb() {}

Lspb::Lspb(double time_delay,double *max_accel, double *default_speed, int dim):
        TrjPlaner(time_delay, max_accel, default_speed, dim){
    _tc = new double[dim];
    _tb = new double[dim];
    _tr = new double[dim];
    _tf = new double[dim];
    _dir_speed = new int[dim];
    _dir_accel = new int[dim];
    _dir_accel_back = new int[dim];
    _default_ch_speed = new double[dim];
    _max_ch_accel = new double[dim];
    _max_ch_accel_back = new double[dim];
}

bool Lspb::generate_trj(const Eigen::VectorXd & start_pose, const Eigen::VectorXd finish_pose, const Eigen::VectorXd start_speed, const Eigen::VectorXd finish_speed) {
    _start_pose = start_pose;
    _finish_pose = finish_pose;
    _start_speed = start_speed;
    _finish_speed = finish_speed;
    _tf_max = 0.0;
    if( start_pose.size()!= _dim || finish_pose.size() != _dim)
        return false;
    for(int i=0; i<_dim; i++){

        //change speed for common vector components ending;
        _default_ch_speed[i] = _default_speed[i];
        _max_ch_accel[i] = _max_accel[i];

        _dir_speed[i] = std::copysign(1, finish_pose[i] - start_pose[i]);
        _dir_accel[i] = std::copysign(1, _dir_speed[i] * _default_speed[i] - start_speed[i]);
        _dir_accel_back[i] = std::copysign(1, finish_speed[i] - _dir_speed[i] * _default_speed[i]);

        //first parabolic blend
        _tb[i] = std::fabs(_dir_speed[i] * _default_speed[i] - start_speed[i]) / _max_accel[i];


        // second parabolic blend
        _tc[i] = (finish_speed[i]-_default_speed[i])/_max_accel[i]*_dir_accel_back[i];

        //time for velocity constant moving
        _tr[i] = (finish_pose[i] - start_pose[i] - start_speed[i] * _tb[i] -
                    _max_accel[i] * _dir_accel[i] * _tb[i] * _tb[i] / 2
                    - _default_speed[i]*_dir_speed[i]*_tc[i] - _max_accel[i]*_dir_accel_back[i]*_tc[i]*_tc[i]/2)/(_default_speed[i]*_dir_speed[i]);
        // estimation full time for moving
        _tf[i] = _tb[i]+_tc[i]+_tr[i];
        std::cout<<"tb "<< std::to_string(i)<<" :"<<_tb[i]<<std::endl;
        std::cout<<"tc "<< std::to_string(i)<<" :"<<_tc[i]<<std::endl;
        std::cout<<"tr "<< std::to_string(i)<<" :"<<_tr[i]<<std::endl;
        if(_tf[i]>_tf_max){
            _tf_max = _tf[i];
            _l_comp = i;
        }
    }

    //rewrite for new speed
    for(int i = 0; i<_dim; i++){
        if(i==_l_comp) continue;
        _tr[i] += (_tf[_l_comp] - _tf[i]);
        std::cout<<"def_s "<< std::to_string(i)<<" :"<<_default_ch_speed[i]<<std::endl;
        _default_ch_speed[i] = std::fabs((_finish_pose[i] - _start_pose[i] - _start_speed[i]*_tb[i] +
                _dir_accel[i]*_max_accel[i]*_start_speed[i]/(_dir_speed[i]*_default_speed[i]-_start_speed[i])*_tb[i]*_tb[i]/2 -
                _dir_accel_back[i]*_max_accel[i]*_finish_speed[i]/(_finish_speed[i]-_dir_speed[i]*_default_speed[i])*_tc[i]*_tc[i]/2)/
                (_tr[i] - _dir_accel[i]*_max_accel[i]/(_dir_speed[i]*_default_speed[i] - _start_speed[i]) + _dir_accel_back[i]*_max_accel[i]/(_finish_speed[i]-_dir_speed[i]*_default_speed[i])));
        _tf[i] = _tf[_l_comp];
        _max_ch_accel[i] = std::fabs(_max_accel[i] * (_dir_speed[i]*_default_ch_speed[i]-_start_speed[i]) / (_dir_speed[i]*_default_speed[i]-_start_speed[i]));
        _max_ch_accel_back[i] = std::fabs(_max_accel[i] * (_finish_speed[i] - _dir_speed[i]*_default_ch_speed[i]) / (_finish_speed[i] - _dir_speed[i]*_default_speed[i]));
        std::cout<<"def_s "<< std::to_string(i)<<" :"<<_default_ch_speed[i]<<std::endl;
        std::cout<<"tb "<< std::to_string(i)<<" :"<<_tb[i]<<std::endl;
        std::cout<<"tc "<< std::to_string(i)<<" :"<<_tc[i]<<std::endl;
        std::cout<<"tr "<< std::to_string(i)<<" :"<<_tr[i]<<std::endl;
        std::cout<<"tf "<< std::to_string(i)<<" :"<<_tf[i]<<std::endl;
    }


    return true;
}

Point Lspb::point(double t) {

    Eigen::VectorXd pointVec = Eigen::VectorXd::Zero(_dim);
    if(t<0.0)
        return Point(t,pointVec);

    for(int i = 0; i < _dim; i++){
        pointVec[i] = _start_pose[i];
        if(t<= _tb[i]) {
            pointVec[i] += _start_speed[i] * t + _dir_accel[i] * _max_ch_accel[i] * t * t / 2;
        }
        else if(t > _tb[i] && t <= (_tb[i] + _tr[i])) {
            pointVec[i] += _start_speed[i] * _tb[i] + _dir_accel[i] * _max_ch_accel[i] * _tb[i] * _tb[i] / 2;
            pointVec[i] += _default_ch_speed[i] * _dir_speed[i] * (t - _tb[i]);
        }
        else{
            pointVec[i] += _start_speed[i] * _tb[i] + _dir_accel[i] * _max_ch_accel[i] * _tb[i] * _tb[i] / 2;
            pointVec[i] += _default_ch_speed[i] * _dir_speed[i] * _tr[i];
            pointVec[i] += _dir_speed[i]*_default_ch_speed[i] * (t - _tb[i] - _tr[i]) + _max_ch_accel_back[i] * _dir_accel_back[i] * (t - _tb[i] - _tr[i])*(t - _tb[i] - _tr[i])/2.0;
        }
    }
    Point p(t,pointVec);
    return p;
}

double Lspb::end_t(){
    return _tf[0];
}

