#pragma once

#include "../trj_planer.h"

class Lspb : public TrjPlaner{
private:
    double *_tc;
    double *_tb;
    double *_tr;
    double *_tf;
    int *_dir_speed;
    int *_dir_accel;
    int *_dir_accel_back;
    Eigen::VectorXd _start_pose;
    Eigen::VectorXd _finish_pose;
    Eigen::VectorXd _start_speed;
    Eigen::VectorXd _finish_speed;
    double _tf_max = 0.0;
    int _l_comp;
    double *_default_ch_speed;
    double *_max_ch_accel;
    double *_max_ch_accel_back;
public:
    ~Lspb();
    Lspb(double time_delay,double *max_accel, double *default_speed, int dim);
    bool generate_trj(const Eigen::VectorXd & start_pose, const Eigen::VectorXd finish_pose, const Eigen::VectorXd start_speed, const Eigen::VectorXd finish_speed) override;
    Point point(double t) override;
    double end_t() override;

};


