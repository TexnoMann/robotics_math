#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <vector>

struct Point;

using Path = std::vector<Eigen::VectorXd>;
using Trj = std::vector<Point>;

struct Point{
private:
  int _dim;
public:
    double ts;
    Eigen::VectorXd p;

    Point(int dim){
        ts = 0;
        _dim=0;
        p = Eigen::VectorXd::Zero(_dim);
    }

    Point(double time, Eigen::VectorXd pointVec){
        _dim = pointVec.size();
        ts = time;
        p  = pointVec;
    }
};


class TrjPlaner{
public:
  virtual ~TrjPlaner() = 0;
  TrjPlaner(double time_delay,double *max_accel, double *default_speed, int dim);
  virtual bool generate_trj(const Eigen::VectorXd & start_pose, const Eigen::VectorXd finish_pose, const Eigen::VectorXd start_speed, const Eigen::VectorXd finish_speed) = 0;
  virtual double end_t() = 0;
  virtual Point point(double t);

protected:
  Trj _trj;
  int _dim = 0;
  double _time_delay = 0;
  double *_max_accel;
  double *_max_velocity;
  double *_default_speed;
};
