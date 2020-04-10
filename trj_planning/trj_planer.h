#pragma once

#include <Eigen/Core>
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
    double dist;
    Eigen::VectorXd p;

    Point(int dim){
        ts = 0;
        dist=0;
        _dim=0;
        p = Eigen::VectorXd::Zero(_dim);
    }

    Point(double time, double distance, Eigen::VectorXd pointVec){
        _dim = pointVec.size();
        ts = time;
        dist = distance;
        p  = pointVec;
    }
};


class TrjPlaner{
public:
  virtual ~TrjPlaner() {}
  TrjPlaner(double time_delay, double max_accel, double max_velocity, double default_speed, int dim){
      _dim = dim;
      _time_delay = time_delay;
      _max_accel = max_accel;
      _max_velocity = max_velocity;
      _default_speed = default_speed;
  }
  virtual bool generate_trj(const Path & path, const Path & speeds) = 0;
  virtual void trj(Trj & trj){
      trj = _trj;
  }
  virtual Point point(double t);

protected:
  Trj _trj;
  int _dim = 0;
  double _time_delay = 0;
  double _max_accel = 0;
  double _max_velocity = 0;
  double _default_speed = 0;
};
