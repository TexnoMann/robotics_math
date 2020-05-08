#pragma once

#include <vector>
#include "math.h"
#include "../trj_planer.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>

class BesieSpline{
public:
    BesieSpline(int dim);
    void generate_spline(Path const & path);
    double len();
    Eigen::VectorXd point(double dist);
private:
    int32_t _size;
    double _length;
    int _dim;
    std::vector<double> _part_dist;

    std::vector<double> _full_dist;
    std::vector<Eigen::VectorXd> _control_points;
    std::vector<Eigen::VectorXd> _besie_points;
    Eigen::MatrixXd _hermite_spline_matrix;
    Path _src_path;

};

