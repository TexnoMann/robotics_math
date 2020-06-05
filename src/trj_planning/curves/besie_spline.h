#pragma once

#include <vector>
#include "math.h"
#include "../trj_planer.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>

struct GaussLengendreCoefficient
{
    float abscissa; // xi
    float weight;   // wi
};

static constexpr GaussLengendreCoefficient gauss_lengendre_coefficients[] =
        {
                { 0.0f, 0.5688889f },
                { -0.5384693f, 0.47862867f },
                { 0.5384693f, 0.47862867f },
                { -0.90617985f, 0.23692688f },
                { 0.90617985f, 0.23692688f }
        };

class BesieSpline{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BesieSpline(int dim);
    void generate_spline(Path const & path);
    double len();
    void point(double dist, Eigen::VectorXd & point);
private:
    int32_t _size;
    double _length;
    double _cart_length;
    int _dim;
    std::vector<double> _part_dist;
    std::vector<double> _cart_part_dist;

    std::vector<double> _full_dist;
    std::vector<double> _cart_full_dist;
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> _control_points;
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> _besie_points;
    Eigen::MatrixXd _hermite_spline_matrix;
    Path _src_path;
    double _get_one_part_len(int part);
    void _derivative_besie_spline(Eigen::VectorXd & p0, Eigen::VectorXd & p1, Eigen::VectorXd & p2, Eigen::VectorXd & p3, Eigen::VectorXd &derivative, double alpha, double t);

};

