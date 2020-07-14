#pragma once
#include <vector>
#include <Eigen/SVD>
#include <iostream>

struct Line2dApprox;
using CLines = std::vector<Line2dApprox, Eigen::aligned_allocator<Line2dApprox>>;
using PointsArray = std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>>;


struct Line2dApprox{
private:
    PointsArray _associate_point_cloud;
    double _max_dist;
    long _threshold_min;
    long _threshold_max;
    void _fit();
    bool _is_packet;
    long _index_newpoint;
    Eigen::VectorXd _center_mass_point;
    float _min_t;
    float _max_t;
    double _sumx;
    double _sumy;
    double _sumxy;
    double _sumx2;
    double _sumy2;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Line2dApprox(double max_dist, int threshold_min, int threshold_max);
    void add_point(Eigen::VectorXd  point);
    bool point_near_line(const Eigen::VectorXd & point, double & dist);
    void center_mass_line(Eigen::VectorXd & center_mass);
    bool is_packet();
    Eigen::VectorXd line_params;
    Eigen::VectorXd init_point;
    Eigen::VectorXd finish_point;
};

class PointCloudToVec2d{
private:
    CLines  _cloud_part;
    PointsArray _intersection_points;
    double _max_dist;
    long _threshold_min;
    long _threshold_max;
    void _filt();
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PointCloudToVec2d(double max_dist, int threshold_min, int threshold_max);
    bool add_point(const Eigen::VectorXd  & potential_point);
    void cloud_lines(CLines & cloud_part);
    void intersection_points(PointsArray & _intersection_points);
};
