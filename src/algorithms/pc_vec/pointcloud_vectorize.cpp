#include "pointcloud_vectorize.h"

Line2dApprox::Line2dApprox(double max_dist, int threshold_min, int threshold_max) {
    _max_dist = max_dist;
    _threshold_min = threshold_min;
    _threshold_max = threshold_max;
    init_point = Eigen::VectorXd::Zero(2);
    finish_point = Eigen::VectorXd::Zero(2);
    line_params = Eigen::VectorXd::Zero(3);
    _center_mass_point = Eigen::VectorXd::Zero(2);
    _is_packet = false;
    _index_newpoint = 0;
    _sumx = 0;
    _sumy = 0;
    _sumxy = 0;
    _sumx2 = 0;
    _sumy2 = 0;
}

bool Line2dApprox::point_near_line(const Eigen::VectorXd & point, double & dist) {
    dist = std::abs(line_params(0)*point(0) + line_params(1)*point(1) + line_params(2));
//    std::cout<<"Given point with dist: "<< dist<< std::endl;
    if(_associate_point_cloud.size() < _threshold_min) {
        return true;
    }
    else{
        _is_packet = true;
        if(dist> _max_dist)
            return false;
        else
            return true;
    }

}

void Line2dApprox::add_point(Eigen::VectorXd point){
    if(_associate_point_cloud.size()==_threshold_max){

        _center_mass_point -= _associate_point_cloud[_index_newpoint]/_associate_point_cloud.size();

        _sumx-= _associate_point_cloud[_index_newpoint](0);
        _sumy-= _associate_point_cloud[_index_newpoint](1);
        _sumxy-= _associate_point_cloud[_index_newpoint](0) * _associate_point_cloud[_index_newpoint](1);
        _sumx2-= _associate_point_cloud[_index_newpoint](0) *_associate_point_cloud[_index_newpoint](0);
        _sumy2-= _associate_point_cloud[_index_newpoint](1) *_associate_point_cloud[_index_newpoint](1);

        _associate_point_cloud[_index_newpoint]= point;

        _center_mass_point+= point/_associate_point_cloud.size();


    }
    else {
        _associate_point_cloud.push_back(point);
        _center_mass_point=_center_mass_point*(_associate_point_cloud.size()-1)/_associate_point_cloud.size()+ point/_associate_point_cloud.size();
    }
    _sumx+= _associate_point_cloud[_index_newpoint](0);
    _sumy+= _associate_point_cloud[_index_newpoint](1);
    _sumxy+= _associate_point_cloud[_index_newpoint](0) * _associate_point_cloud[_index_newpoint](1);
    _sumx2+= _associate_point_cloud[_index_newpoint](0) *_associate_point_cloud[_index_newpoint](0);
    _sumy2+= _associate_point_cloud[_index_newpoint](1) *_associate_point_cloud[_index_newpoint](1);

    if(_associate_point_cloud.size() >= _threshold_min){
        _fit();
    }

    // Estim intersections

    if (init_point == Eigen::VectorXd::Zero(2) || finish_point == Eigen::VectorXd::Zero(2)){
        init_point = point;
        finish_point = point;
    }
    else{
        init_point(0) = point(0)<=init_point(0) ? point(0): init_point(0);
        init_point(1) = point(1)<=init_point(1)? point(1): init_point(1);
        finish_point(0) = point(0)>=finish_point(0)? point(0): finish_point(0);
        finish_point(1) = point(1)>=finish_point(1)? point(1): finish_point(1);
    }


    _index_newpoint+=1;
    if(_index_newpoint>=_threshold_max)
        _index_newpoint = 0;
}

bool Line2dApprox::is_packet() {
    return _is_packet;
}

void Line2dApprox::center_mass_line(Eigen::VectorXd &center_mass) {
    center_mass = _center_mass_point;
}

void Line2dApprox::_fit(){
    long N =_associate_point_cloud.size();
    double p  = ((_sumx*_sumy) - N*_sumxy);
    double q = (N*_sumx2 - N*_sumy2 - _sumx*_sumx + _sumy*_sumy);
    double n = p/q;
    if(q>0)
        line_params(0) = std::sqrt(1/2.0 - 1/(2*std::sqrt(4*n*n+1)));
    else
        line_params(0) = std::sqrt(1/2.0 + 1/(2*std::sqrt(4*n*n+1)));

    if(p>0)
        line_params(1) = std::sqrt(1-line_params(0)*line_params(0));
    else
        line_params(1) = -std::sqrt(1-line_params(0)*line_params(0));

    line_params(2) = - 1.0/N*(line_params(0)*_sumx + line_params(1)*_sumy);
}

PointCloudToVec2d::PointCloudToVec2d(double max_dist, int threshold_min, int threshold_max) {
    _max_dist = max_dist;
    _threshold_min= threshold_min;
    _threshold_max = threshold_max;
}

bool PointCloudToVec2d::add_point(const Eigen::VectorXd &potential_point) {
    if (potential_point.size() !=2)
        return false;
    double min_distp = 1.0e+6;
    long cloud_part_index=-1;
    double dist = 0;
    bool is_near = false;
    for( long l = 0; l< _cloud_part.size(); l++){
        is_near |= _cloud_part[l].point_near_line(potential_point, dist);
        if(dist < min_distp){
            min_distp = dist;
            cloud_part_index = l;
        }
    }
    if(is_near){
        _cloud_part[cloud_part_index].add_point(potential_point);
    }
    else if(_cloud_part.size()==0){
        //Generate first line
        Line2dApprox p2l(_max_dist, _threshold_min, _threshold_max);
        _cloud_part.push_back(p2l);
        _cloud_part[_cloud_part.size() - 1].add_point(potential_point);
    }
    else{
        //Generate new line
        std::cout<<"Generate new line: "<< dist<< std::endl;
        Eigen::VectorXd init_point_neiborgh =_cloud_part[cloud_part_index].init_point;
        Eigen::VectorXd finish_point_neiborgh = _cloud_part[cloud_part_index].finish_point;
        if((potential_point - init_point_neiborgh).norm() < _max_dist*2) {
            Line2dApprox p2l(_max_dist, _threshold_min, _threshold_max);
            _cloud_part.push_back(p2l);
            _cloud_part[_cloud_part.size() - 1].add_point(potential_point);
            _intersection_points.push_back(init_point_neiborgh);
        }
        else if((potential_point - finish_point_neiborgh).norm() < _max_dist*2) {
            Line2dApprox p2l(_max_dist, _threshold_min, _threshold_max);
            _cloud_part.push_back(p2l);
            _cloud_part[_cloud_part.size() - 1].add_point(potential_point);
            _intersection_points.push_back(finish_point_neiborgh);
        }
        else{
            Line2dApprox p2l(_max_dist, _threshold_min, _threshold_max);
            _cloud_part.push_back(p2l);
            _cloud_part[_cloud_part.size() - 1].add_point(potential_point);
        }
    }
    return true;
}

void PointCloudToVec2d::cloud_lines(CLines &cloud_part) {
    cloud_part = _cloud_part;
}

void PointCloudToVec2d::intersection_points(PointsArray & intersection_points){
    intersection_points = _intersection_points;
}



