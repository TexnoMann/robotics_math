#include "besie_spline.h"


BesieSpline::BesieSpline(int dim) {
    _dim = dim;
    _length = 0;
    _size = 0;
    _cart_length = 0;
}

double BesieSpline::len() {
    return _cart_length;
}
void BesieSpline::generate_spline(Path const & path){
    _length = 0;
    _cart_length = 0;
    _control_points.erase(_control_points.begin(), _control_points.end());
    _besie_points.erase(_besie_points.begin(), _besie_points.end());
    _part_dist.erase(_part_dist.begin(), _part_dist.end());
    _full_dist.erase(_full_dist.begin(), _full_dist.end());
    _cart_part_dist.erase(_part_dist.begin(), _part_dist.end());
    _cart_full_dist.erase(_full_dist.begin(), _full_dist.end());

    _size = path.size();
    _src_path = path;
    _part_dist.reserve(_size-1);
    _full_dist.reserve(_size-1);
    _cart_part_dist.reserve(_size-1);
    _cart_full_dist.reserve(_size-1);
    for(int b=0; b<_dim; b++){
        _control_points.push_back(Eigen::VectorXd::Zero((_size-1)*2));
    }
    _hermite_spline_matrix = Eigen::MatrixXd::Zero((_size-1)*2,(_size-1)*2);


    _hermite_spline_matrix.block(0,0,1,2)<<2,-1;
    _part_dist[0]=0;
    for(int b=0; b<_dim; b++) {
        _control_points[b](0) = _src_path[0](b);
        _part_dist[0] += powf((_src_path[1](b)-_src_path[0](b)),2);
    }

    _part_dist[0] = sqrtf( _part_dist[0]);
    _full_dist[0]= _part_dist[0];
    double last_alpha =1.0/_part_dist[0];
    double alpha;
    _length+=_part_dist[0];
    for(int32_t i=1; i<_size-1; i++) {

        _part_dist[i] = 0;
        for(int b=0; b<_dim; b++) {
            _part_dist[i] += powf((_src_path[i+1](b)-_src_path[i](b)),2);
        }
        _part_dist[i]= sqrtf( _part_dist[i]);

        alpha = 1.0/_part_dist[i];
        _hermite_spline_matrix.block((i-1)*2+1,2*(i-1),2,4)<<1*(last_alpha*last_alpha), - 2*(last_alpha*last_alpha), 2*(alpha*alpha), -1*(alpha*alpha), 0.0, last_alpha, alpha, 0.0;
        for(int b=0; b<_dim; b++){
            _control_points[b]((i-1)*2+1) = (alpha*alpha - last_alpha*last_alpha)*_src_path[i](b); _control_points[b]((i-1)*2+2) = (last_alpha+alpha)*_src_path[i](b);
        }
        last_alpha = alpha;
        _full_dist[i]=_full_dist[i-1]+_part_dist[i];
        _length+=_part_dist[i];
    }
    _hermite_spline_matrix.block(2*(_size-1)-1,2*(_size-1)-2,1,2)<<-1, 2;
//    std::cout<<_hermite_spline_matrix<<std::endl;
    for(int b=0; b<_dim; b++) {
        _control_points[b]((_size-1)*2-1) = _src_path[_size - 1](b);
    }

    for(int b=0; b<_dim; b++) {
        _besie_points.push_back(_hermite_spline_matrix.inverse()*_control_points[b]);
    }

    _cart_part_dist[0] = _get_one_part_len(0);
    _cart_full_dist[0] = _cart_part_dist[0];
    _cart_length +=_cart_part_dist[0];
    for(int part=1; part < (_size-1); part++) {
        _cart_part_dist[part] = _get_one_part_len(part);
        _cart_full_dist[part] = _cart_full_dist[part-1] + _cart_part_dist[part];
        _cart_length +=_cart_part_dist[part];
    }

    }

void BesieSpline::point(double dist, Eigen::VectorXd & point) {
    int part;
    double cart_dist_on_part;
    for(part=0; part<_size-1; part++)
        if( dist <= _cart_full_dist[part]) break;

    if(part!=0)
        cart_dist_on_part = dist-_cart_full_dist[part-1];
    else
        cart_dist_on_part = dist;

    Eigen::VectorXd p0 = _src_path[part];
    Eigen::VectorXd p1 = Eigen::VectorXd::Zero(_dim);
    Eigen::VectorXd p2 = Eigen::VectorXd::Zero(_dim);
    Eigen::VectorXd p3 = _src_path[part+1];

    for(int j=0; j<_dim;j++){
        p1(j) = _besie_points[j](2*part);
        p2(j) = _besie_points[j](2*part+1);
    }

    point = Eigen::VectorXd::Zero(_dim);
    double t = cart_dist_on_part/_cart_part_dist[part];
    point = powf((1 - t),3)*p0 + 3*powf((1 - t),2)*t*p1 + 3*(1 - t)*t*t*p2 + t*t*t*p3;

}

double BesieSpline::_get_one_part_len(int part) {
    Eigen::VectorXd p0 = _src_path[part];
    Eigen::VectorXd p1 = Eigen::VectorXd::Zero(_dim);
    Eigen::VectorXd p2 = Eigen::VectorXd::Zero(_dim);
    Eigen::VectorXd p3 = _src_path[part + 1];

    for (int j = 0; j < _dim; j++) {
        p1(j) = _besie_points[j](2 * part);
        p2(j) = _besie_points[j](2 * part + 1);
    }

    float length = 0.f;
    for (auto coefficient : gauss_lengendre_coefficients)
    {
        float const t = 0.5f * (1.f + coefficient.abscissa); // This and the final (0.5 *) below are needed for a change of interval to [0, 1] from [-1, 1]
        Eigen::VectorXd derive;
        _derivative_besie_spline(p0,p1,p2,p3,derive,1.0, t);
        length += derive.norm() * coefficient.weight;

    }
    return 0.5f * length;
}

void BesieSpline::_derivative_besie_spline(Eigen::VectorXd &p0, Eigen::VectorXd &p1, Eigen::VectorXd &p2,
                                           Eigen::VectorXd &p3,Eigen::VectorXd &derivative, double alpha, double t) {
    derivative = 3*alpha*(-(1-alpha*t)*(1-alpha*t)*p0 + (1-3*alpha*t)*(1-alpha*t)*p1 + alpha*t*(2-3*alpha*t)*p2 + alpha*alpha*t*t*p3);
}

