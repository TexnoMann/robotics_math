
#include "monte_carlo.h"


long get_random_long(const long &begin, const long &end);

MonteCarlo::MonteCarlo(float norm_eps, long particles_count, int dim, const Eigen::VectorXd & min_border_map, const Eigen::VectorXd & max_border_map, const float sensor_rmse){
    _dim = dim;
    _norm_eps = norm_eps;
    _particles_count = particles_count;

    _min_border_map = min_border_map;
    _max_border_map = max_border_map;

    _sensor_rmse = sensor_rmse;
    _state_rmse = 1e-3;

    _maxweight = 0;
    _weight_sum=0;

    _swarm_particle.reserve(_particles_count);
//    std::cout<<"generate particles"<< std::endl;

    for (long p = 0; p < _particles_count; p++) {
        _swarm_particle[p].resize(_dim);
    }
}

float MonteCarlo::get_pose(Eigen::VectorXd &pose, const std::vector <Eigen::VectorXd> &measure_features) {
    _spawn_particles();
    _measure_update(measure_features);
    pose = Eigen::VectorXd::Zero(_dim);
    for(long p=0; p<_particles_count; p++){
        pose += _swarm_particle[p].weight*_swarm_particle[p].pose;
    }

    return _state_rmse;
}

void MonteCarlo::hard_resample() {
    _resampling_wheel();
}

float MonteCarlo::_gaus_likehood(const Eigen::VectorXd &pose, const Eigen::VectorXd &mean, const float rmse) {
//    std::cout<<pose- mean<< std::endl;
    float n = (pose - mean).norm();
    float likehood = 1.0/(std::sqrt(2*M_PI)*rmse)*(exp(-(n*n)/(2*rmse*rmse)));
//    std::cout<<-(n*n)/(rmse*rmse)<< std::endl;
    return likehood;
}


void MonteCarlo::_uniform_distribution(const Eigen::VectorXd &min_pose, const Eigen::VectorXd max_pose,
                                       Eigen::VectorXd &output) {
    for(int i=0; i<_dim; i++){
        output(i) = _uniform_distribution_1d(min_pose(i), max_pose(i));
    }
}

float MonteCarlo::_uniform_distribution_1d(const float & min_elem, const float & max_elem) {
        std::uniform_real_distribution<float> distribution(min_elem, max_elem);
        return distribution(_random_generator);
}

void MonteCarlo::_spawn_particles() {
    for (long i= 0; i<_particles_count; i++ ){
        _swarm_particle[i].pose = Eigen::VectorXd::Zero(_dim);
        _uniform_distribution(_min_border_map, _max_border_map, _swarm_particle[i].pose);
    }
}

void MonteCarlo::_measure_update(const std::vector<Eigen::VectorXd> & measure_features){
    long count_features = measure_features.size();
    _weight_sum = 0;
    for(long p=0; p<_particles_count; p++) {
        _swarm_particle[p].weight = 1;
        for (long f = 0; f < count_features; f++) {
            _swarm_particle[p].weight *= _gaus_likehood(_swarm_particle[p].pose, measure_features[f], _sensor_rmse);
        }
        _weight_sum+= _swarm_particle[p].weight;
    }

    _normalize();

}

void MonteCarlo::_resampling_wheel() {
    int segment = get_random_long(0, _particles_count);
    float betta = 0.0;
    SwarmParticle new_swarm_particle(_particles_count);
    for(long p=0; p < _particles_count; p++) {
        betta = betta + _uniform_distribution_1d(0, 2*_maxweight);
        while(betta > _swarm_particle[segment].weight) {
            betta = betta - _swarm_particle[segment].weight;
            segment = (segment+1)% _particles_count;
        }
        new_swarm_particle[p] = _swarm_particle[segment];
    }
    _swarm_particle = new_swarm_particle;

    _normalize();


}

void MonteCarlo::_normalize() {
    // Normalisation
    _maxweight = 0.0;
    for(long p=0; p<_particles_count; p++) {
        _swarm_particle[p].weight /= _weight_sum;
        if(_swarm_particle[p].weight > _maxweight)
            _maxweight = _swarm_particle[p].weight;
    }
}

long get_random_long(const long &begin, const long &end) {
    return begin >= end ? 0 : begin + (long) rand()*((end - begin)/RAND_MAX);
};
