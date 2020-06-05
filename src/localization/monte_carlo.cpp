
#include "monte_carlo.h"

MonteCarlo::MonteCarlo(float norm_eps, long particles_count, int dim, const Eigen::VectorXd min_border_map, const Eigen::VectorXd max_border_map) {
    _dim = _dim;
    _norm_eps = norm_eps;
    _particles_count = particles_count;

    _min_border_map = min_border_map;
    _max_border_map = max_border_map;

    _swarm_particle.reserve(_particles_count);
}

float MonteCarlo::_gaus_likehood(const Eigen::VectorXd &pose, const Eigen::VectorXd &mean, const float rmse) {

    return 1/(std::sqrt(2*M_PI*rmse*rmse))*pose.norm();
}

void MonteCarlo::_uniform_distribution(const Eigen::VectorXd &min_pose, const Eigen::VectorXd max_pose,
                                       Eigen::VectorXd &output) {
    for(int i=0; i<_dim; i++){
        std::uniform_real_distribution<float> distribution(min_pose(i), max_pose(i));
        output(i) = distribution(_random_generator);
    }
}

void MonteCarlo::_spawn_particles() {
    for (long i= 0; i<_particles_count; i++ ){

    }
}

