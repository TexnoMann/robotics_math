#pragma once

#include <vector>
#include "math.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <experimental/random>

#define DEFAULT_DIM_VECTOR 3

struct Particle{
public:
    Eigen::VectorXd pose;
    float weight;
    Particle(int dim, const Eigen::VectorXd in_pose){
        _dim = dim;
        pose  = in_pose;
        weight = 0;
    }
    Particle(int dim){
        _dim = dim;
        pose  = Eigen::VectorXd::Zero(_dim);
        weight= 0;
    }
    Particle(){
        _dim = 0;
        weight= 0;
    }
    void resize(int dim){
        _dim = dim;
        pose  = Eigen::VectorXd::Zero(_dim);
        weight= 0;
    }
    int dim(){return _dim;}
private:
    int _dim;
};

typedef std::vector<Particle> SwarmParticle;

class MonteCarlo {
public:
    MonteCarlo(float norm_eps, long particles_count, int dim, const Eigen::VectorXd &min_border_map, const Eigen::VectorXd &max_border_map, const float sensor_rmse);
    float get_pose(Eigen::VectorXd &pose, const std::vector <Eigen::VectorXd> &measure_features);
    void hard_resample();

private:
    int _dim;
    long _particles_count;
    float _norm_eps;
    float _sensor_rmse;
    float _state_rmse;
    float _maxweight;
    float _weight_sum;
    Eigen::VectorXd _min_border_map;
    Eigen::VectorXd _max_border_map;
    std::default_random_engine _random_generator;

    SwarmParticle _swarm_particle;

    void _normalize();
    float _gaus_likehood(const Eigen::VectorXd & pose, const Eigen::VectorXd & mean, const float rmse);
    void _uniform_distribution(const Eigen::VectorXd & min_pose, const Eigen::VectorXd max_pose, Eigen::VectorXd & output);
    float _uniform_distribution_1d(const float & min_elem, const float & max_elem);
    void _measure_update(const std::vector<Eigen::VectorXd> & measure_features);

    void _spawn_particles();

    void _resampling_wheel();
};

