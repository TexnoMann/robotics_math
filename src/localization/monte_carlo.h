#pragma once

#include <vector>
#include "math.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <random>

struct Particle{
public:
    Eigen::VectorXd pose;
    float weight;
    Particle(int dim, const Eigen::VectorXd & in_pose){
        _dim = dim;
        pose  = in_pose;
    }
    Particle(int dim){
        _dim = dim;
        pose  = Eigen::VectorXd::Zero(_dim);
    }
    int dim(){return _dim;}
private:
    int _dim;
};

typedef std::vector<Particle> SwarmParticle;

class MonteCarlo {
public:
    MonteCarlo(float norm_eps, long particles_count, int dim, const Eigen::VectorXd min_border_map, const Eigen::VectorXd max_border_map);
    void get_pose(const Eigen::VectorXd & pose);

private:
    int _dim;
    long _particles_count;
    float _norm_eps;
    Eigen::VectorXd _min_border_map;
    Eigen::VectorXd _max_border_map;
    std::default_random_engine _random_generator;

    SwarmParticle _swarm_particle;

    void _normalize();
    float _gaus_likehood(const Eigen::VectorXd & pose, const Eigen::VectorXd & mean, const float rmse);
    void _uniform_distribution(const Eigen::VectorXd & min_pose, const Eigen::VectorXd max_pose, Eigen::VectorXd & output);

    void _spawn_particles();
    void _resample();
};

