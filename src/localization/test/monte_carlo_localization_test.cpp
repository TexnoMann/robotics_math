#include "../monte_carlo.h"
#include <iostream>
#include <fstream>
#include <string>

#include "../../../matplotlibcpp/matplotlibcpp.h"

namespace plt = matplotlibcpp;

float get_uniform_rand_float(float min, float max);


int main(int argc , char *argv[]) {

    int  stime;
    long ltime;
    ltime = time (NULL);
    stime = (unsigned int) ltime/2;
    srand(stime);

    std::cout << "Start\n";

    std::vector <Eigen::VectorXd> features;
    Eigen::VectorXd f1(2);
    Eigen::VectorXd f2(2);
    Eigen::VectorXd f3(2);
    Eigen::VectorXd f4(2);

    Eigen::VectorXd real_pose(2);

    f1 << 1.0, 1.0;
    f2 << -1.0, 1.0;
    f3 << -1.0, -1.0;
    f4 << 1.0, -1.0;
    features.push_back(f1); features.push_back(f2); features.push_back(f3); features.push_back(f4);

    real_pose << -6.0, 6.0;

    Eigen::VectorXd borders_min(2);
    Eigen::VectorXd borders_max(2);
    borders_min<< -10.0, -10.0;
    borders_max<< 10.0,10.0;

    MonteCarlo mcl(1e-2, 1e+2,2,borders_min,borders_max, 1);

    std::cout << "Given this points for features:\n" << f1 << "\n\n" << f2 << "\n\n" << f3 << "\n\n" << f4 << std::endl;
    int count_itter = 1e+2;
    std::cout << "Generate measurements\n";

    Eigen::VectorXd estim_pose = Eigen::VectorXd::Zero(2);
    std::vector<double> xplot;
    std::vector<double> yplot;

    for(int i=0; i<count_itter; ++i) {
        std::vector<Eigen::VectorXd> measures;
        Eigen::VectorXd meas = Eigen::VectorXd::Zero(2);
        meas(0) = real_pose(0) + get_uniform_rand_float(-1e-1, 1e-1);
        meas(1) = real_pose(1) + get_uniform_rand_float(-1e-1, 1e-1);
        measures.push_back(meas);
        mcl.get_pose(estim_pose, measures);
        xplot.push_back(estim_pose(0));
        yplot.push_back(estim_pose(1));
        std::cout<<"Calculated position: "<< estim_pose(0)<<", "<<estim_pose(1) << std::endl;
    }
    plt::plot(xplot, yplot);
    // sleep(3);
    plt::save("./mcllot.png");
}

float get_uniform_rand_float(float min, float max){
    return min + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max-min)));
}
