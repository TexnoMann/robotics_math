#include "../planners/lspb.h"
#include <iostream>
#include <fstream>
#include <string>

int main(int argc , char *argv[]){
    std::cout<<"Start\n";

    std::ofstream file;
    file.open("out.csv");

    Eigen::VectorXd p1 = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd p2 = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd sp1 = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd sp2 = Eigen::VectorXd::Zero(2);

    std::array<double,2> def_speed = {0.1,0.1};
    std::array<double,2> max_accel = {0.02,0.02};

    p1<<0,0;
    p2<<2,2;
    sp1<<0.0,0.0;
    sp2<<0.0,0.0;

    std::cout<<"Given this points for lspb generation:\n"<< p1<<"\n\n"<<p2<<std::endl;
    std::cout<<"Generating linear segments with parabolic blends!\n";

    Lspb lspb(1e-3, max_accel.data(), def_speed.data(), 2);
    lspb.generate_trj(p1,p2,sp1,sp2);
    file<<"x y w l\n";
    for(double t=0.0; t<lspb.end_t(); t+=0.001){
        Eigen::VectorXd point = lspb.point(t).p;
        std::string row = std::to_string(point(0)) + "," + std::to_string(point(1)) + "," + std::to_string(t);
        file<<row<<"\n";
    }
    file.close();

    return 0;
}