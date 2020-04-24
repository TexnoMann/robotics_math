#include "../curves/besie_spline.h"
#include <iostream>
#include <fstream>
#include <string>

int main(int argc , char *argv[]){
    std::cout<<"Start\n";

    std::ofstream file;
    file.open("out.csv");

    Eigen::VectorXd p1 = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd p2 = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd p3 = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd p4 = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd p5 = Eigen::VectorXd::Zero(2);
    p1<<0,1;
    p2<<0,2;
    p3<<0,4;
    p4<<2,4;
    p5<<4,8;
    std::cout<<"Given this points for besie interpolation:\n"<< p1<<"\n\n"<<p2<<"\n\n"<<p3<<"\n\n"<<p4<<std::endl;
    std::cout<<"Generating besie spline!\n";
    Path p;
    p.emplace_back(p1); p.emplace_back(p2); p.emplace_back(p3); p.emplace_back(p4); p.emplace_back(p5);
    BesieSpline bspline(2);
    bspline.generate_spline(p);

    file<<"x y l\n";
    for(double l=0.0; l< bspline.len(); l+=0.001){
        Eigen::VectorXd point = bspline.point(l);
        std::string row = std::to_string(point(0)) + "," + std::to_string(point(1)) + "," + std::to_string(l);
        file<<row<<"\n";
    }
    file.close();

    return 0;
}
