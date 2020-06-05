#include "../curves/besie_spline.h"
#include <iostream>
#include <fstream>
#include <string>

int main(int argc , char *argv[]){
    std::cout<<"Start\n";

    std::ofstream file;
    file.open("out.csv");

    Eigen::VectorXd p1 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd p2 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd p3 = Eigen::VectorXd::Zero(3);
    p1<<0,1,1;
    p2<<1,2,1;
    p3<<-2,4,1;

    std::cout<<"Given this points for besie interpolation:\n"<< p1<<"\n\n"<<p2<<"\n\n"<<p3<<std::endl;
    std::cout<<"Generating besie spline!\n";
    Path p;
    p.emplace_back(p1); p.emplace_back(p2); p.emplace_back(p3);
    BesieSpline bspline(3);
    bspline.generate_spline(p);
    std::cout<<bspline.len()<<std::endl;

    file<<"x y l\n";
    for(double l=0.0; l< bspline.len(); l+=0.001){
        Eigen::VectorXd point;
        bspline.point(l, point);
        std::string row = std::to_string(point(0)) + "," + std::to_string(point(1)) + "," + std::to_string(l);
        file<<row<<"\n";
    }
    file.close();

    return 0;
}
