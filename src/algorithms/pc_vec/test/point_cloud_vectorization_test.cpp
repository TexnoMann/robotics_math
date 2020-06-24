#include "../pointcloud_vectorize.h"
#include <iostream>
#include <fstream>
#include <string>

#include "../../../../matplotlibcpp/matplotlibcpp.h"

namespace plt = matplotlibcpp;

float get_uniform_rand_float(float min, float max);

int main(int argc , char *argv[]) {
    std::cout << "Start linear tls fit test\n";

    PointCloudToVec2d pc2v(1, 10, 90);
    std::cout << "Line Vectorization algorithm initialize\n";
    std::vector<double> xplot_list;
    std::vector<double> yplot_list;
    std::vector<double> yest_list;
    std::vector <Line2dApprox> cloud_parts;
    int c=0;
    for(double x=0; x<10.0; x+=0.1){
        Eigen::VectorXd point(2);
        double xg = x + get_uniform_rand_float(-1,1);
        point<< xg, 2.5*xg + 3.5 + get_uniform_rand_float(-1,1);
        xplot_list.push_back(point(0));
        yplot_list.push_back(point(1));
        pc2v.add_point(point);
        c++;
    }
    for(double x=9.9; x<15.0; x+=0.05){
        Eigen::VectorXd point(2);
        double xg = x + get_uniform_rand_float(-1,1);
        point<< xg, 10*xg -65 + get_uniform_rand_float(-1,1);
        xplot_list.push_back(point(0));
        yplot_list.push_back(point(1));
        pc2v.add_point(point);
    }

    pc2v.cloud_lines(cloud_parts);
    std::cout<<"Count finded lines: \n" <<cloud_parts.size()<< std::endl;
    std::cout<<"First line params: \n" <<cloud_parts[0].line_params<< std::endl;
    Eigen::VectorXd center_mass_line;
    cloud_parts[0].center_mass_line(center_mass_line);
    std::cout<<"First line center_mass: \n" <<center_mass_line<< std::endl;
    std::cout<<"First line init_point: \n" <<cloud_parts[0].init_point<< std::endl;
    std::cout<<"First line finish_point: \n" <<cloud_parts[0].finish_point<< std::endl;
    auto p2l = cloud_parts[0];
    for(int i=0; i<c; i++){
        yest_list.push_back(-p2l.line_params(0)/p2l.line_params(1)*xplot_list[i] - p2l.line_params(2)/p2l.line_params(1));
    }
    std::cout<<"Second lines params: \n" <<cloud_parts[1].line_params<< std::endl;
    std::cout<<"Second line init_point: \n" <<cloud_parts[1].init_point<< std::endl;
    std::cout<<"Second line finifsh_point: \n" <<cloud_parts[1].finish_point<< std::endl;
    auto p2l2 = cloud_parts[1];
    for(int i=c; i<xplot_list.size(); i++){
        yest_list.push_back(-p2l2.line_params(0)/p2l2.line_params(1)*xplot_list[i] - p2l2.line_params(2)/p2l2.line_params(1));
    }
//
    plt::figure_size(1200, 780);
    plt::plot(xplot_list, yplot_list);
    plt::plot(xplot_list, yest_list);
    plt::show();
////    plt::save("./result/pcvec_test.png");
}

float get_uniform_rand_float(float min, float max){
    return min + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max-min)));
}

