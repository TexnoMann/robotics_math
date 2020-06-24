#include "../pointcloud_vectorize.h"
#include <iostream>
#include <fstream>
#include <string>

#include "../../../../matplotlibcpp/matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main(int argc , char *argv[]) {
    std::cout << "Start linear tls fit test\n";

    std::vector <Eigen::VectorXd> points;
    std::vector<double> xplot_list{0,1,2,3,4,5,6};
    std::vector<double> yplot_list{1,2,-1,8,10,14,16};
    std::vector<double> yest_list;
    Line2dApprox p2l(0.1, 3, 6);
    std::cout << "Point to 2d Line approximation initialize\n";

    for(int i=0; i<xplot_list.size(); i++){
        Eigen::VectorXd point(2);
        point<<xplot_list[i],yplot_list[i];
        points.push_back(point);
        p2l.add_point(point);
    }

    std::cout<<"Vector estim params: \n" <<p2l.line_params<< std::endl;
    for(int i=0; i<xplot_list.size(); i++){
        yest_list.push_back(-p2l.line_params(0)/p2l.line_params(1)*xplot_list[i] - p2l.line_params(2)/p2l.line_params(1));
    }

    plt::figure_size(1200, 780);
    plt::plot(xplot_list, yplot_list);
    plt::plot(xplot_list, yest_list);
    plt::save("./mcllot.png");
}

