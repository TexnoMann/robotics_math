#include "../graph_map.h"
#include <iostream>
#include <fstream>
#include <string>

int main(int argc , char *argv[]){
    GraphMap graphmap(3);

    Eigen::VectorXd p1 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd p2 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd p3 = Eigen::VectorXd::Zero(3);
    p1<<0,1,1;
    p2<<1,2,1;
    p3<<3,4,1;

    GraphPoint pg1(p1);
    GraphPoint pg2(p2);
    GraphPoint pg3(p3);
    pg3.add_neighbor(pg1);
    pg3.add_neighbor(pg2);

    std::cout<<"Add in map points"<<std::endl;

    graphmap.append_point(pg1);
    graphmap.append_point(pg2);
    graphmap.append_point(pg3);
    std::cout<<"Adj matrix is complete!"<<std::endl;
//
    std::cout<<graphmap;
    return 0;
}
