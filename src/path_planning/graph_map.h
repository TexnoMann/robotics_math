#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <map>

class GraphPoint{
public:
    GraphPoint();
    GraphPoint(Eigen::VectorXd coords){
        coords = coords;
        neighbors.reserve(0);
    };
    GraphPoint(int id, Eigen::VectorXd coords, std::vector<int> neighbors){
        coords = coords;
        neighbors = neighbors;
    }
    int id;
    Eigen::VectorXd coords;
    std::vector<GraphPoint> neighbors;

    bool operator <(const GraphPoint& gp2) const;

};


class GraphMap {
public:
    void append_point(GraphPoint graph_point);
    void remove_point(int id);
    Eigen::MatrixXd get_adjacency_matrix();
    std::vector<GraphPoint> get_neighbors(int id);
    int size();
private:
    std::map <int, GraphPoint> _map;
    std::map <GraphPoint, int> _id_map;
    int _counter;

};
