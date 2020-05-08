#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <map>
#include <math.h>
#include <memory>

class GraphPoint{
public:
    GraphPoint();
    GraphPoint(Eigen::VectorXd coords){
        this->coords = coords;
        neighbors.reserve(0);
    };
    GraphPoint(int id, Eigen::VectorXd coords, std::vector<GraphPoint> neighbors){
        this->coords = coords;
        this->neighbors = neighbors;
    }
    Eigen::VectorXd coords;
    std::vector<GraphPoint> neighbors;
    void add_neighbor(const GraphPoint & graph_point);
    bool remove_neighbor(GraphPoint & graph_point);
    bool operator <(const GraphPoint& gp2) const;
    friend std::ostream& operator<< (std::ostream& os, GraphPoint & graph_map);

};

class GraphMap;
typedef std::shared_ptr<GraphMap> GraphMapPtr;

class GraphMap {
public:
    GraphMap(int dim);
    int append_point(GraphPoint & graph_point);
    bool remove_point(int id);
    bool remove_point(GraphPoint & graph_point);
    int find_point_by_pos(Eigen::VectorXd coords);
    Eigen::MatrixXd get_adjacency_matrix();
    std::vector<GraphPoint> get_neighbors(int id);
    int find_point_near_pos(Eigen::VectorXd coords);
    GraphPoint point(int id);
    long size();
    int dim();
    friend std::ostream& operator<< (std::ostream& os, GraphMap & graph_map);
private:
    std::map <int, GraphPoint> _map;
    std::map <GraphPoint, int> _id_map;
    int _dim;
    long _counter;

};
