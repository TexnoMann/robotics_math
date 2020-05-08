#include "graph_map.h"


GraphMap::GraphMap(int dim){
    _dim = dim;
    _counter = 0;
}

std::vector<GraphPoint> GraphMap::get_neighbors(int id){
    std::map <int, GraphPoint> :: iterator it;
    it = _map.find(id);

    if (it == _map.end()) {
        std::vector<GraphPoint> null_vector;
        return null_vector;
    }

    return it->second.neighbors;
}

int GraphMap::append_point(GraphPoint & graph_point) {
    _map[_counter] = graph_point;
    _id_map[graph_point] = _counter;
    _counter++;
    return _counter-1;
}

bool GraphMap::remove_point(GraphPoint & graph_point) {
    std::map<GraphPoint,int>::iterator it;
    it=_id_map.find(graph_point);
    if (it == _id_map.end()) {
        return false;
    }
    _id_map.erase(it);
    _map.erase(it->second);
    return true;
}

int GraphMap::find_point_by_pos(Eigen::VectorXd coords){
    if(coords.size()!=_dim) return -2;
    std::map <int, GraphPoint> :: iterator it = _map.begin();
    for (int i = 0; it != _map.end(); it++, i++){
        bool comp = true;
        for(int j=0; j<_dim; j++){
            comp &= coords(j)== it->second.coords(j);
        }
        if (comp) return it->first;
    }
    return -1;
}

int GraphMap::find_point_near_pos(Eigen::VectorXd coords) {
    if (coords.size() != _dim) return -2;
    std::map<int, GraphPoint>::iterator it = _map.begin();
    double min_dist=1e+6;
    int id;
    for (int i = 0; it != _map.end(); it++, i++) {
        double dist = (coords - it->second.coords).norm();
        if(dist< min_dist){
            min_dist = dist;
            id = it->first;
        }
    }
    return -1;
}


bool GraphMap::remove_point(int id) {
    std::map<int,GraphPoint>::iterator it;
    it=_map.find(id);
    if (it == _map.end()) {
        return false;
    }
    _map.erase(it);
    _id_map.erase(it->second);
    return true;
}

Eigen::MatrixXd GraphMap::get_adjacency_matrix(){
    Eigen::MatrixXd adj_matrix = Eigen::MatrixXd::Zero(_counter, _counter);
    std::map <int, GraphPoint> :: iterator it = _map.begin();
    for (int i = 0; it != _map.end(); it++, i++){
        std::vector<GraphPoint> neighbors = it->second.neighbors;
        for(int j = 0;j < neighbors.size();j++){
            std::map <GraphPoint, int> :: iterator it_id;
            it_id =_id_map.find(neighbors[j]);
            Eigen::VectorXd vect_btw = neighbors[j].coords - it->second.coords;
            adj_matrix(i,it_id->second) = vect_btw.norm();
            adj_matrix(it_id->second, i) = vect_btw.norm();
        }
    }
    return adj_matrix;
}

GraphPoint GraphMap::point(int id){
    std::map<int,GraphPoint>::iterator it;
    it=_map.find(id);
    if (it == _map.end()) throw("invalid id");
    return it->second;
}

long GraphMap::size() {
    return _map.size();
}

int GraphMap::dim(){
    return _dim;
}


bool GraphPoint::operator <(const GraphPoint& s2) const {
    return this->coords.norm() < s2.coords.norm();
}

void GraphPoint::add_neighbor(const GraphPoint & graph_point) {
    neighbors.push_back(graph_point);
}

bool GraphPoint::remove_neighbor(GraphPoint & graph_point) {

}

std::ostream& operator<< (std::ostream& os, GraphMap & graph_map){
    os << "[graph map] adj_matrix: \n"<< graph_map.get_adjacency_matrix();
    return os;
}


std::ostream& operator<< (std::ostream& os, GraphPoint & graph_point){
    os << "[graph point] coords: \n"<< graph_point.coords<<std::endl;
    return os;
}


GraphPoint::GraphPoint() {

}