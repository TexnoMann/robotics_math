#include "graph_map.h"


std::vector<GraphPoint> GraphMap::get_neighbors(int id) {
    std::map <int, GraphPoint> :: iterator it;
    it = _map.find(id);

    if (it == _map.end()) {
        std::vector<GraphPoint> null_vector;
        return null_vector;
    }

    return it->second.neighbors;
}

void GraphMap::append_point(GraphPoint graph_point) {
    _map[_counter] = graph_point;
    _id_map[graph_point] = _counter;
    _counter++;
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
        }
    }
}

bool GraphPoint::operator <(const GraphPoint& s2) const {
    return this->coords.norm() < s2.coords.norm();
}

GraphPoint::GraphPoint() {

}