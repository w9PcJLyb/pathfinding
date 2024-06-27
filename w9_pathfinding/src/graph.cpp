#include "include/graph.h"


Graph::Graph(int num_vertices) : num_vertices_(num_vertices) {
    edges_.resize(num_vertices_, vector<Edge>(0));
}

size_t Graph::size() const {
    return num_vertices_;
}

size_t Graph::num_edges() const {
    size_t count = 0;
    for (const vector<Edge> &e : edges_) {
        count += e.size();
    }
    return count;
}

void Graph::add_edge(int start, int end, double cost) {
    edges_[start].push_back(Edge(end, cost));
}

void Graph::add_edges(vector<int> starts, vector<int> ends, vector<double> costs) {
    for (size_t i = 0; i < starts.size(); i++) {
        add_edge(starts[i], ends[i], costs[i]);
    }
}

vector<vector<double>> Graph::get_edges() const {
    vector<vector<double>> raw;
    for (int i = 0; i < num_vertices_; i++) {
        for (const Edge &e: edges_[i]) {
            raw.push_back({double(i), double(e.node_id), e.cost});
        }
    }
    return raw;
}

vector<pair<int, double>> Graph::get_neighbours(int node) const {
    vector<pair<int, double>> nb;
    for (const Edge &e : edges_[node]) {
        nb.push_back({e.node_id, e.cost});
    }
    return nb;
}

double Graph::estimate_distance(int v1, int v2) const {
    throw std::runtime_error("Unable to estimate distance for graph.");
}

AbsGraph* Graph::reverse() const {
    Graph* reversed_graph(new Graph(num_vertices_));
    for (int i = 0; i < num_vertices_; i++) {
        for (const Edge &e: edges_[i]) {
            reversed_graph->add_edge(e.node_id, i, e.cost);
        }
    }
    return reversed_graph;
}
