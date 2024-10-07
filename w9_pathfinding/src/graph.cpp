#include "include/graph.h"
#define _unused(x) ((void)(x))


Graph::Graph(int num_vertices, bool directed) : num_vertices_(num_vertices), directed_(directed) {
    edges_.resize(num_vertices_, vector<Edge>(0));
}

Graph::Graph(int num_vertices, bool undirected, vector<vector<double>> coordinates) : Graph(num_vertices, undirected) {
    set_coordinates(coordinates);
}

bool Graph::is_directed_graph() const {
    return directed_;
}

void Graph::set_coordinates(vector<vector<double>> &coordinates) {
    if (coordinates.size() == 0)
        return;

    assert(int(coordinates.size()) == num_vertices_);
    if (num_vertices_ > 0) {
        size_t d = coordinates[0].size();
        _unused(d);
        assert(d > 0);
        for (const vector<double> &x : coordinates) {
            _unused(x);
            assert(x.size() == d);
        }
    }
    coordinates_ = coordinates;
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
    if (cost < min_weight_)
        min_weight_ = cost;
    edges_[start].push_back(Edge(end, cost));
    if (!directed_)
        edges_[end].push_back(Edge(start, cost));
    else if (!reversed_edges_.empty()) {
        reversed_edges_[end].push_back(Edge(start, cost));
        if (!directed_)
            reversed_edges_[start].push_back(Edge(end, cost));
    }
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

vector<vector<double>> Graph::get_coordinates() const {
    return coordinates_;
}

vector<pair<int, double>> Graph::get_neighbors(int node, bool reversed) {
    vector<pair<int, double>> nb;
    if (reversed && directed_) {
        if (reversed_edges_.empty()) {
            reversed_edges_.resize(num_vertices_, vector<Edge>(0));
            for (int i = 0; i < num_vertices_; i++) {
                for (const Edge &e: edges_[i]) {
                    reversed_edges_[e.node_id].push_back(Edge(i, e.cost));
                }
            }
        }

        for (const Edge &e : reversed_edges_[node])
            nb.push_back({e.node_id, e.cost});
    }
    else {
        for (const Edge &e : edges_[node])
            nb.push_back({e.node_id, e.cost});
    }
    return nb;
}

bool Graph::has_coordinates() const {
    return (num_vertices_ == 0) || (coordinates_.size() > 0);
}

double Graph::estimate_distance(int v1, int v2) const {
    assert(has_coordinates());

    const vector<double> &c1 = coordinates_[v1];
    const vector<double> &c2 = coordinates_[v2];

    double distance = 0;
    for (unsigned int i = 0; i < c1.size(); i++) {
        distance += std::pow(c1[i] - c2[i], 2);
    }
    distance = std::sqrt(distance);
    return distance;
}

Graph* Graph::create_reversed_graph() const {
    Graph* reversed_graph = new Graph(num_vertices_, directed_, coordinates_);
    for (int i = 0; i < num_vertices_; i++) {
        for (const Edge &e: edges_[i]) {
            reversed_graph->add_edge(e.node_id, i, e.cost);
        }
    }
    reversed_graph->set_pause_action_cost(get_pause_action_cost());
    return reversed_graph;
}

AbsGraph* Graph::reverse() const {
    return create_reversed_graph();
}

void Graph::reverse_inplace() {
    vector<int> new_starts, new_ends;
    vector<double> new_costs;
    for (int i = 0; i < num_vertices_; i++) {
        for (const Edge &e: edges_[i]) {
            new_starts.push_back(e.node_id);
            new_ends.push_back(i);
            new_costs.push_back(e.cost);
        }
        edges_[i].clear();
    }

    add_edges(new_starts, new_ends, new_costs);
}

bool Graph::adjacent(int v1, int v2) const {
    for (const Edge &e : edges_[v1]) {
        if (e.node_id == v2) {
            return true;
        }
    }
    return false;
}

void Graph::set_edge_collision(bool b) {
    if (b && !is_directed_graph()) {
        throw std::invalid_argument("An undirected graph does not support edge collisions");
    }
    AbsGraph::set_edge_collision(b);
}
