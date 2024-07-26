#include "include/core.h"


double AbsGraph::calculate_cost(vector<int> &path) const {
    if (path.size() <= 1)
        return 0;

    bool pause_action_allowed = is_pause_action_allowed();
    double pause_action_cost = get_pause_action_cost();

    double total_cost = 0;

    int node_id = path[0];

    for (size_t i = 1; i < path.size(); i++) {
        int next_node_id = path[i];

        double step_cost = -1;
        for (auto &[n, cost] : get_neighbors(node_id)) {
            if (n == next_node_id) {
                if (step_cost == -1 || cost < step_cost) {
                    step_cost = cost;
                }
            }
        }

        if (pause_action_allowed && next_node_id == node_id) {
            if (step_cost == -1)
                step_cost = get_pause_action_cost();
            else if (pause_action_cost < step_cost)
                step_cost = pause_action_cost;
        }

        if (step_cost == -1) {
            // not connected path
            return -1;
        }

        total_cost += step_cost;
        node_id = next_node_id;
    }

    return total_cost;
}

vector<int> AbsGraph::find_component_(vector<bool> &visited, int start) const {
    visited[start] = true;
    vector<int> component = {start};

    vector<int> stack = {start};
    while (!stack.empty()) {
        int x = stack.back();
        stack.pop_back();
        for (auto& [n, cost] : get_neighbors(x)) {
            if (!visited[n]) {
                visited[n] = true;
                component.push_back(n);
                stack.push_back(n);
            }
        }
    }

    return component;
}

vector<vector<int>> AbsGraph::find_components() const {
    if (is_directed_graph())
        throw std::runtime_error("find_components only works for an undirected graph");

    int graph_size = size();
    vector<bool> visited(graph_size, false);
    vector<vector<int>> components;

    int offset = 0;
    while (true) {

        int start = -1;
        for (int i = offset; i < graph_size; i++) {
            if (!visited[i]) {
                start = i;
                offset = i + 1;
                break;
            }
        }

        if (start == -1)
            break;

        components.push_back(find_component_(visited, start));
    }

    return components;
}

void dfs_with_order_(const AbsGraph *graph, vector<bool> &visited, vector<int> &order, int start) {
    visited[start] = true;
    for (auto& [n, cost] : graph->get_neighbors(start)) {
        if (!visited[n])
            dfs_with_order_(graph, visited, order, n);
    }
    order.push_back(start);
}

vector<int> dfs_sort_(const AbsGraph *graph) {
    int graph_size = graph->size();

    vector<bool> visited(graph_size, false);
    vector<int> order;

    int offset = 0;
    while (true) {

        int start = -1;
        for (int i = offset; i < graph_size; i++) {
            if (!visited[i]) {
                start = i;
                offset = i + 1;
                break;
            }
        }

        if (start == -1)
            break;

        dfs_with_order_(graph, visited, order, start);
    }

    return order;
}

vector<vector<int>> AbsGraph::find_scc() const {
    if (!is_directed_graph()) {
        throw std::runtime_error("find_scc only works for a directed graph");
    }

    // Kosaraju's algorithm

    AbsGraph* reversed_graph = reverse();
    vector<int> order = dfs_sort_(reversed_graph);
    std::reverse(order.begin(), order.end());
    delete reversed_graph;

    int graph_size = size();
    vector<bool> visited(graph_size, false);
    vector<vector<int>> scc;

    int offset = 0;
    while (true) {

        int start = -1;
        for (int i = offset; i < graph_size; i++) {
            if (!visited[order[i]]) {
                start = order[i];
                offset = i + 1;
                break;
            }
        }

        if (start == -1)
            break;

        scc.push_back(find_component_(visited, start));
    }

    return scc;
}

bool AbsGraph::adjacent(int v1, int v2) const {
    for (auto &[node_id, cost] : get_neighbors(v1)) {
        if (node_id == v2) {
            return true;
        }
    }
    return false;
}

void AbsGraph::set_pause_action_cost(double cost) {
    if (cost < 0 && cost != -1)
        throw std::invalid_argument("Pause action cost must be either non-negative or equal to -1");
    pause_action_cost_ = cost;
}

double AbsGraph::get_pause_action_cost() const {
    return pause_action_cost_;
}

bool AbsGraph::is_pause_action_allowed() const {
    return pause_action_cost_ >= 0;
}

void AbsGraph::set_edge_collision(bool b) {
    edge_collision_ = b;
}

bool AbsGraph::edge_collision() const {
    return edge_collision_;
}

size_t AbsGrid::size() const {
    return weights_.size();
}

bool AbsGrid::has_coordinates() const {
    return true;
}

bool AbsGrid::is_directed_graph() const {
    return false;
}

void AbsGrid::set_weights(vector<double> &weights) {
    if (weights.size() != size())
        throw std::invalid_argument("Wrong shape");

    if (!weights.empty()) {
        min_weight_ = -1;
        for (double w : weights) {
            if (w < 0 && w != -1) {
                throw std::invalid_argument("Weight must be either non-negative or equal to -1");
            }
            if (w != -1) {
                if (min_weight_ == -1 || min_weight_ > w)
                    min_weight_ = w;
            }
        }
    }

    weights_ = weights;
}

vector<double> AbsGrid::get_weights() const {
    return weights_;
}

bool AbsGrid::has_obstacle(int node) const {
    return weights_.at(node) == -1;
}

void AbsGrid::add_obstacle(int node) {
    weights_.at(node) = -1;
}

void AbsGrid::remove_obstacle(int node) {
    weights_.at(node) = 1;
}

void AbsGrid::clear_weights() {
    std::fill(weights_.begin(), weights_.end(), 1);
}

vector<vector<int>> AbsGrid::find_components() const {
    vector<vector<int>> components = AbsGraph::find_components();
    vector<vector<int>> components_without_walls;
    for (vector<int> &x : components) {
        if (x.size() == 1 && has_obstacle(x[0]))
            continue;
        components_without_walls.push_back(x);
    }
    return components_without_walls;
}
