#include "include/core.h"


double AbsGraph::calculate_cost(Path& path) {
    if (path.size() <= 1)
        return 0;

    double pause_action_cost = get_pause_action_cost();

    double total_cost = 0;

    int node_id = path[0];

    for (size_t i = 1; i < path.size(); i++) {
        int next_node_id = path[i];

        double step_cost = -1;
        for (auto &[n, cost] : get_neighbors(node_id, false, true)) {
            if (n == next_node_id) {
                if (step_cost == -1 || cost < step_cost)
                    step_cost = cost;
            }
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

bool AbsGraph::is_valid_path(Path& path) {
    if (path.size() == 0)
        return true;

    int graph_size = size();
    int node_id = path[0];

    if (node_id < 0 || node_id >= graph_size)
        return false;

    for (size_t i = 1; i < path.size(); i++) {
        int next_node_id = path[i];

        if (next_node_id == node_id) {
            // pause action
            continue;
        }

        if (next_node_id < 0 || next_node_id >= graph_size || !adjacent(node_id, next_node_id))
            return false;

        node_id = next_node_id;
    }

    return true;
}

vector<int> AbsGraph::find_component_(vector<bool> &visited, int start) {
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

vector<vector<int>> AbsGraph::find_components() {
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

void dfs_with_order_(AbsGraph *graph, vector<bool> &visited, vector<int> &order, int start) {
    visited[start] = true;
    for (auto& [n, cost] : graph->get_neighbors(start, true)) {
        if (!visited[n])
            dfs_with_order_(graph, visited, order, n);
    }
    order.push_back(start);
}

vector<int> dfs_sort_(AbsGraph *graph) {
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

vector<vector<int>> AbsGraph::find_scc() {
    if (!is_directed_graph()) {
        throw std::runtime_error("find_scc only works for a directed graph");
    }

    // Kosaraju's algorithm

    vector<int> order = dfs_sort_(this);
    std::reverse(order.begin(), order.end());

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

bool AbsGraph::adjacent(int v1, int v2) {
    for (auto &[node_id, cost] : get_neighbors(v1)) {
        if (node_id == v2) {
            return true;
        }
    }
    return false;
}

std::string AbsGraph::node_to_string(int v) const {
    return std::to_string(v);
}

void AbsGraph::print_path(const Path& path) const {
    std::string s;
    for (int node_id : path) {
        if(!s.empty())
            s += ", ";
        s += node_to_string(node_id);
    }
    cout << "[" << s << "]" << endl;
}

void AbsGrid::update_weight(int node, double w) {
    if (w < 0 && w != -1)
        throw std::invalid_argument("Weight must be either non-negative or equal to -1");

    if (node < 0 || node >= int(size()))
        throw std::invalid_argument("Node " + std::to_string(node) + " is out of the grid");

    if (w >= 0 && (min_weight_ == -1 || min_weight_ > w))
        min_weight_ = w;

    weights_.at(node) = w;
}

void AbsGrid::set_weights(vector<double> &weights) {
    if (weights.size() != size())
        throw std::invalid_argument(
            "Weights must have exactly " + std::to_string(size()) + " elements"
        );

    double min_weight = -1;
    for (double w : weights) {
        if (w < 0 && w != -1)
            throw std::invalid_argument("Weight must be either non-negative or equal to -1");
        if (w >= 0 && (min_weight == -1 || min_weight > w))
            min_weight = w;
    }

    min_weight_ = min_weight;
    weights_ = weights;
}

vector<vector<int>> AbsGrid::find_components() {
    vector<vector<int>> components = AbsGraph::find_components();
    vector<vector<int>> components_without_walls;
    for (vector<int> &x : components) {
        if (x.size() == 1 && has_obstacle(x[0]))
            continue;
        components_without_walls.push_back(x);
    }
    return components_without_walls;
}

void ensure_path_length(Path& path, int length) {
    if ((int)path.size() >= length + 1)
        return;

    Path rest(length + 1 - path.size(), path.back());
    path.insert(path.end(), rest.begin(), rest.end());
}


bool has_collision(const vector<int>& positions, const vector<int>& next_positions, bool edge_collision) {
    std::unordered_map<int, int> node_to_agent;
    for (size_t i = 0; i < next_positions.size(); i++) {
        int node_id = next_positions[i];
        if (node_to_agent.count(node_id)) {
            // vertex conflict
            return true;
        }
        else
            node_to_agent[node_id] = i;
    }

    if (edge_collision) {
        for (size_t i = 0; i < positions.size(); i++) {
            int p = positions[i];
            int next_p = next_positions[i];
            if (p == next_p)
                continue;
            if (node_to_agent.count(p) && positions[node_to_agent[p]] == next_p) {
                // edge conflict
                return true;
            }
        }
    }

    return false;
}
