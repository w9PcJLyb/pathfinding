#include "include/core.h"


double AbsGraph::calculate_cost(vector<int> &path) const {
    if (path.size() <= 1)
        return 0;

    bool pause_action_allowed = is_pause_action_allowed();
    double pause_action_cost = get_pause_action_cost();

    double total_cost = 0;

    int node_id = path[0];
    if (has_dynamic_obstacle(0, node_id))
        return -1;

    for (size_t i = 1; i < path.size(); i++) {
        int next_node_id = path[i];
        if (has_dynamic_obstacle(i, next_node_id))
           return -1;

        double step_cost = -1;

        for (auto &[n, cost] : get_neighbours(node_id)) {
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
        for (auto& [n, cost] : get_neighbours(x)) {
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
    for (auto& [n, cost] : graph->get_neighbours(start)) {
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

void AbsGraph::set_dynamic_obstacles(vector<unordered_set<int>> dynamic_obstacles) {
    dynamic_obstacles_ = dynamic_obstacles;
}

vector<unordered_set<int>> AbsGraph::get_dynamic_obstacles() const {
    return dynamic_obstacles_;
}

void AbsGraph::add_dynamic_obstacles(vector<int> path) {
    size_t s = dynamic_obstacles_.size();
    for (size_t t = 0; t < path.size(); t++) {
        if (t < s)
            dynamic_obstacles_[t].insert(path[t]);
        else
            dynamic_obstacles_.push_back({path[t]});
    }
}

void AbsGraph::set_semi_dynamic_obstacles(vector<unordered_set<int>> semi_dynamic_obstacles) {
    semi_dynamic_obstacles_ = semi_dynamic_obstacles;
}

void AbsGraph::add_semi_dynamic_obstacles(int time, int node_id) {
    int s = semi_dynamic_obstacles_.size();

    if (s == 0) {
        semi_dynamic_obstacles_.resize(time + 1);
        semi_dynamic_obstacles_[time].insert(node_id);
    }
    else if (time < s) {
        for (int t = time; t < s; t++) {
            semi_dynamic_obstacles_[t].insert(node_id);
        }
    }
    else {
        auto obstacles = semi_dynamic_obstacles_.back();
        for (int t = s; t <= time; t++) {
            semi_dynamic_obstacles_.push_back(obstacles);
        }
        semi_dynamic_obstacles_[time].insert(node_id);
    }
}

vector<unordered_set<int>> AbsGraph::get_semi_dynamic_obstacles() const {
    return semi_dynamic_obstacles_;
}

bool AbsGraph::has_dynamic_obstacle(int time, int node_id) const {
    if (time < (int)dynamic_obstacles_.size()) {
        if (dynamic_obstacles_[time].count(node_id))
            return true;
    }
    if (time < (int)semi_dynamic_obstacles_.size()) {
        if (semi_dynamic_obstacles_[time].count(node_id))
            return true;
    }
    else if (!semi_dynamic_obstacles_.empty()) {
        if (semi_dynamic_obstacles_.back().count(node_id))
            return true;
    }
    return false;
}
