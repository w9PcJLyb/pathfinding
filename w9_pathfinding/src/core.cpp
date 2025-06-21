#include "include/core.h"


double AbsGraph::calculate_cost(Path& path) {
    if (path.size() <= 1)
        return 0;

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

        if (next_node_id < 0 || next_node_id >= graph_size || !adjacent(node_id, next_node_id))
            return false;

        node_id = next_node_id;
    }

    return true;
}

bool AbsGraph::adjacent(int v1, int v2) {
    for (auto &[node_id, cost] : get_neighbors(v1, false, true)) {
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

void AbsGrid::set_pause_weight(double w) {
    if (w < 0)
        throw std::invalid_argument("Pause weight must be non-negative");
    pause_weight_ = w;
    pause_weights_.clear();
}

void AbsGrid::set_pause_weights(vector<double> &weights) {
    if (weights.size() != size())
        throw std::invalid_argument(
            "Weights must have exactly " + std::to_string(size()) + " elements"
        );

    for (double w : weights) {
        if (w < 0 && w != -1)
            throw std::invalid_argument("Weight must be either non-negative or equal to -1");
    }

    pause_weights_ = weights;
}

double AbsGrid::get_pause_weight(int node) const {
    if (!pause_weights_.empty())
        return pause_weights_.at(node);
    return pause_weight_;
}

vector<double> AbsGrid::get_pause_weights() const {
    if (!pause_weights_.empty())
        return pause_weights_;

    vector<double> weights(size(), pause_weight_);
    return weights;
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
