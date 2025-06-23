#include "include/env.h"


double Env::calculate_cost(Path& path) {
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

bool Env::is_valid_path(Path& path) {
    if (path.size() == 0)
        return true;

    int env_size = size();
    int node_id = path[0];

    if (node_id < 0 || node_id >= env_size)
        return false;

    for (size_t i = 1; i < path.size(); i++) {
        int next_node_id = path[i];

        if (next_node_id < 0 || next_node_id >= env_size || !adjacent(node_id, next_node_id))
            return false;

        node_id = next_node_id;
    }

    return true;
}

bool Env::adjacent(int v1, int v2) {
    for (auto &[node_id, cost] : get_neighbors(v1, false, true)) {
        if (node_id == v2) {
            return true;
        }
    }
    return false;
}

std::string Env::node_to_string(int v) const {
    return std::to_string(v);
}

void Env::print_path(const Path& path) const {
    std::string s;
    for (int node_id : path) {
        if(!s.empty())
            s += ", ";
        s += node_to_string(node_id);
    }
    cout << "[" << s << "]" << endl;
}

void GridEnv::update_weight(int node, double w) {
    if (w < 0 && w != -1)
        throw std::invalid_argument("Weight must be either non-negative or equal to -1");

    if (node < 0 || node >= int(size()))
        throw std::invalid_argument("Node " + std::to_string(node) + " is out of the grid");

    if (w >= 0 && (min_weight_ == -1 || min_weight_ > w))
        min_weight_ = w;

    weights_.at(node) = w;
}

void GridEnv::set_weights(vector<double> &weights) {
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

void GridEnv::set_pause_weight(double w) {
    if (w < 0)
        throw std::invalid_argument("Pause weight must be non-negative");
    pause_weight_ = w;
    pause_weights_.clear();
}

void GridEnv::set_pause_weights(vector<double> &weights) {
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

double GridEnv::get_pause_weight(int node) const {
    if (!pause_weights_.empty())
        return pause_weights_.at(node);
    return pause_weight_;
}

vector<double> GridEnv::get_pause_weights() const {
    if (!pause_weights_.empty())
        return pause_weights_;

    vector<double> weights(size(), pause_weight_);
    return weights;
}
