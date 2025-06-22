#pragma once

#include <cmath>
#include <queue>
#include <array>
#include <vector>
#include <random>
#include <numeric>
#include <ostream>
#include <cassert>
#include <iostream>
#include <algorithm>
#include <stdexcept>
#include <unordered_map>

using std::cout;
using std::endl;
using std::pair;
using std::vector;
using std::priority_queue;

typedef vector<int> Path;


class Env {
    public:
        Env() {};
        virtual ~Env() {};

        // Returns the number of nodes (vertices) in the environment.
        virtual size_t size() const = 0;

        // Returns the neighbors of a given node.
        // Each neighbor is represented as a pair (neighbor_id, edge_cost).
        virtual vector<pair<int, double>> get_neighbors(
            int node, bool reversed=false, bool include_self=false
        ) = 0;

        // Indicates whether the implementation provides a heuristic function (estimate_distance).
        virtual bool has_heuristic() const = 0;

        // Estimates a lower bound on the cost between two vertices.
        // This function is used as a heuristic in A*-like search algorithms.
        // Only valid if `has_heuristic()` returns true.
        virtual double estimate_distance(int v1, int v2) const = 0;

        // Computes the total cost of a path
        virtual double calculate_cost(Path& path);

        // Checks whether all consecutive node pairs in the path are connected by an edge.
        bool is_valid_path(Path& path);

        // returns true if there is a path of length 1 from vertex v1 to vertex v2
        virtual bool adjacent(int v1, int v2);

        virtual std::string node_to_string(int v) const;
        void print_path(const Path& path) const;

        double min_weight() const {
            return min_weight_;
        }

    protected:
        // the minimum value in weights, used in the heuristic function (estimate_distance)
        double min_weight_ = 1.0;

    // For multi agent path finding
    private:
        // if edge_collision_ is true, two agents can not pass on the same edge
        // at the same time in two different directions
        bool edge_collision_ = false;

    public:
        virtual void set_edge_collision(bool b) {
            edge_collision_ = b;
        }

        bool edge_collision() const {
            return edge_collision_;
        }
};


class GridEnv : public Env {

    public:
        size_t size() const {
            return weights_.size();
        }

        bool has_heuristic() const {
            return true;
        }

        double get_weight(int node) const {
            return weights_.at(node);
        }

        vector<double> get_weights() const {
            return weights_;
        }

        bool has_obstacle(int node) const {
            return get_weight(node) == -1;
        }

        void add_obstacle(int node) {
            update_weight(node, -1);
        }

        void remove_obstacle(int node) {
            update_weight(node, 1);
        }

        void clear_weights() {
            std::fill(weights_.begin(), weights_.end(), 1);
        }

        void update_weight(int node, double w);
        void set_weights(vector<double> &weights);

        void set_pause_weight(double w);
        void set_pause_weights(vector<double> &weights);
        double get_pause_weight(int node) const;
        vector<double> get_pause_weights() const;

        void clear_pause_weights() {
            pause_weights_.clear();
            pause_weight_ = 1;
        }

    protected:
        // weights_[i] - cost to move to node i.
        // If weights_[i] == -1, the node i is considered an impassable obstacle and is unreachable.
        vector<double> weights_;

        // Default pause cost for all nodes.
        // Used when pause_weights_ is empty.
        double pause_weight_ = 1;

        // pause_weights_[i] - cost to pause at node i.
        // If pause_weights_[i] == -1, pausing is not allowed at node i.
        vector<double> pause_weights_;
};


class AbsPathFinder {
    public:
        AbsPathFinder() {};
        virtual ~AbsPathFinder() {};
        virtual Path find_path(int start, int end) = 0;
};


class AbsMAPF {
    public:
        AbsMAPF() {};
        virtual ~AbsMAPF() {};
        virtual vector<Path> mapf(vector<int> starts, vector<int> goals) = 0;
};


class timeout_exception : public std::runtime_error {
    public:
        using std::runtime_error::runtime_error;
};


void ensure_path_length(Path& path, int length);


bool has_collision(const vector<int>& positions, const vector<int>& next_positions, bool edge_collision);


template <typename T>
bool generate_next_combination(vector<vector<T>>& data, vector<T>& combination, vector<int>& indices) {
    if (combination.empty()) {
        if (data.empty())
            return false;

        for (auto& d : data) {
            if (d.empty())
                return false;
            combination.push_back(d[0]);
        }
        indices.resize(data.size(), 0);
        return true;
    }

    for (int i = data.size() - 1; i >= 0; --i) {
        if (indices[i] < (int)data[i].size() - 1) {
            indices[i]++;
            combination[i] = data[i][indices[i]];
            return true;
        } else {
            indices[i] = 0;
            combination[i] = data[i][0];
        }
    }
    return false;
}

struct SpaceHash {
    size_t operator()(const std::vector<int>& position) const {
        size_t hash_value = 0;
        for (const auto& x : position) {
            hash_value ^= std::hash<int>{}(x) + 0x9e3779b9 + (hash_value << 6) + (hash_value >> 2);
        }
        return hash_value;
    }
};


struct SpaceTimeHash {
    size_t operator()(const std::pair<int, std::vector<int>>& p) const {
        size_t hash_value = std::hash<int>{}(p.first);
        size_t vector_hash = SpaceHash{}(p.second);
        hash_value ^= vector_hash + 0x9e3779b9 + (hash_value << 6) + (hash_value >> 2);
        return hash_value;
    }
};
