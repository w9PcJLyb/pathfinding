#pragma once

#include <cmath>
#include <queue>
#include <vector>
#include <ostream>
#include <cassert>
#include <iostream>
#include <algorithm>
#include <stdexcept>

using std::cout;
using std::endl;
using std::pair;
using std::vector;
using std::priority_queue;


class AbsGraph {
    public:
        AbsGraph() {};
        virtual ~AbsGraph() {};
        virtual size_t size() const = 0;
        virtual vector<pair<int, double>> get_neighbors(int node) const = 0;
        virtual AbsGraph* reverse() const = 0;

        // returns a lower bound of the distance between two vertices
        // used by A* algorithm
        virtual double estimate_distance(int v1, int v2) const = 0;

        virtual bool is_directed_graph() const = 0;

        // returns a cost of moving along the path
        // returns -1 if the path is not possible
        double calculate_cost(vector<int> &path) const;

        // returns connected components in an undirected graph
        virtual vector<vector<int>> find_components() const;

        // returns Strongly Connected Components (SCC) in a directed graph
        virtual vector<vector<int>> find_scc() const;

        // returns true if there is a path of length 1 from vertex v1 to vertex v2
        virtual bool adjacent(int v1, int v2) const;

    private:
        vector<int> find_component_(vector<bool> &visited, int start) const;

    // For multi agent path finding
    private:
        // the cost of the pause action
        // if pause_action_cost_ == -1, the pause action is not allowed,
        // but in any case, an agent can do pause actions at its destination.
        double pause_action_cost_ = 1;

        // if edge_collision_ is true, two agents can not pass on the same edge
        // at the same time in two different directions
        bool edge_collision_ = false;

    public:
        void set_pause_action_cost(double cost);
        double get_pause_action_cost() const;
        bool is_pause_action_allowed() const;
        virtual void set_edge_collision(bool b);
        bool edge_collision() const;
};


class AbsGrid : public AbsGraph {

    public:
        size_t size() const;
        bool is_directed_graph() const;
        void set_weights(vector<double> &weights);
        vector<double> get_weights() const;
        bool has_obstacle(int node) const;
        void add_obstacle(int node);
        void remove_obstacle(int node);
        void clear_weights();
        vector<vector<int>> find_components() const override;

    protected:
        // if weight == -1 - there is an impassable obstacle, the node is unreachable
        // if weight >= 0 - weight is the cost of entering this node
        vector<double> weights_;

        // the minimum value in weights, used in the heuristic function (estimate_distance)
        double min_weight_;

        // is a reversed graph, used in bidirectional algorithms
        bool reversed_ = false;
};


class AbsPathFinder {
    public:
        AbsPathFinder() {};
        virtual ~AbsPathFinder() {};
        virtual vector<int> find_path(int start, int end) = 0;
};


class AbsMAPF : public AbsPathFinder {
    public:
        AbsMAPF() {};
        virtual ~AbsMAPF() {};
        virtual vector<int> find_path(int start, int end) = 0;
        virtual vector<vector<int>> mapf(vector<int> starts, vector<int> goals) = 0;
};
