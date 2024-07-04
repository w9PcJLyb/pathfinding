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
        virtual vector<pair<int, double>> get_neighbours(int node) const = 0;
        virtual AbsGraph* reverse() const = 0;
        virtual void reverse_inplace() = 0;
        virtual double estimate_distance(int v1, int v2) const = 0;
        virtual bool is_directed_graph() const = 0;
        double calculate_cost(vector<int> &path) const;
        virtual vector<vector<int>> find_components() const;
        virtual vector<vector<int>> find_scc() const;

    private:
        vector<int> find_component_(vector<bool> &visited, int start) const;
};


class AbsPathFinder {
    public:
        AbsPathFinder() {};
        virtual ~AbsPathFinder() {};
        virtual vector<int> find_path(int start, int end) = 0;
};
