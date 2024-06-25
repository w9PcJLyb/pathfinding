#pragma once

#include <cmath>
#include <queue>
#include <vector>
#include <ostream>
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
        virtual size_t size() const {
            return 0;
        }
        virtual vector<pair<int, double>> get_neighbours(int node) const {
            return {};
        }
};


class AbsPathFinder {
    public:
        AbsPathFinder() {};
        virtual ~AbsPathFinder() {};
        virtual vector<int> find_path(int start, int end) {
            return {};
        }
};
