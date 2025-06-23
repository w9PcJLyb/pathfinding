#pragma once

#include "pf.h"


class GBS : public AbsPathFinder {
    // Greedy Best-first Search

    typedef pair<double, int> key;  // first - potential, second - node_id
    typedef priority_queue<key, vector<key>, std::greater<key>> Queue;

    public:
        Env* env;
        GBS(Env* env);

        vector<int> find_path(int start, int end);

    private:
        vector<int> came_from_;
        vector<int> reconstruct_path(int start, int end);
        void clear();
};
