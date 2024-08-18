#pragma once

#include "core.h"


class IDAStar : public AbsPathFinder {
    // Iterative deepening A*

    public:
        AbsGraph* graph;
        IDAStar(AbsGraph *graph);

        vector<int> find_path(int start, int goal);
        vector<int> find_path(int start, int goal, double max_distance);

    private:
        int goal_;
        double search(vector<int> &path, double g, double bound);
};
 
