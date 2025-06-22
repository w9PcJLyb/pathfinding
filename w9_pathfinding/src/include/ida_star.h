#pragma once

#include "pf.h"


class IDAStar : public AbsPathFinder {
    // Iterative deepening A*

    public:
        Env* env;
        IDAStar(Env* env);

        vector<int> find_path(int start, int goal);
        vector<int> find_path(int start, int goal, double max_distance);

    private:
        int goal_;
        double search(vector<int> &path, double g, double bound);
};
 
