#pragma once

#include "env.h"


class DFS : public AbsPathFinder {
    // Depth-first search
    public:
        Env* env;
        DFS(Env* env);

        vector<int> find_path(int start, int end);

    private:
        vector<int> came_from_;
        vector<int> reconstruct_path(int start, int end); 
        void clear();
};
