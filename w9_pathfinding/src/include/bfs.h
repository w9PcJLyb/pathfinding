#pragma once

#include "env.h"


class BFS : public AbsPathFinder {
    // Breadth-first search
    public:
        Env* env;
        BFS(Env *env);

        vector<int> find_path(int start, int end);

    private:
        vector<int> came_from_;
        vector<int> reconstruct_path(int start, int end); 
        void clear();
};
