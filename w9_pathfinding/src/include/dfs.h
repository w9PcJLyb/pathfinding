#pragma once

#include "graph.h"


class DFS : public AbsPathFinder {
    // Depth-first search
    public:
        AbsGraph* graph;
        DFS(AbsGraph *graph);

        vector<int> find_path(int start, int end);

    private:
        vector<int> came_from_;
        vector<int> reconstruct_path(int start, int end); 
        void clear();
};
