#pragma once

#include "graph.h"


class BFS : public AbsPathFinder {
    // Breadth-first search
    public:
        AbsGraph* graph;
        BFS(AbsGraph *graph);

        vector<int> find_path(int start, int end);

    private:
        vector<int> came_from_;
        vector<int> reconstruct_path(int start, int end); 
        void clear();
};
