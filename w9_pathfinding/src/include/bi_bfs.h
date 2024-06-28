#pragma once

#include "graph.h"


class BiBFS : public AbsPathFinder {
    // Bidirectional Breadth-first search

    struct Node {
        int parent;
        int distance;  // from start to this node

        Node() : parent(-1), distance(-1) {};

        void clear() {
            parent = -1;
            distance = -1;
        }
    };

    public:
        AbsGraph* graph;
        BiBFS(AbsGraph *graph);
        ~BiBFS();

        vector<int> find_path(int start, int end);

    private:
        vector<Node> forward_nodes_, backward_nodes_;
        vector<int> workset_;
        vector<bool> closedset_;
        AbsGraph* reversed_graph_;
        vector<int> reconstruct_path(int start, int end); 
        bool step(std::queue<int> &queue, vector<Node> &nodes, AbsGraph* g);
        void clear();
};
