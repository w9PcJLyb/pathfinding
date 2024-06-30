#pragma once

#include "grid.h"


class BiAStar : public AbsPathFinder {

    typedef pair<double, int> key;  // first - distance + potential, second - node_id
    typedef priority_queue<key, vector<key>, std::greater<key>> Queue;

    struct Node {
        int parent;
        double distance;  // from start to this node
        double f;  // distance + heuristic
        bool visited;

        Node() : parent(-1), distance(-1), f(0), visited(false) {};

        void clear() {
            parent = -1;
            distance = -1;
            f = 0;
            visited = false;
        }
    };

    public:
        AbsGraph* graph;
        BiAStar(AbsGraph* graph);
        ~BiAStar();

        vector<int> find_path(int start, int end);

    private:
        int start_node_, end_node_;
        AbsGraph* reversed_graph_;
        vector<vector<Node>> nodes_;
        vector<int> workset_;
        vector<int> reconstruct_path(int start, int end); 
        void clear();
        bool step(int side, Queue &queue, AbsGraph* g);
        double potential(int node_id, int side);
};
