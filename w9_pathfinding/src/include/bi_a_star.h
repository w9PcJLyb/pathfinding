#pragma once

#include "grid.h"


class BiAStar : public AbsPathFinder {

    typedef pair<double, int> key;  // first - distance + potential, second - node_id
    typedef priority_queue<key, vector<key>, std::greater<key>> Queue;

    struct Node {
        int parent;
        double distance;  // from start to this node
        double f;  // distance + heuristic

        Node() : parent(-1), distance(-1), f(0) {};

        void clear() {
            parent = -1;
            distance = -1;
            f = 0;
        }
    };

    public:
        AbsGraph* graph;
        BiAStar(AbsGraph* graph);
        ~BiAStar();

        vector<int> find_path(int start, int end);

    private:
        double epsilon = 0.000001;
        int start_node, end_node;
        AbsGraph* reversed_graph_;
        vector<Node> forward_nodes_, backward_nodes_;
        vector<int> workset_;
        vector<bool> closedset_;
        vector<int> reconstruct_path(int start, int end); 
        void clear();
        bool step(Queue &queue, vector<Node> &nodes, AbsGraph* g, bool is_backward);
        double potential(int node_id, bool is_backward);
};
