#pragma once

#include "env.h"


class AStar : public AbsPathFinder {

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
        Env* env;
        AStar(Env* env);

        vector<int> find_path(int start, int end);

    private:
        vector<Node> nodes_;
        vector<Node*> workset_;
        vector<int> reconstruct_path(int start, int end); 
        void clear();
};
