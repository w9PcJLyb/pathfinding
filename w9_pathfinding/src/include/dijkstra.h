#pragma once

#include "env.h"


class Dijkstra : public AbsPathFinder {

    typedef pair<double, int> key; // first - distance, second - node_id
    typedef priority_queue<key, vector<key>, std::greater<key>> Queue; 

    struct Node {
        int parent;
        double distance;  // from start to this node

        Node() : parent(-1), distance(-1) {};

        void clear() {
            parent = -1;
            distance = -1;
        }
    };

    public:
        Env* env;
        Dijkstra(Env* env);

        vector<int> find_path(int start, int end);

    private:
        vector<Node> nodes_;
        vector<int> reconstruct_path(int start, int end); 
        void clear();
};
 
 
