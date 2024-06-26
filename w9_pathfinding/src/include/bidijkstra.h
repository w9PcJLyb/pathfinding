 #pragma once

#include "graph.h"


class BiDijkstra : public AbsPathFinder {
    // Bidirectional Dijkstra

    typedef pair<double, int> key;  // first - distance, second - node_id
    typedef priority_queue<key, vector<key>, std::greater<key>> Queue; 

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
        BiDijkstra(AbsGraph *graph);
        ~BiDijkstra();

        vector<int> find_path(int start, int end);

    private:
        AbsGraph* reversed_graph_;
        vector<vector<Node>> nodes_;
        vector<int> workset_;
        vector<int> reconstruct_path(int start, int end); 
        void clear();
        void visit_node(int side, int node_id, vector<Queue> &openset, double cost, int parent);
        int extract_min(int side, vector<Queue> &openset); 
        void process_node(int side, int node_id, vector<Queue> &openset, AbsGraph *g);
};
 
 
