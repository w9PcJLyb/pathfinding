#pragma once

#include "core.h"


class Graph : public AbsGraph {

    struct Edge {
        int node_id;
        double cost;

        Edge() : node_id(0), cost(1) {};
        Edge(int node_id, double cost) : node_id(node_id), cost(cost) {}; 
    };

    public:
        Graph(int num_vertices);
        size_t size() const;
        size_t num_edges() const;
        void add_edge(int start, int end, double cost);
        void add_edges(vector<int> starts, vector<int> ends, vector<double> costs);
        vector<pair<int, double>> get_neighbours(int node) const;
        vector<vector<double>> get_edges() const;
        double estimate_distance(int v1, int v2) const;
        AbsGraph* reverse() const;
        Graph* create_reversed_graph() const;
        void reverse_inplace();

    private:
        int num_vertices_; 
        vector<vector<Edge>> edges_;
};