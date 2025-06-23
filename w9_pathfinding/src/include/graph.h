#pragma once

#include "env.h"


class Graph : public Env {

    struct Edge {
        int node_id;
        double cost;

        Edge() : node_id(0), cost(1) {};
        Edge(int node_id, double cost) : node_id(node_id), cost(cost) {};
    };

    public:
        Graph(int num_vertices, bool directed);
        Graph(int num_vertices, bool directed, vector<vector<double>> coordinates);
        bool is_directed_graph() const;
        size_t size() const;
        size_t num_edges() const;
        void add_edge(int start, int end, double cost);
        void add_edges(vector<int> starts, vector<int> ends, vector<double> costs);
        vector<pair<int, double>> get_neighbors(int node, bool reversed=false, bool include_self=false);
        vector<vector<double>> get_edges() const;
        vector<vector<double>> get_coordinates() const;
        void set_coordinates(vector<vector<double>> &coordinates);
        bool has_heuristic() const;
        double estimate_distance(int v1, int v2) const;
        Env* reverse() const;
        Graph* create_reversed_graph() const;
        void reverse_inplace();
        bool adjacent(int v1, int v2) const;
        void set_edge_collision(bool b);

    private:
        int num_vertices_;
        const bool directed_;
        vector<vector<Edge>> edges_, reversed_edges_;
        vector<vector<double>> coordinates_;

        void update_reversed_edges();
};
