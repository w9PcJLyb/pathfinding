#pragma once

#include <vector>
#include <cassert>
#include <iostream>
#include <unordered_set>
#include <unordered_map>

using std::pair;
using std::vector;


class ReservationTable {
    public:
        const int graph_size;

        ReservationTable(int graph_size);

        ReservationTable& operator=(const ReservationTable& rt);

        void add_vertex_constraint(int time, int node_id);
        void add_edge_constraint(int time, int n1, int n2);
        void add_semi_static_constraint(int time, int node_id);
        bool is_reserved(int time, int node_id) const;
        std::unordered_set<int> get_reserved_edges(int time, int node_id) const;
        void add_path(int start_time, vector<int> &path, bool reserve_destination, bool add_edge_constraints);
        int last_time_reserved(int node_id) const;
        void print() const;

    private:
        int max_time_ = 0;

        // nodes in space-time reserved by other agents
        // {(time, node_id), ...}
        std::unordered_set<int> vertex_constraints_;

        // static obstacles from specific times, e.g. agents that have reached their destinations
        // node_id -> start time
        std::unordered_map<int, int> semi_static_constraints_;

        // edges in space-time reserved by other agents
        // (time, node_id) -> {node_id, ...}
        std::unordered_map<int, std::unordered_set<int>> edge_constraints_;

        int st(int time, int node_id) const {
            return time * graph_size + node_id;
        }
};
