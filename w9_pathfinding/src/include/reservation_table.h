#pragma once

#include <vector>
#include <cassert>
#include <iostream>
#include <unordered_map>

using std::pair;
using std::vector;


class ReservationTable {
    public:
        const int graph_size;

        ReservationTable(int graph_size);

        ReservationTable& operator=(const ReservationTable& rt);

        void add_vertex_constraint(int agent_id, int time, int node_id);
        void add_edge_constraint(int agent_id, int time, int n1, int n2);
        bool is_reserved(int time, int node_id) const;
        int get_reserved_edge(int time, int node_id) const;
        int reserved_by(int time, int node_id) const;
        void add_path(int agent_id, int start_time, vector<int> &path, bool reserve_destination, bool add_edge_constraints);
        void remove_path(int start_time, vector<int> &path);
        int last_time_reserved(int node_id) const;

    private:
        int max_time_ = 0;

        // nodes in space-time reserved by other agents
        // (time, node_id) -> agent_id
        std::unordered_map<int, int> vertex_constraints_;

        // static obstacles from specific times, e.g. agents that have reached their destinations
        // node_id -> (start time, agent_id)
        std::unordered_map<int, pair<int, int>> semi_static_constraints_;

        // edges in space-time reserved by other agents
        // (time, node_id) -> (node_id, agent_id)
        std::unordered_map<int, pair<int, int>> edge_constraints_;

        int st(int time, int node_id) const {
            return time * graph_size + node_id;
        }
};
