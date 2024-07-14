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

        bool reserved(int time, int node_id) const;
        int reserved_by(int time, int node_id) const;
        void add_path(int agent_id, int start_time, vector<int> &path, bool reserve_destination);
        void remove_path(int start_time, vector<int> &path);
        int last_time_reserved(int node_id) const;

    private:
        int max_time_ = 0;

        // nodes in space-time reserved by other agents
        // (time, node_id) -> agent_id
        std::unordered_map<int, int> dynamic_;

        // static obstacles from specific times, e.g. agents that have reached their destinations
        // node_id -> (start time, agent_id)
        std::unordered_map<int, pair<int, int>> semi_static_;

        int st(int time, int node_id) const {
            return time * graph_size + node_id;
        }
}; 
