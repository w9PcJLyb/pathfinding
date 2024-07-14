#include "include/reservation_table.h"

ReservationTable::ReservationTable(int graph_size) : graph_size(graph_size) {
}

bool ReservationTable::reserved(int time, int node_id) const {
    if (dynamic_.count(st(time, node_id)))
        return true;

    if (semi_static_.count(node_id)) {
        auto time_agent = semi_static_.at(node_id);
        if (time_agent.first <= time)
            return true;
    }

    return false;
}

int ReservationTable::reserved_by(int time, int node_id) const {
    if (dynamic_.count(st(time, node_id)))
        return dynamic_.at(st(time, node_id));

    if (semi_static_.count(node_id)) {
        auto time_agent = semi_static_.at(node_id);
        if (time_agent.first <= time) {
            return time_agent.second;
        }
    }

    return -1;
}

void ReservationTable::add_path(int agent_id, int start_time, vector<int> &path, bool reserve_destination) {
    assert(agent_id >= 0);

    if (path.empty())
        return;

    for (size_t i = 0; i < path.size(); i++) {
        int time = start_time + i;
        if (time > max_time_) {
            max_time_ = time;
        }
        dynamic_[st(time, path[i])] = agent_id;
    }

    if (reserve_destination) {
        semi_static_[path.back()] = {start_time + path.size(), agent_id};
    }
}

void ReservationTable::remove_path(int start_time, vector<int> &path) {
    if (path.empty())
        return;

    for (size_t i = 0; i < path.size(); i++) {
        dynamic_.erase(st(start_time + i, path[i]));
    }

    dynamic_.erase(st(start_time + path.size(), path.back()));
}

int ReservationTable::last_time_reserved(int node_id) const {
    for (int time = max_time_; time >= 0; time--) {
        if (dynamic_.count(st(time, node_id))) {
            return time;
        }
    }
    return 0;
}
