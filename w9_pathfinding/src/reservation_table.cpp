#include "include/reservation_table.h"

ReservationTable::ReservationTable(int graph_size) : graph_size(graph_size) {
}

ReservationTable& ReservationTable::operator=(const ReservationTable& rt) {
    if (graph_size != rt.graph_size) {
        throw std::invalid_argument("graph_size should be the same");
    }
    vertex_constraints_ = rt.vertex_constraints_;
    semi_static_constraints_ = rt.semi_static_constraints_;
    max_time_ = rt.max_time_;
    return *this;
}

void ReservationTable::add_vertex_constraint(int agent_id, int time, int node_id) {
    vertex_constraints_[st(time, node_id)] = agent_id;
}

void ReservationTable::add_edge_constraint(int agent_id, int time, int n1, int n2) {
    edge_constraints_[st(time, n1)] = {n2, agent_id};
}

bool ReservationTable::is_reserved(int time, int node_id) const {
    if (vertex_constraints_.count(st(time, node_id)))
        return true;

    if (semi_static_constraints_.count(node_id)) {
        auto time_agent = semi_static_constraints_.at(node_id);
        if (time_agent.first <= time)
            return true;
    }

    return false;
}

int ReservationTable::get_reserved_edge(int time, int node_id) const {
    int st_ = st(time, node_id);
    if (edge_constraints_.count(st_)) {
        return edge_constraints_.at(st_).first;
    }
    return -1;
}

int ReservationTable::reserved_by(int time, int node_id) const {
    if (vertex_constraints_.count(st(time, node_id)))
        return vertex_constraints_.at(st(time, node_id));

    if (semi_static_constraints_.count(node_id)) {
        auto time_agent = semi_static_constraints_.at(node_id);
        if (time_agent.first <= time) {
            return time_agent.second;
        }
    }

    return -1;
}

void ReservationTable::add_path(
    int agent_id,
    int start_time,
    vector<int> &path,
    bool reserve_destination,
    bool add_edge_constraints
) {
    assert(agent_id >= 0);

    if (path.empty())
        return;

    for (size_t i = 0; i < path.size(); i++) {
        int time = start_time + i;
        if (time > max_time_) {
            max_time_ = time;
        }
        if (add_edge_constraints && i > 0) {
            edge_constraints_[st(time - 1, path[i])] = {path[i - 1], agent_id};
        }
        vertex_constraints_[st(time, path[i])] = agent_id;
    }

    if (reserve_destination) {
        semi_static_constraints_[path.back()] = {start_time + path.size(), agent_id};
    }
}

void ReservationTable::remove_path(int start_time, vector<int> &path) {
    if (path.empty())
        return;

    for (size_t i = 0; i < path.size(); i++) {
        vertex_constraints_.erase(st(start_time + i, path[i]));
        if (i > 0) {
            int st_ = st(start_time + i - 1, path[i]);
            if (edge_constraints_.count(st_)) {
                if (edge_constraints_[st_].first == path[i - 1]) {
                    edge_constraints_.erase(st_);
                }
            }
        }
    }

    vertex_constraints_.erase(st(start_time + path.size(), path.back()));
}

int ReservationTable::last_time_reserved(int node_id) const {
    for (int time = max_time_; time >= 0; time--) {
        if (vertex_constraints_.count(st(time, node_id))) {
            return time;
        }
    }
    return 0;
}
