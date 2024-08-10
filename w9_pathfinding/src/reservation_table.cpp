#include "include/reservation_table.h"

using std::cout;
using std::endl;

ReservationTable::ReservationTable(int graph_size) : graph_size(graph_size) {
}

ReservationTable& ReservationTable::operator=(const ReservationTable& rt) {
    if (graph_size != rt.graph_size) {
        throw std::invalid_argument("graph_size should be the same");
    }
    vertex_constraints_ = rt.vertex_constraints_;
    edge_constraints_ = rt.edge_constraints_;
    semi_static_constraints_ = rt.semi_static_constraints_;
    max_time_ = rt.max_time_;
    return *this;
}

void ReservationTable::add_vertex_constraint(int time, int node_id) {
    if (time > max_time_)
        max_time_ = time;
    vertex_constraints_.insert(st(time, node_id));
}

void ReservationTable::add_edge_constraint(int time, int n1, int n2) {
    edge_constraints_[st(time, n1)].insert(n2);
}

void ReservationTable::add_semi_static_constraint(int time, int node_id) {
    if (semi_static_constraints_.count(node_id))
        semi_static_constraints_[node_id] = std::min(time, semi_static_constraints_[node_id]);
    else
        semi_static_constraints_[node_id] = time;
}

void ReservationTable::clear_semi_static_constraints() {
    semi_static_constraints_.clear();
}

bool ReservationTable::is_reserved(int time, int node_id) const {
    if (vertex_constraints_.count(st(time, node_id)))
        return true;

    if (semi_static_constraints_.count(node_id)) {
        if (semi_static_constraints_.at(node_id) <= time)
            return true;
    }

    return false;
}

std::unordered_set<int> ReservationTable::get_reserved_edges(int time, int node_id) const {
    int st_ = st(time, node_id);
    if (edge_constraints_.count(st_))
        return edge_constraints_.at(st_);
    else {
        std::unordered_set<int> nodes;
        return nodes;
    }
}

void ReservationTable::add_path(
    int start_time,
    vector<int> &path,
    bool reserve_destination,
    bool add_edge_constraints
) {
    if (path.empty())
        return;

    for (size_t i = 0; i < path.size(); i++) {
        int time = start_time + i;
        if (time > max_time_) {
            max_time_ = time;
        }
        if (add_edge_constraints && i > 0) {
            edge_constraints_[st(time - 1, path[i])].insert(path[i - 1]);
        }
        vertex_constraints_.insert(st(time, path[i]));
    }

    if (reserve_destination) {
        semi_static_constraints_[path.back()] = start_time + path.size();
    }
}

int ReservationTable::last_time_reserved(int node_id) const {
    for (int time = max_time_; time >= 0; time--) {
        if (vertex_constraints_.count(st(time, node_id))) {
            return time;
        }
    }
    return 0;
}

void ReservationTable::print() const {
    if (vertex_constraints_.empty() && edge_constraints_.empty() && semi_static_constraints_.empty()) {
        cout << "No constraints" << endl;
        return;
    }

    if (!vertex_constraints_.empty()) {
        cout << "Vertex constraints:" << endl;
        for (auto st : vertex_constraints_) {
            int time = st / graph_size;
            int node_id = st % graph_size;
            cout << "- time=" << time << ", node=" << node_id << endl;
        }
    }

    if (!edge_constraints_.empty()) {
        cout << "Edge constraints:" << endl;
        for (auto &[st, d] : edge_constraints_) {
            int time = st / graph_size;
            int n1 = st % graph_size;
            for (auto n2 : d) {
                cout << "- time=" << time << ", edge=" << n1 << "->" << n2 << endl;
            }
        }
    }

    if (!semi_static_constraints_.empty()) {
        cout << "Semi static constraints:" << endl;
        for (auto &[node_id, time] : semi_static_constraints_) {
            cout << "- time=" << time << ", node=" << node_id << endl;
        }
    }
}
