#include <unordered_map>

#include "include/hc_a_star.h"


HCAStar::HCAStar(AbsGraph *graph) : graph(graph) {
    reversed_graph_ = graph->reverse();
}

HCAStar::~HCAStar() {
    delete reversed_graph_;
}

vector<int> HCAStar::reconstruct_path(int start, Node* node) {
    if (node->time == -1) {
        // is terminal node
        node = node->parent;
    }

    vector<int> path = {node->node_id};
    while (node->parent != nullptr) {
        node = node->parent;
        path.push_back(node->node_id);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

ResumableSearch* HCAStar::reverse_resumable_search(int node_id) {
    if (graph->has_coordinates())
        return new ResumableAStar(reversed_graph_, node_id);
    else
        return new ResumableDijkstra(reversed_graph_, node_id);
}

vector<int> HCAStar::find_path(int start, int end) {
    return find_path(start, end, 100, nullptr);
}

vector<int> HCAStar::find_path(int start, int end, int search_depth, const ReservationTable *rt) {
    ResumableSearch* rrs_ = reverse_resumable_search(end);
    vector<int> path;

    if (!rt) {
        ReservationTable rt_(graph->size());
        path = find_path_(0, start, end, search_depth, rrs_, rt_);
    }
    else {
        assert(rt->graph_size == int(graph->size()));
        path = find_path_(0, start, end, search_depth, rrs_, *rt);
    }

    delete rrs_;
    return path;
}

vector<int> HCAStar::find_path_(
    int start_time,
    int start,
    int goal,
    int search_depth,
    ResumableSearch *rrs,
    const ReservationTable &rt
) {
    double f0 = rrs->distance(start);
    if (f0 == -1)
        // unreachable
        return {};

    int graph_size = graph->size();
    int terminal_time = start_time + search_depth;
    double pause_action_cost = graph->get_pause_action_cost();
    bool pause_action_allowed = graph->is_pause_action_allowed();

    int min_search_depth = rt.last_time_reserved(goal);

    Queue openset;

    Node* n0 = new Node(nullptr, start, start_time, 0, f0);

    openset.push({0, n0});
    std::unordered_map<int, Node*> nodes;
    nodes[start] = n0;

    auto process_node = [&] (int node_id, double cost, Node* current) {
        int time = current->time + 1;

        if (rt.is_reserved(time, node_id)) {
            return false;
        }

        double distance = current->distance + cost;

        int h = node_id + time * graph_size;
        if (!nodes.count(h)) {
            double f = distance + rrs->distance(node_id);
            Node* n = new Node(current, node_id, time, distance, f);
            nodes[h] = n;
            openset.push({f, n});
        }
        else if (nodes[h]->distance > distance) {
            Node *n = nodes[h];
            n->f = n->f - n->distance + distance;
            n->distance = distance;
            n->parent = current;
            openset.push({n->f, n});
        }
        return true;
    };

    while (!openset.empty()) {
        auto [f, current] = openset.top();
        openset.pop();

        if (current->node_id == goal) {
            if (current->time >= min_search_depth || current->time == -1) {
                auto path = reconstruct_path(start, current);
                for (auto it : nodes)
                    delete it.second;
                return path;
            }
        }

        int h = current->node_id + current->time * graph_size;
        if (nodes.count(h) && f > nodes[h]->f) {
            continue;
        }

        if (current->time >= terminal_time) {
            // terminal node
            if (process_node(goal, rrs->distance(current->node_id), current)) {
                nodes[goal + (current->time + 1) * graph_size]->time = -1;
            }
        }
        else {
            if (pause_action_allowed)
                process_node(current->node_id, pause_action_cost, current);

            int reserved_edge = rt.get_reserved_edge(current->time, current->node_id);
            for (auto &[node_id, cost] : graph->get_neighbors(current->node_id)) {
                if (reserved_edge != node_id)
                    process_node(node_id, cost, current);
            }
        }
    }

    for (auto it : nodes)
        delete it.second;

    return {};
}

vector<vector<int>> HCAStar::mapf(vector<int> starts, vector<int> goals) {
    return mapf(starts, goals, 100, false, nullptr);
}

vector<vector<int>> HCAStar::mapf(
    vector<int> starts,
    vector<int> goals,
    int search_depth,
    bool despawn_at_destination,
    const ReservationTable *rt
) {
    assert(starts.size() == goals.size());

    if (starts.size() == 0)
        return {};

    ReservationTable reservation_table(graph->size());
    if (rt)
        reservation_table = *rt;

    bool edge_collision = graph->edge_collision();

    vector<vector<int>> paths;
    for (size_t i = 0; i < starts.size(); i++) {
        ResumableSearch* rrs_ = reverse_resumable_search(goals[i]);
        vector<int> path = find_path_(0, starts[i], goals[i], search_depth, rrs_, reservation_table);
        delete rrs_;
        paths.push_back(path);
        reservation_table.add_path(i, 0, path, !despawn_at_destination, edge_collision);
    }

    if (!despawn_at_destination) {
        // all paths must have the same size
        size_t max_size = paths[0].size();
        for (size_t i = 1; i < paths.size(); i++) {
            max_size = std::max(max_size, paths[i].size());
        }

        for (size_t i = 0; i < paths.size(); i++) {
            if (paths[i].empty()) {
                continue;
            }
            else if (paths[i].size() < max_size) {
                vector<int> path(max_size - paths[i].size(), paths[i].back());
                paths[i].insert(paths[i].end(), path.begin(), path.end());
            }
        }
    }

    return paths;
}
