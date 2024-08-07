#include "include/space_time_a_star.h"


SpaceTimeAStar::SpaceTimeAStar(AbsGraph *graph) : graph(graph) {
    reversed_graph_ = graph->reverse();
}

SpaceTimeAStar::~SpaceTimeAStar() {
    delete reversed_graph_;
}

vector<int> SpaceTimeAStar::reconstruct_path(int start, Node* node) {
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

ResumableSearch* SpaceTimeAStar::reverse_resumable_search(int node_id) {
    if (graph->has_coordinates())
        return new ResumableAStar(reversed_graph_, node_id);
    else
        return new ResumableDijkstra(reversed_graph_, node_id);
}

vector<int> SpaceTimeAStar::find_path(int start, int end) {
    return find_path(start, end, 100, nullptr);
}

vector<int> SpaceTimeAStar::find_path(int start, int end, int search_depth, const ReservationTable *rt) {
    ResumableSearch* rrs_ = reverse_resumable_search(end);
    vector<int> path;

    if (!rt) {
        ReservationTable* rt_ = new ReservationTable(graph->size());
        path = find_path(0, start, end, search_depth, rrs_, rt_).first;
        delete rt_;
    }
    else {
        assert(rt->graph_size == int(graph->size()));
        path = find_path(0, start, end, search_depth, rrs_, rt).first;
    }

    delete rrs_;
    return path;
}

pair<vector<int>, double> SpaceTimeAStar::find_path(
    int start_time,
    int start,
    int goal,
    int search_depth,
    ResumableSearch *rrs,
    const ReservationTable *rt
) {
    double f0 = rrs->distance(start);
    if (f0 == -1) {
        // unreachable
        return {{}, -1};
    }

    int graph_size = graph->size();
    int terminal_time = start_time + search_depth;
    double pause_action_cost = graph->get_pause_action_cost();
    bool pause_action_allowed = graph->is_pause_action_allowed();

    int min_search_depth = rt->last_time_reserved(goal);

    Queue openset;

    Node* n0 = new Node(nullptr, start, start_time, 0, f0);

    openset.push({0, n0});
    std::unordered_map<int, Node*> nodes;
    nodes[start] = n0;

    auto process_node = [&] (int node_id, double cost, Node* current) {
        int time = current->time + 1;

        if (rt->is_reserved(time, node_id)) {
            return false;
        }

        double h = rrs->distance(node_id);
        if (h == -1)
            return false;

        double distance = current->distance + cost;

        int st = node_id + time * graph_size;
        if (!nodes.count(st)) {
            Node* n = new Node(current, node_id, time, distance, distance + h);
            nodes[st] = n;
            openset.push({n->f, n});
        }
        else if (nodes[st]->distance > distance) {
            Node *n = nodes[st];
            n->f = distance + h;
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
                double distance = current->distance;
                for (auto it : nodes)
                    delete it.second;
                return {path, distance};
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
            if (current->node_id == goal)
                process_node(current->node_id, 0, current);
            else if (pause_action_allowed)
                process_node(current->node_id, pause_action_cost, current);

            int reserved_edge = rt->get_reserved_edge(current->time, current->node_id);
            for (auto &[node_id, cost] : graph->get_neighbors(current->node_id)) {
                if (reserved_edge != node_id)
                    process_node(node_id, cost, current);
            }
        }
    }

    for (auto it : nodes)
        delete it.second;

    return {{}, -1};
}
