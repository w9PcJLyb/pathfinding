#include "include/space_time_a_star.h"


SpaceTimeAStar::SpaceTimeAStar(AbsGraph *graph) : graph(graph), rrs_(nullptr) {
}

SpaceTimeAStar::~SpaceTimeAStar() {
    delete rrs_;
}

Path SpaceTimeAStar::reconstruct_path(int start, Node* node) {
    if (node->time == -1) {
        // is terminal node
        node = node->parent;
    }

    Path path = {node->node_id};
    while (node->parent != nullptr) {
        node = node->parent;
        path.push_back(node->node_id);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

ResumableSearch* SpaceTimeAStar::reverse_resumable_search(int node_id) {
    if (graph->has_coordinates())
        return new ResumableAStar(graph, node_id, true);
    else
        return new ResumableDijkstra(graph, node_id, true);
}

ResumableSearch* SpaceTimeAStar::ensure_rrs(ResumableSearch* rrs, int goal) {
    if (rrs)
        return rrs;

    if (!rrs_)
        rrs_ = reverse_resumable_search(goal);
    else
        rrs_->set_start_node(goal);

    return rrs_;
}

Path SpaceTimeAStar::find_path(int start, int end) {
    return find_path_with_depth_limit(start, end, 100, nullptr);
}

Path SpaceTimeAStar::find_path_with_depth_limit(
    int start,
    int goal,
    int search_depth,
    const ReservationTable *rt,
    ResumableSearch *rrs,
    int min_terminal_time,
    int start_time
) {
    rrs = ensure_rrs(rrs, goal);

    if (!rt || rt->empty()) {
        // the time dimension is not needed in this case
        Path path = rrs->find_path(start);
        if (path.empty())
            return path;

        // we used a reversed search, so we need to reverse the result
        std::reverse(path.begin(), path.end());

        int min_length = min_terminal_time - start_time;
        if ((int)path.size() < min_length + 1) {
            Path rest(min_length + 1 - path.size(), path.back());
            path.insert(path.end(), rest.begin(), rest.end());
        }

        if ((int)path.size() > search_depth + 1)
            path.resize(search_depth + 1);

        return path;
    }

    // Space-Time A*

    double f0 = rrs->distance(start);
    if (f0 == -1) {
        // unreachable
        return {};
    }

    int graph_size = graph->size();
    int max_terminal_time = start_time + search_depth;
    double pause_action_cost = graph->get_pause_action_cost();

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
            if (current->time >= min_terminal_time || current->time == -1) {
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

        if (current->time >= max_terminal_time) {
            // terminal node
            if (process_node(goal, rrs->distance(current->node_id), current)) {
                nodes[goal + (current->time + 1) * graph_size]->time = -1;
            }
        }
        else {
            process_node(current->node_id, pause_action_cost, current);

            auto reserved_edges = rt->get_reserved_edges(current->time, current->node_id);
            for (auto &[node_id, cost] : graph->get_neighbors(current->node_id)) {
                if (!reserved_edges.count(node_id))
                    process_node(node_id, cost, current);
            }
        }
    }

    for (auto it : nodes)
        delete it.second;

    return {};
}
