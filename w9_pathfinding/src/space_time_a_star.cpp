#include "include/space_time_a_star.h"


SpaceTimeAStar::SpaceTimeAStar(AbsGraph *graph) : graph(graph), rrs_(nullptr) {
}

Path SpaceTimeAStar::reconstruct_path(int start, Node* node) {
    Path path = {node->node_id};
    while (node->parent != nullptr) {
        node = node->parent;
        path.push_back(node->node_id);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::unique_ptr<ResumableSearch> SpaceTimeAStar::reverse_resumable_search(int node_id) {
    if (graph->has_heuristic())
        return std::make_unique<ResumableAStar>(graph, node_id, true);
    else
        return std::make_unique<ResumableDijkstra>(graph, node_id, true);
}

ResumableSearch* SpaceTimeAStar::ensure_rrs(ResumableSearch* rrs, int goal) {
    if (rrs)
        return rrs;

    if (!rrs_)
        rrs_ = reverse_resumable_search(goal);
    else
        rrs_->set_start_node(goal);

    return rrs_.get();
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

        ensure_path_length(path, min_terminal_time - start_time);

        if ((int)path.size() > search_depth + 1)
            path.resize(search_depth + 1);

        return path;
    }

    // Space-Time A*

    const ReservationTable& rt_ = *rt;

    double f0 = rrs->distance(start);
    if (f0 == -1) {
        // unreachable
        return {};
    }

    int graph_size = graph->size();
    int max_terminal_time = start_time + search_depth;

    typedef pair<double, Node*> key;
    priority_queue<key, vector<key>, std::greater<key>> openset;

    std::unordered_map<int, Node> nodes;
    nodes.emplace(start, Node(nullptr, start, start_time, 0, f0));
    openset.push({0, &nodes.at(start)});

    auto process_node = [&] (int node_id, double cost, Node* current) {
        int time = current->time + 1;

        double h = rrs->distance(node_id);
        if (h == -1)
            return false;

        double distance = current->distance + cost;

        int st = node_id + time * graph_size;
        if (!nodes.count(st)) {
            nodes.emplace(st, Node(current, node_id, time, distance, distance + h));
            openset.push({distance + h, &nodes.at(st)});
        }
        else if (nodes.at(st).distance > distance) {
            Node& n = nodes.at(st);
            n.f = distance + h;
            n.distance = distance;
            n.parent = current;
            openset.push({n.f, &n});
        }
        return true;
    };

    while (!openset.empty()) {
        auto [f, current] = openset.top();
        openset.pop();

        int t = current->time;
        int n = current->node_id;

        if (n == goal) {
            if (t >= min_terminal_time)
                return reconstruct_path(start, current);
            else if (t == -1) {
                // terminal node
                return reconstruct_path(start, current->parent);
            }
        }

        int h = n + t * graph_size;
        if (nodes.count(h) && f > nodes.at(h).f)
            continue;

        if (t >= max_terminal_time) {
            // terminal node
            if (process_node(goal, rrs->distance(n), current))
                nodes.at(goal + (t + 1) * graph_size).time = -1;
        }
        else {
            auto reserved_edges = rt_.get_reserved_edges(t, n);
            for (auto &[node_id, cost] : graph->get_neighbors(n, false, true)) {
                if (!reserved_edges.count(node_id) && !rt_.is_reserved(t + 1, node_id))
                    process_node(node_id, cost, current);
            }
        }
    }

    return {};
}


Path SpaceTimeAStar::find_path_with_exact_length(
    int start,
    int goal,
    int length,
    const ReservationTable *rt,
    int start_time
) {
    int graph_size = graph->size();
    int terminal_time = start_time + length;

    typedef pair<double, Node*> key;
    priority_queue<key, vector<key>, std::greater<key>> openset;

    std::unordered_map<int, Node> nodes;
    nodes.emplace(start, Node(nullptr, start, start_time, 0, 0));
    openset.push({0, &nodes.at(start)});

    auto process_node = [&] (int node_id, double cost, Node* current) {
        int time = current->time + 1;
        double h = graph->estimate_distance(node_id, goal);
        double distance = current->distance + cost;

        int st = node_id + time * graph_size;
        if (!nodes.count(st)) {
            nodes.emplace(st, Node(current, node_id, time, distance, distance + h));
            openset.push({distance + h, &nodes.at(st)});
        }
        else if (nodes.at(st).distance > distance) {
            Node& n = nodes.at(st);
            n.f = distance + h;
            n.distance = distance;
            n.parent = current;
            openset.push({n.f, &n});
        }
    };

    while (!openset.empty()) {
        auto [f, current] = openset.top();
        openset.pop();

        int time = current->time;

        if (time == terminal_time) {
            if (current->node_id == goal)
                return reconstruct_path(start, current);
            continue;
        }

        int h = current->node_id + time * graph_size;
        if (nodes.count(h) && f > nodes.at(h).f)
            continue;

        if (!rt) {
            for (auto &[node_id, cost] : graph->get_neighbors(current->node_id, false, true))
                process_node(node_id, cost, current);
        }
        else {
            auto reserved_edges = rt->get_reserved_edges(time, current->node_id);
            for (auto &[node_id, cost] : graph->get_neighbors(current->node_id, false, true)) {
                if (!reserved_edges.count(node_id) && !rt->is_reserved(time + 1, node_id))
                    process_node(node_id, cost, current);
            }
        }
    }

    return {};
}


Path SpaceTimeAStar::find_path_with_length_limit(
    int start,
    int goal,
    int max_length,
    const ReservationTable *rt,
    ResumableSearch* rrs,
    int min_terminal_time,
    int start_time
) {
    if (!rt || rt->empty()) {
        if (rrs && rrs->distance(start) < 0)
            return {};

        Path path = find_path_with_length_limit__static(start, goal, max_length);
        if (path.empty())
            return path;

        ensure_path_length(path, min_terminal_time - start_time);
        return path;
    }

    rrs = ensure_rrs(rrs, goal);

    ResumableSearch& rrs_ = *rrs;
    if (rrs_.distance(start) < 0)
        return {};

    const ReservationTable& rt_ = *rt;

    int graph_size = graph->size();
    int terminal_time = start_time + max_length;

    typedef pair<double, Node*> key;
    priority_queue<key, vector<key>, std::greater<key>> openset;

    std::unordered_map<int, Node> nodes;
    nodes.emplace(start, Node(nullptr, start, start_time, 0, 0));
    openset.push({0, &nodes.at(start)});

    auto process_node = [&] (int node_id, double cost, Node* current) {
        int time = current->time + 1;
        double h = rrs_.distance(node_id);
        double distance = current->distance + cost;

        int st = node_id + time * graph_size;
        if (!nodes.count(st)) {
            nodes.emplace(st, Node(current, node_id, time, distance, distance + h));
            openset.push({distance + h, &nodes.at(st)});
        }
        else if (nodes.at(st).distance > distance) {
            Node& n = nodes.at(st);
            n.f = distance + h;
            n.distance = distance;
            n.parent = current;
            openset.push({n.f, &n});
        }
    };

    while (!openset.empty()) {
        auto [f, current] = openset.top();
        openset.pop();

        int time = current->time;

        if (time >= min_terminal_time && current->node_id == goal)
            return reconstruct_path(start, current);

        if (time >= terminal_time)
            continue;

        int h = current->node_id + time * graph_size;
        if (nodes.count(h) && f > nodes.at(h).f)
            continue;

        auto reserved_edges = rt_.get_reserved_edges(time, current->node_id);
        for (auto &[node_id, cost] : graph->get_neighbors(current->node_id, false, true)) {
            if (!reserved_edges.count(node_id) && !rt_.is_reserved(time + 1, node_id))
                process_node(node_id, cost, current);
        }
    }

    return {};
}


Path SpaceTimeAStar::find_path_with_length_limit__static(int start, int goal, int max_length) {
    typedef pair<double, Node*> key;
    priority_queue<key, vector<key>, std::greater<key>> openset;

    std::unordered_map<int, Node> nodes;
    nodes.emplace(start, Node(nullptr, start, 0, 0, 0));
    openset.push({0, &nodes.at(start)});

    while (!openset.empty()) {
        auto [f, current] = openset.top();
        openset.pop();

        if (current->node_id == goal)
            return reconstruct_path(start, current);

        if (current->time == max_length)
            continue;

        if (nodes.count(current->node_id) && f > nodes.at(current->node_id).f)
            continue;

        int time = current->time + 1;
        for (auto &[node_id, cost] : graph->get_neighbors(current->node_id)) {
            double h = 0;
            if (graph->has_heuristic())
                h = graph->estimate_distance(node_id, goal);

            double distance = current->distance + cost;

            if (!nodes.count(node_id)) {
                nodes.emplace(node_id, Node(current, node_id, time, distance, distance + h));
                openset.push({distance + h, &nodes.at(node_id)});
            }
            else if (nodes.at(node_id).distance > distance) {
                Node& n = nodes.at(node_id);
                n.f = distance + h;
                n.distance = distance;
                n.parent = current;
                n.time = time;
                openset.push({n.f, &n});
            }
        }
    }

    return {};
}
