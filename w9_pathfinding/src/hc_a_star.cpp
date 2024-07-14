#include <unordered_map>

#include "include/hc_a_star.h"


ResumableAStar::ResumableAStar(AbsGraph *graph, int start, int end) : graph(graph), start_(start), end_(end) {
    nodes_.resize(graph->size());

    openset_ = Queue();
    openset_.push({0, start});
    Node &n0 = nodes_[start]; 
    n0.distance = 0;
}

void ResumableAStar::clear() {
    openset_ = Queue();
    for (Node node : nodes_)
        node.clear();
}

double ResumableAStar::distance(int node_id) {
    Node& node = nodes_[node_id];
    if (!node.closed)
        search(node_id);

    return node.distance;
}

void ResumableAStar::search(int node_id) {

    while (!openset_.empty()) {
        key top = openset_.top();
        openset_.pop();

        int current_id = top.second;
        Node& current = nodes_[current_id];

        if (current.closed)
            continue;

        current.closed = true;

        for (auto& [n, cost] : graph->get_neighbours(current_id)) {
            Node &node = nodes_[n];
            double new_distance = current.distance + cost;
            if (node.distance < 0) {
                node.f = new_distance + graph->estimate_distance(n, end_);
                node.distance = new_distance;
                openset_.push({node.f, n});
            }
            else if (node.distance > new_distance) {
                node.f = node.f - node.distance + new_distance;
                node.distance = new_distance;
                openset_.push({node.f, n});
            }
        }

        if (current_id == node_id)
            return;
    }

    Node& node = nodes_[node_id];
    node.closed = true;
    node.distance = -1;
}

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

vector<int> HCAStar::find_path(int start, int end) {
    return find_path(start, end, 100, nullptr);
}

vector<int> HCAStar::find_path(int start, int end, int search_depth) {
    return find_path(start, end, search_depth, nullptr);
}

vector<int> HCAStar::find_path(int start, int end, int search_depth, ReservationTable *rt) {
    ResumableAStar rra(reversed_graph_, end, start);

    if (rt == nullptr) {
        ReservationTable rt_(graph->size());
        return find_path_(start, end, search_depth, rra, rt_);
    }
    else {
        assert(rt->graph_size == graph->size());
        return find_path_(start, end, search_depth, rra, *rt);
    }
}

vector<int> HCAStar::find_path_(
    int start,
    int end,
    int search_depth,
    ResumableAStar &rra,
    ReservationTable &rt
) {
    double f0 = rra.distance(start);
    if (f0 == -1)
        // unreachable
        return {};

    int graph_size = graph->size();
    double pause_action_cost = graph->get_pause_action_cost();
    bool pause_action_allowed = graph->is_pause_action_allowed();

    int min_search_depth = rt.last_time_reserved(end);

    Queue openset;

    Node* n0 = new Node(nullptr, start, 0, 0, f0);

    openset.push({0, n0});
    std::unordered_map<int, Node*> nodes;
    nodes[start] = n0;

    auto process_node = [&] (int node_id, double cost, Node* current) {
        int time = current->time + 1;

        if (rt.reserved(time, node_id)) {
            return;
        }

        double distance = current->distance + cost;

        int h = node_id + time * graph_size;
        if (!nodes.count(h)) {
            double f = distance + rra.distance(node_id);
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
    };

    while (!openset.empty()) {
        auto [f, current] = openset.top();
        openset.pop();

        if (current->node_id == end) {
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

        if (current->time >= search_depth) {
            // terminal node
            process_node(end, rra.distance(current->node_id), current);
            nodes[end + (current->time + 1) * graph_size]->time = -1;
        }
        else {
            if (pause_action_allowed)
                process_node(current->node_id, pause_action_cost, current);

            for (auto &[node_id, cost] : graph->get_neighbours(current->node_id)) {
                process_node(node_id, cost, current);
            }
        }
    }

    for (auto it : nodes)
        delete it.second;

    return {};
}

vector<vector<int>> HCAStar::mapf(
    vector<int> starts,
    vector<int> goals,
    int search_depth,
    bool despawn_at_destination
) {
    ReservationTable *rt = new ReservationTable(graph->size());
    auto paths = mapf(starts, goals, search_depth, despawn_at_destination, rt);
    delete rt;
    return paths;
}

vector<vector<int>> HCAStar::mapf(
    vector<int> starts,
    vector<int> goals,
    int search_depth,
    bool despawn_at_destination,
    ReservationTable *rt
) {
    assert(starts.size() == goals.size());
    assert(rt->graph_size == graph->size());

    if (starts.size() == 0)
        return {};

    vector<vector<int>> paths;
    for (size_t i = 0; i < starts.size(); i++) {
        ResumableAStar rra(reversed_graph_, goals[i], starts[i]);
        vector<int> path = find_path_(starts[i], goals[i], search_depth, rra, *rt);
        paths.push_back(path);
        rt->add_path(i, 0, path, !despawn_at_destination);
    }

    if (!despawn_at_destination) {
        // all paths must have the same size
        size_t max_size = paths[0].size();
        for (size_t i = 1; i < paths.size(); i++) {
            max_size = std::max(max_size, paths[i].size());
        }

        for (size_t i = 0; i < paths.size(); i++) {
            if (paths[i].empty()) {
                vector<int> path(max_size, starts[i]);
                paths[i] = path;
            }
            else if (paths[i].size() < max_size) {
                vector<int> path(max_size - paths[i].size(), paths[i].back());
                paths[i].insert(paths[i].end(), path.begin(), path.end());
            }
        }
    }

    return paths;
}
