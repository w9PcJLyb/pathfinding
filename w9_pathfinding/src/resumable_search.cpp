#include "include/resumable_search.h"


ResumableBFS::ResumableBFS(AbsGraph *graph, int start, bool reverse) : ResumableSearch(graph, start, reverse) {
    nodes_.resize(graph->size());

    openset_.push(start);
    Node &n0 = nodes_[start];
    n0.distance = 0;
}

void ResumableBFS::clear() {
    std::queue<int> empty;
    std::swap(openset_, empty);
    for (Node &node : nodes_)
        node.clear();
}

void ResumableBFS::set_start_node(int start) {
    if (start_ != start) {
        start_ = start;
        clear();

        openset_.push(start);
        Node &n0 = nodes_[start];
        n0.distance = 0;
    }
}

double ResumableBFS::distance(int node_id) {
    Node& node = nodes_[node_id];
    if (node.distance < 0)
        search(node_id);

    return node.distance;
}

vector<int> ResumableBFS::reconstruct_path(int node_id) {
    int p = node_id;
    vector<int> path;
    while (p >= 0) {
        path.push_back(p);
        p = nodes_[p].parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

vector<int> ResumableBFS::find_path(int node_id) {
    Node& node = nodes_[node_id];
    if (node.distance < 0)
        search(node_id);

    if (node.distance >= 0)
        return reconstruct_path(node_id);

    return {};
}

void ResumableBFS::search(int node_id) {
    while (!openset_.empty()) {
        int current_id = openset_.front();
        openset_.pop();

        Node& current = nodes_[current_id];

        for (auto& [n, cost] : graph->get_neighbors(current_id, reverse_)) {
            Node &node = nodes_[n];
            if (node.distance < 0) {
                node.parent = current_id;
                node.distance = current.distance + 1;
                openset_.push(n);
            }
        }

        if (current_id == node_id)
            return;
    }
}

ResumableAStar::ResumableAStar(AbsGraph *graph, int start, bool reverse) : ResumableSearch(graph, start, reverse) {
    nodes_.resize(graph->size());

    openset_.push({0, start});
    Node &n0 = nodes_[start];
    n0.distance = 0;
}

void ResumableAStar::clear() {
    openset_ = Queue();
    for (Node &node : nodes_)
        node.clear();
}

void ResumableAStar::set_start_node(int start) {
    if (start_ != start) {
        start_ = start;
        clear();

        openset_.push({0, start});
        Node &n0 = nodes_[start];
        n0.distance = 0;
    }
}

double ResumableAStar::distance(int node_id) {
    Node& node = nodes_[node_id];
    if (!node.closed)
        search(node_id);

    return node.distance;
}

void ResumableAStar::search(int node_id) {
    if (end_ == -1)
        end_ = node_id;

    while (!openset_.empty()) {
        key top = openset_.top();
        openset_.pop();

        int current_id = top.second;
        Node& current = nodes_[current_id];

        if (current.closed)
            continue;

        current.closed = true;

        for (auto& [n, cost] : graph->get_neighbors(current_id, reverse_)) {
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
}

ResumableDijkstra::ResumableDijkstra(AbsGraph *graph, int start, bool reverse) : ResumableSearch(graph, start, reverse) {
    nodes_.resize(graph->size());

    openset_.push({0, start});
    Node &n0 = nodes_[start]; 
    n0.distance = 0;
}

void ResumableDijkstra::clear() {
    openset_ = Queue();
    for (Node &node : nodes_)
        node.clear();
}

void ResumableDijkstra::set_start_node(int start) {
    if (start_ != start) {
        start_ = start;
        clear();

        openset_.push({0, start});
        Node &n0 = nodes_[start];
        n0.distance = 0;
    }
}

double ResumableDijkstra::distance(int node_id) {
    Node& node = nodes_[node_id];
    if (!node.closed)
        search(node_id);

    return node.distance;
}

vector<int> ResumableDijkstra::reconstruct_path(int node_id) {
    int p = node_id;
    vector<int> path;
    while (p >= 0) {
        path.push_back(p);
        p = nodes_[p].parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

vector<int> ResumableDijkstra::find_path(int node_id) {
    Node& node = nodes_[node_id];
    if (!node.closed)
        search(node_id);

    if (node.distance >= 0)
        return reconstruct_path(node_id);

    return {};
}

void ResumableDijkstra::search(int node_id) {
    while (!openset_.empty()) {
        key top = openset_.top();
        openset_.pop();

        int current_id = top.second;
        Node& current = nodes_[current_id];

        if (current.closed)
            continue;

        current.closed = true;

        for (auto& [n, cost] : graph->get_neighbors(current_id, reverse_)) {
            Node &node = nodes_[n];
            double new_distance = current.distance + cost;
            if (node.distance < 0 || node.distance > new_distance) {
                node.distance = new_distance;
                node.parent = current_id;
                openset_.push({new_distance, n});
            }
        }

        if (current_id == node_id)
            return;
    }

    Node& node = nodes_[node_id];
    node.closed = true;
}
