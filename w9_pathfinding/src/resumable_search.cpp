#include "include/resumable_search.h"


ResumableAStar::ResumableAStar(AbsGraph *graph, int start) : ResumableSearch(graph, start) {
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

        for (auto& [n, cost] : graph->get_neighbors(current_id)) {
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

ResumableDijkstra::ResumableDijkstra(AbsGraph *graph, int start) : ResumableSearch(graph, start) {
    nodes_.resize(graph->size());

    openset_ = Queue();
    openset_.push({0, start});
    Node &n0 = nodes_[start]; 
    n0.distance = 0;
}

void ResumableDijkstra::clear() {
    openset_ = Queue();
    for (Node node : nodes_)
        node.clear();
}

double ResumableDijkstra::distance(int node_id) {
    Node& node = nodes_[node_id];
    if (!node.closed)
        search(node_id);

    return node.distance;
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

        for (auto& [n, cost] : graph->get_neighbors(current_id)) {
            Node &node = nodes_[n];
            double new_distance = current.distance + cost;
            if (node.distance < 0 || node.distance > new_distance) {
                node.distance = new_distance;
                openset_.push({new_distance, n});
            }
        }

        if (current_id == node_id)
            return;
    }

    Node& node = nodes_[node_id];
    node.closed = true;
    node.distance = -1;
}
