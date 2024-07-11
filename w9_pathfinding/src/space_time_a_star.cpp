#include <map>

#include "include/space_time_a_star.h"


AStarBackwardSearch::AStarBackwardSearch(AbsGraph *graph_) {
    graph = graph_->reverse();
    nodes_.resize(graph->size());
}

void AStarBackwardSearch::clear() {
    openset_ = Queue();
    for (Node *node : workset_)
        node->clear();
    workset_.clear();
}

AStarBackwardSearch::~AStarBackwardSearch() {
    delete graph;
}

void AStarBackwardSearch::set_direction(int start, int end) {
    clear();
    start_ = start;
    end_ = end;

    openset_.push({0, start});
    Node &n0 = nodes_[start]; 
    n0.distance = 0;
    workset_.push_back(&n0);
}

double AStarBackwardSearch::distance(int node_id) {
    Node& node = nodes_[node_id];
    if (!node.closed)
        search(node_id);

    return node.distance;
}

void AStarBackwardSearch::search(int node_id) {

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
                workset_.push_back(&node);
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
    workset_.push_back(&node);
}

SpaceTimeAStar::SpaceTimeAStar(AbsGraph *graph) : graph(graph), backward_search_(graph) {
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

vector<int> SpaceTimeAStar::find_path(int start, int end) {
    return find_path(start, end, -1);
}

vector<int> SpaceTimeAStar::find_path(int start, int end, int max_steps) {
    backward_search_.set_direction(end, start);

    double f0 = backward_search_.distance(start);
    if (f0 == -1)
        // unreachable
        return {};

    double pause_action_cost = graph->get_pause_action_cost();
    bool pause_action_allowed = graph->is_pause_action_allowed();

    Queue openset;

    Node* n0 = new Node(nullptr, start, 0, 0, f0);

    openset.push({0, n0});
    std::map<pair<int, int>, Node*> nodes;
    nodes[{0, start}] = n0;

    auto process_node = [&] (int node_id, double cost, Node* current) {
        int time = current->time + 1;

        if (graph->has_dynamic_obstacle(time, node_id)) {
            return;
        }

        double distance = current->distance + cost;

        if (!nodes.count({time, node_id})) {
            double f = distance + backward_search_.distance(node_id);
            Node* n = new Node(current, node_id, time, distance, f);
            nodes[{time, node_id}] = n;
            openset.push({f, n});
        }
        else if (nodes[{time, node_id}]->distance > distance) {
            Node *n = nodes[{time, node_id}];
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
            auto path = reconstruct_path(start, current);
            for (auto it : nodes)
                delete it.second;
            return path;
        }

        if (
            nodes.count({current->time, current->node_id})
            && f > nodes[{current->time, current->node_id}]->f
        ) {
            continue;
        }

        if (max_steps > 0 && current->time >= max_steps) {
            // terminal node
            process_node(end, backward_search_.distance(current->node_id), current);
            nodes[{current->time + 1, end}]->time = -1;
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