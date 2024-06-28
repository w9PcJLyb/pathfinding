#include "include/bi_dijkstra.h"


BiDijkstra::BiDijkstra(AbsGraph *graph) : graph(graph) {
    reversed_graph_ = graph->reverse();
    forward_nodes_.resize(graph->size());
    backward_nodes_.resize(graph->size());
    closedset_.resize(graph->size(), 0);
}

BiDijkstra::~BiDijkstra() {
    delete reversed_graph_;
} 

void BiDijkstra::clear() {
    for (int i : workset_) {
        forward_nodes_[i].clear();
        backward_nodes_[i].clear();
        closedset_[i] = 0;
    }
    workset_.clear();
}

vector<int> BiDijkstra::reconstruct_path(int start, int end) {
    double min_distance = -1;
    int middle_point = -1;
    for (int i : workset_) {
        double d1 = forward_nodes_[i].distance, d2 = backward_nodes_[i].distance;
        if (d1 >= 0 && d2 >= 0) {
            if (min_distance < 0 || min_distance > d1 + d2) {
                min_distance = d1 + d2;
                middle_point = i;
            }
        }
    }
    if (middle_point == -1)
        return {};

    int p;
    vector<int> path;
    
    // from start to middle 
    p = middle_point;
    while (p != start) {
        p = forward_nodes_[p].parent;
        path.push_back(p);
    }
    std::reverse(path.begin(), path.end());

    // from middle to end
    p = middle_point;
    path.push_back(middle_point);
    while (p != end) {
        p = backward_nodes_[p].parent;
        path.push_back(p);
    }

    return path;
}

bool BiDijkstra::step(Queue &queue, vector<Node> &nodes, AbsGraph* g) {
    int node_id = -1;
    key top;
    while (!queue.empty()) {
        top = queue.top();
        queue.pop();

        if (top.first > nodes[top.second].distance)
            continue;
        
        node_id = top.second;
        break;
    }

    if (node_id == -1)
        return false;

    if (closedset_[node_id])
        return false;

    closedset_[node_id] = 1;

    for (auto& [n, cost] : g->get_neighbours(node_id)) {
        Node &nb = nodes[n];
        double new_cost = top.first + cost;
        if (nb.distance < 0 || nb.distance - new_cost > epsilon) {
            nb.parent = node_id;
            nb.distance = new_cost;
            queue.push({new_cost, n});
            workset_.push_back(n);
        }
    }

    return true;
}

vector<int> BiDijkstra::find_path(int start, int end) {
    clear();

    Queue forward_queue, backward_queue;
    forward_queue.push({0., start});
    backward_queue.push({0., end});

    forward_nodes_[start].distance = 0.;
    backward_nodes_[end].distance = 0.;

    workset_.push_back(start);
    workset_.push_back(end);

    while (
        step(forward_queue, forward_nodes_, graph)
        && step(backward_queue, backward_nodes_, reversed_graph_)
    ) {
    }

    return reconstruct_path(start, end);
}
