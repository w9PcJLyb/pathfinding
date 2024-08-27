#include "include/bi_a_star.h"


BiAStar::BiAStar(AbsGraph *graph) : graph(graph) {
    nodes_.resize(2, vector<Node>(graph->size()));
}

void BiAStar::clear() {
    for (int i : workset_) {
        nodes_[0][i].clear();
        nodes_[1][i].clear();
    }
    workset_.clear();
}

vector<int> BiAStar::reconstruct_path(int start, int end) {
    double min_distance = -1;
    int middle_point = -1;
    for (int i : workset_) {
        double d1 = nodes_[0][i].distance, d2 = nodes_[1][i].distance;
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
        p = nodes_[0][p].parent;
        path.push_back(p);
    }
    std::reverse(path.begin(), path.end());

    // from middle to end
    p = middle_point;
    path.push_back(middle_point);
    while (p != end) {
        p = nodes_[1][p].parent;
        path.push_back(p);
    }

    return path;
}

double BiAStar::potential(int node_id, int side) {
    double d1 = graph->estimate_distance(node_id, start_node_);
    double d2 = graph->estimate_distance(node_id, end_node_);
    if (side == 0)
        return (d2 - d1) / 2;
    else
        return (d1 - d2) / 2;
}

bool BiAStar::step(int side, Queue &queue) {
    int node_id = -1;
    while (!queue.empty()) {
        key top = queue.top();
        queue.pop();

        if (!nodes_[side][top.second].visited) {
            node_id = top.second;
            break;
        }
    }

    if (node_id == -1)
        return false;

    if (nodes_[1 - side][node_id].visited)
        return false;

    nodes_[side][node_id].visited = true;

    double d = nodes_[side][node_id].distance;
    for (auto& [n, cost] : graph->get_neighbors(node_id, side)) {
        Node &nb = nodes_[side][n];
        if (nb.visited)
            continue;

        double new_distance = d + cost;
        if (nb.distance < 0) {
            nb.parent = node_id;
            nb.distance = new_distance;
            nb.f = new_distance + potential(n, side);
            queue.push({nb.f, n});
            workset_.push_back(n);
        }
        else if (nb.distance > new_distance) {
            nb.parent = node_id;
            nb.f = nb.f - nb.distance + new_distance;
            nb.distance = new_distance;
            queue.push({nb.f, n});
        }
    }

    return true;
}

vector<int> BiAStar::find_path(int start, int end) {
    start_node_ = start;
    end_node_ = end;

    clear();

    Queue forward_queue, backward_queue;
    forward_queue.push({0., start});
    backward_queue.push({0., end});

    nodes_[0][start].distance = 0.;
    nodes_[1][end].distance = 0.;

    workset_.push_back(start);
    workset_.push_back(end);

    while (step(0, forward_queue) && step(1, backward_queue)) {
    }

    return reconstruct_path(start, end);
}
