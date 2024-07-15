#include "include/a_star.h"


AStar::AStar(AbsGraph *graph) : graph(graph) {
    nodes_.resize(graph->size());
}

void AStar::clear() {
    for (Node *node : workset_)
        node->clear();
    workset_.clear();
}

vector<int> AStar::reconstruct_path(int start, int end) {
    int p = end;
    vector<int> path = {p};
    while (p != start) {
        p = nodes_[p].parent;
        path.push_back(p);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

vector<int> AStar::find_path(int start, int end) {
    clear();

    Queue openset;
    
    openset.push({0, start});
    nodes_[start].distance = 0;
    workset_.push_back(&nodes_[start]);
    
    while (!openset.empty()) {
        key top = openset.top();
        openset.pop();

        int x = top.second; 

        if (x == end) {
            return reconstruct_path(start, end);
        }

        if (top.first > nodes_[x].f) {
            continue;
        }

        double distance = nodes_[x].distance;
        for (auto& [n, cost] : graph->get_neighbors(x)) {
            Node &node = nodes_[n];
            double new_distance = distance + cost;
            if (node.distance < 0) {
                workset_.push_back(&node);
                node.f = new_distance + graph->estimate_distance(n, end);
                node.distance = new_distance;
                node.parent = x;
                openset.push({node.f, n});
            }
            else if (node.distance > new_distance) {
                node.f = node.f - node.distance + new_distance;
                node.distance = new_distance;
                node.parent = x;
                openset.push({node.f, n});
            }
        }
    }
    return {};
}
