#include "include/a_star.h"


AStar::AStar(Grid *grid, int heuristic) : grid(grid) {
    if (heuristic < 0 || heuristic > 2) {
        std::invalid_argument("Unknown heuristic " + std::to_string(heuristic));
    }

    // 0 - manhattan
    // 1 - chebyshev
    // 2 - euclidean
    heuristic_ = heuristic;

    nodes_.resize(grid->size());
}

void AStar::clear() {
    for (Node *node : workset_)
        node->clear();
    workset_.clear();
}

double AStar::potential(int node1, int node2) const {
    if (heuristic_ == 0)
        return grid->manhattan_distance(node1, node2);
    else if (heuristic_ == 1)
        return grid->chebyshev_distance(node1, node2);
    else 
        return grid->euclidean_distance(node1, node2);
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
        for (auto& [n, cost] : grid->get_neighbours(x)) {
            Node &node = nodes_[n]; 
            if (node.distance < 0 || node.distance > distance + cost) {
                double f = distance + cost + potential(n, end);
                node.f = f;
                node.distance = distance + 1;
                node.parent = x;
                openset.push({f, n});
                workset_.push_back(&node);
            }
        }
    }
    return {};
}
