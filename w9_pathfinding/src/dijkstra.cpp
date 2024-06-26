#include "include/dijkstra.h"


Dijkstra::Dijkstra(AbsGraph *graph) : graph(graph) {
    nodes_.resize(graph->size());
}

void Dijkstra::clear() {
    for (Node &node : nodes_)
        node.clear();
}

vector<int> Dijkstra::reconstruct_path(int start, int end) {
    int p = end;
    vector<int> path = {p};
    while (p != start) {
        p = nodes_[p].parent;
        path.push_back(p);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

vector<int> Dijkstra::find_path(int start, int end) {
    clear();

    Queue openset; 

    nodes_[start].distance = 0;
    openset.push({0, start});

    while (!openset.empty()) {
        key top = openset.top();
        openset.pop();

        double distance = nodes_[top.second].distance;
        if (top.first > distance)
            continue;

        if (top.second == end) {
            return reconstruct_path(start, end);
        }

        for (auto& [n, cost] : graph->get_neighbours(top.second)) {
            double total_cost = distance + cost;
            Node &node = nodes_[n];
            if (node.distance < 0 || node.distance > total_cost) {
                node.distance = total_cost;
                node.parent = top.second;
                openset.push({total_cost, n});
            }
        }
    }

    return {};
}
