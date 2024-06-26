#include "include/bidijkstra.h"


BiDijkstra::BiDijkstra(AbsGraph *graph) : graph(graph) {
    reversed_graph_ = graph->reverse();
    nodes_.resize(2, vector<Node>(graph->size()));
    closedset_.resize(graph->size(), 0);
}

BiDijkstra::~BiDijkstra() {
    delete reversed_graph_;
} 

void BiDijkstra::clear() {
    for (int i : workset_) {
        nodes_[0][i].clear();
        nodes_[1][i].clear();
        closedset_[i] = 0;
    }
    workset_.clear();
}

vector<int> BiDijkstra::reconstruct_path(int start, int end) {
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
    assert(middle_point >= 0);

    int p;
    
    // from start to middle 
    p = middle_point;
    vector<int> path = {p};
    while (p != start) {
        p = nodes_[0][p].parent;
        path.push_back(p);
    }
    std::reverse(path.begin(), path.end());

    // from middle to end
    p = middle_point;
    while (p != end) {
        p = nodes_[1][p].parent;
        path.push_back(p);
    }

    return path;
}

void BiDijkstra::visit_node(int side, int node_id, vector<Queue> &openset, double distance, int parent) {
    Node &node = nodes_[side][node_id];
    node.parent = parent;
    node.distance = distance;
    workset_.push_back(node_id);
    openset[side].push({distance, node_id});
}


int BiDijkstra::extract_min(int side, vector<Queue> &openset) {
    Queue &q = openset[side];

    key top;
    while (!q.empty()) {
        top = q.top();
        q.pop();

        if (top.first > nodes_[side][top.second].distance) 
            continue;
        
        return top.second;
    }

    return -1;
}

void BiDijkstra::process_node(int side, int node_id, vector<Queue> &openset, AbsGraph *g) {
    double node_dinstance = nodes_[side][node_id].distance;
    for (auto& [n, cost] : g->get_neighbours(node_id)) {
        Node &neighbour = nodes_[side][n];
        if (neighbour.distance < 0 || neighbour.distance > node_dinstance + cost) {
            visit_node(side, n, openset, node_dinstance + cost, node_id);
        }
    }
}


vector<int> BiDijkstra::find_path(int start, int end) {
    clear();

    vector<Queue> openset(2);  

    visit_node(0, start, openset, 0., start);
    visit_node(1, end, openset, 0., end);

    int node_id;
    while (true) {

        node_id = extract_min(0, openset);
        if (node_id == -1)
            break;

        if (closedset_[node_id])
            return reconstruct_path(start, end);

        closedset_[node_id] = 1;
        process_node(0, node_id, openset, graph);

        node_id = extract_min(1, openset);
        if (node_id == -1)
            break;

        if (closedset_[node_id])
            return reconstruct_path(start, end);

        closedset_[node_id] = 1;
        process_node(1, node_id, openset, reversed_graph_);
    }

    return {};
}

