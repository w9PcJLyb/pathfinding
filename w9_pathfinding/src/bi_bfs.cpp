#include "include/bi_bfs.h" 


BiBFS::BiBFS(AbsGraph *graph) : graph(graph) {
    reversed_graph_ = graph->reverse();
    forward_nodes_.resize(graph->size());
    backward_nodes_.resize(graph->size());
    closedset_.resize(graph->size(), 0);
}

BiBFS::~BiBFS() {
    delete reversed_graph_;
} 

void BiBFS::clear() {
    for (int i : workset_) {
        forward_nodes_[i].clear();
        backward_nodes_[i].clear();
        closedset_[i] = 0;
    }
    workset_.clear();
}

vector<int> BiBFS::reconstruct_path(int start, int end) {
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


bool BiBFS::step(std::queue<int> &queue, vector<Node> &nodes, AbsGraph* g) {
    if (queue.empty())
        return false;

    int node_id = queue.front();
    if (closedset_[node_id])
        return false;
    
    closedset_[node_id] = 1;
    queue.pop();
    int d = nodes[node_id].distance + 1;

    for (auto& [n, cost] : g->get_neighbors(node_id)) {
        Node &nb = nodes[n];
        if (nb.distance < 0) {
            nb.distance = d;
            nb.parent = node_id;
            queue.push(n);
            workset_.push_back(n);
        }
    }

    return true;
}


vector<int> BiBFS::find_path(int start, int end) {
    clear();

    std::queue<int> forward_queue, backward_queue;
    forward_queue.push(start);
    backward_queue.push(end);

    forward_nodes_[start].parent = start;
    forward_nodes_[start].distance = 0;
    backward_nodes_[end].parent = end;
    backward_nodes_[end].distance = 0;

    workset_.push_back(start);
    workset_.push_back(end);

    while (
        step(forward_queue, forward_nodes_, graph) 
        && step(backward_queue, backward_nodes_, reversed_graph_)
    ) {
    }

    return reconstruct_path(start, end);
}
