#include "include/limited_search.h"


LimitedSearch::LimitedSearch(AbsGraph *graph) : graph(graph) {
    reversed_graph_ = graph->reverse();
    bfs_ = new ResumableBFS(graph, 0);
    nodes_.resize(graph->size());
}

LimitedSearch::~LimitedSearch() {
    delete bfs_;
}

void LimitedSearch::clear() {
    for (Node *node : workset_)
        node->clear();
    workset_.clear();
}

vector<int> LimitedSearch::reconstruct_path(int start, int goal) {
    int p = start;
    vector<int> path = {p};
    while (p != goal) {
        p = nodes_[p].parent;
        path.push_back(p);
    }
    return path;
}

vector<int> LimitedSearch::find_path(int start, int goal, int max_steps) {
    bfs_->set_start_node(start);
    if (bfs_->distance(goal) > max_steps)
        return {};

    clear();

    Queue openset;

    openset.push({0, goal});
    nodes_[goal].distance = 0;
    workset_.push_back(&nodes_[goal]);
    double min_w = graph->min_weight();
    
    while (!openset.empty()) {
        key top = openset.top();
        openset.pop();

        int x = top.second; 

        if (x == start) {
            return reconstruct_path(start, goal);
        }

        Node &current = nodes_[x];
        if (current.time + bfs_->distance(x) > max_steps) {
            continue;
        }

        if (top.first > current.f) {
            continue;
        }

        for (auto& [n, cost] : reversed_graph_->get_neighbors(x)) {
            Node &node = nodes_[n];
            double new_distance = current.distance + cost;
            if (node.distance < 0) {
                workset_.push_back(&node);
                node.f = new_distance + bfs_->distance(n) * min_w;
                node.distance = new_distance;
                node.parent = x;
                node.time = current.time + 1;
                openset.push({node.f, n});
            }
            else if (node.distance > new_distance) {
                node.f = node.f - node.distance + new_distance;
                node.distance = new_distance;
                node.parent = x;
                node.time = current.time + 1;
                openset.push({node.f, n});
            }
        }
    }
    return {};
}

vector<int> LimitedSearch::find_path_to_moving_goal(int start, vector<int> goal, int max_steps) {
    vector<int> best_path = {};
    double best_cost = -1; 
    for (int i = 0; i < (int)goal.size(); i++) {
        if (goal[i] < 0)
            continue;
        vector<int> path = find_path(start, goal[i], std::min(i, max_steps));
        if (path.empty())
            continue;
        double cost = graph->calculate_cost(path);
        if (best_cost == -1 || cost < best_cost) {
            best_path = path;
            best_cost = cost;
        }
    }
    return best_path;   
}