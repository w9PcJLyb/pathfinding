#include "include/dfs.h"


DFS::DFS(AbsGraph *graph) : graph(graph) {
    came_from_.resize(graph->size(), -1);
}

void DFS::clear() {
    std::fill(came_from_.begin(), came_from_.end(), -1);
}

vector<int> DFS::reconstruct_path(int start, int end) {
    int p = end;
    vector<int> path = {p};
    while (p != start) {
        p = came_from_[p];
        path.push_back(p);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

vector<int> DFS::find_path(int start, int end) {
    // there is no guarantee that it will return the shortest path.
    
    if (start == end)
        return {start};

    clear();
    
    vector<int> stack = {start};
    while (!stack.empty()) {
        int x = stack.back();
        stack.pop_back();
        for (auto& [n, cost] : graph->get_neighbors(x)) {
            if (came_from_[n] < 0) {
                came_from_[n] = x;
                if (n == end)
                    return reconstruct_path(start, end);
                stack.push_back(n);
            }
        }
    }
    return {};
}
