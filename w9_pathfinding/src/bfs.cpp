#include "include/bfs.h"


BFS::BFS(Env *env) : env(env) {
    came_from_.resize(env->size(), -1);
}

void BFS::clear() {
    std::fill(came_from_.begin(), came_from_.end(), -1);
}

vector<int> BFS::reconstruct_path(int start, int end) {
    int p = end;
    vector<int> path = {p};
    while (p != start) {
        p = came_from_[p];
        path.push_back(p);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

vector<int> BFS::find_path(int start, int end) {
    if (start == end) 
        return {start};

    clear();
    came_from_[start] = start;

    std::queue<int> queue;
    queue.push(start);

    while (!queue.empty()) {
        int x = queue.front();
        queue.pop();
        for (auto& [n, cost] : env->get_neighbors(x)) {
            if (came_from_[n] < 0) {
                came_from_[n] = x;
                if (n == end)
                    return reconstruct_path(start, end);
                queue.push(n);
            }
        }
    }
    return {};
}
