#include "include/gbs.h"


GBS::GBS(Env* env) : env(env) {
    came_from_.resize(env->size(), -1);
}

void GBS::clear() {
    std::fill(came_from_.begin(), came_from_.end(), -1);
}

vector<int> GBS::reconstruct_path(int start, int end) {
    int p = end;
    vector<int> path = {p};
    while (p != start) {
        p = came_from_[p];
        path.push_back(p);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

vector<int> GBS::find_path(int start, int end) {
    if (start == end)
        return {start};

    clear();

    Queue openset;
    openset.push({0, start});

    while (!openset.empty()) {
        key top = openset.top();
        openset.pop();

        int node_id = top.second;
        for (auto& [n, cost] : env->get_neighbors(node_id)) {
            if (came_from_[n] < 0) {
                came_from_[n] = node_id;
                if (n == end)
                    return reconstruct_path(start, end);
                double potential = env->estimate_distance(n, end);
                openset.push({potential, n});
            }
        }
    }

    return {};
}
