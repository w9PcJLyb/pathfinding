#include "include/ida_star.h"


IDAStar::IDAStar(Env* env) : env(env) {
}

vector<int> IDAStar::find_path(int start, int goal) {
    return find_path(start, goal, 10);
}

vector<int> IDAStar::find_path(int start, int goal, double max_distance) {
    goal_ = goal;

    double bound = env->estimate_distance(start, goal);
    vector<int> path = {start};
    while (true) {
        if (bound > max_distance)
            return {};
        double t = search(path, 0, bound);
        if (t == -2)
            return path;
        if (t == -1)
            return {};
        bound = t;
    }
}

double IDAStar::search(vector<int> &path, double g, double bound) {
    int node = path.back();
    double f = g + env->estimate_distance(node, goal_);
    if (f > bound)
        return f;
    if (node == goal_)
        return -2;  // found

    double min = -1; // inf
    for (auto &[succ, cost] : env->get_neighbors(node)) {
        if (std::find(path.begin(), path.end(), succ) == path.end()) {
            path.push_back(succ);
            double t = search(path, g + cost, bound);
            if (t == -2)
                return -2;
            if (min == -1 || (t >= 0 && t < min))
                min = t;
            path.pop_back();  
        }
    }

    return min;
}
