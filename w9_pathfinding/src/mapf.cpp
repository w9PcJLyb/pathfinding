#include "include/mapf.h"


void ensure_path_length(Path& path, int length) {
    if ((int)path.size() >= length + 1)
        return;

    vector<int> rest(length + 1 - path.size(), path.back());
    path.insert(path.end(), rest.begin(), rest.end());
}


bool has_collision(const vector<int>& positions, const vector<int>& next_positions, bool edge_collision) {
    std::unordered_map<int, int> node_to_agent;
    for (size_t i = 0; i < next_positions.size(); i++) {
        int node_id = next_positions[i];
        if (node_to_agent.count(node_id)) {
            // vertex conflict
            return true;
        }
        else
            node_to_agent[node_id] = i;
    }

    if (edge_collision) {
        for (size_t i = 0; i < positions.size(); i++) {
            int p = positions[i];
            int next_p = next_positions[i];
            if (p == next_p)
                continue;
            if (node_to_agent.count(p) && positions[node_to_agent[p]] == next_p) {
                // edge conflict
                return true;
            }
        }
    }

    return false;
}
