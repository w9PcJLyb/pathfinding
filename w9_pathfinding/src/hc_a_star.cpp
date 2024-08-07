#include "include/hc_a_star.h"


HCAStar::HCAStar(AbsGraph *graph) : graph(graph), st_a_star_(graph) {
}

void HCAStar::expand_paths(vector<vector<int>> &paths) {
    size_t max_size = paths[0].size();
    for (size_t i = 1; i < paths.size(); i++) {
        max_size = std::max(max_size, paths[i].size());
    }

    for (size_t i = 0; i < paths.size(); i++) {
        if (paths[i].empty()) {
            continue;
        }
        else if (paths[i].size() < max_size) {
            vector<int> path(max_size - paths[i].size(), paths[i].back());
            paths[i].insert(paths[i].end(), path.begin(), path.end());
        }
    }
}

vector<vector<int>> HCAStar::mapf(vector<int> starts, vector<int> goals) {
    return mapf(starts, goals, 100, false, nullptr);
}

vector<vector<int>> HCAStar::mapf(
    vector<int> starts,
    vector<int> goals,
    int search_depth,
    bool despawn_at_destination,
    const ReservationTable *rt
) {
    assert(starts.size() == goals.size());

    if (starts.size() == 0)
        return {};

    ReservationTable reservation_table(graph->size());
    if (rt)
        reservation_table = *rt;

    bool edge_collision = graph->edge_collision();

    vector<vector<int>> paths;
    for (size_t i = 0; i < starts.size(); i++) {
        vector<int> path = st_a_star_.find_path(starts[i], goals[i], search_depth, &reservation_table);
        if (path.empty())
            return {};
        paths.push_back(path);
        reservation_table.add_path(i, 0, path, !despawn_at_destination, edge_collision);
    }

    if (!despawn_at_destination)
        expand_paths(paths);

    return paths;
}
