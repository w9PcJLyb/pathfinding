#include "include/hc_a_star.h"


HCAStar::HCAStar(AbsGraph *graph) : graph(graph), st_a_star_(graph) {
}

vector<vector<int>> HCAStar::mapf(vector<int> starts, vector<int> goals) {
    return mapf(starts, goals, 100, nullptr);
}

vector<vector<int>> HCAStar::mapf(
    vector<int> starts,
    vector<int> goals,
    int search_depth,
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
        if (path.empty() || path.back() != goals[i])
            return {};
        paths.push_back(path);
        reservation_table.add_path(0, path, true, edge_collision);
    }

    return paths;
}
