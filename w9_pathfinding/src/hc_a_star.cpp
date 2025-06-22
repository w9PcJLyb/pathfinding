#include "include/hc_a_star.h"


HCAStar::HCAStar(Env* env) : env(env), st_a_star_(env) {
}

vector<vector<int>> HCAStar::mapf(vector<int> starts, vector<int> goals) {
    return mapf(starts, goals, 100, nullptr);
}

vector<vector<int>> HCAStar::mapf(
    vector<int> starts,
    vector<int> goals,
    int max_length,
    const ReservationTable *rt
) {
    assert(starts.size() == goals.size());

    if (starts.size() == 0)
        return {};

    ReservationTable reservation_table(env->size());
    if (rt)
        reservation_table = *rt;

    bool edge_collision = env->edge_collision();

    vector<vector<int>> paths;
    for (size_t i = 0; i < starts.size(); i++) {
        int goal = goals[i];
        vector<int> path = st_a_star_.find_path_with_length_limit(
            starts[i],
            goal,
            max_length,
            &reservation_table,
            nullptr,
            reservation_table.last_time_reserved(goal)
        );
        if (path.empty())
            return {};
        paths.push_back(path);
        reservation_table.add_path(0, path, true, edge_collision);
    }

    return paths;
}
