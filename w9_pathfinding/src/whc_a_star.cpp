#include "include/whc_a_star.h"


WHCAStar::WHCAStar(AbsGraph *graph) : graph(graph), st_a_star_(graph) {
}

vector<vector<int>> WHCAStar::mapf(vector<int> starts, vector<int> goals) {
    return mapf(starts, goals, 100, 16, false, nullptr);
}

vector<vector<int>> WHCAStar::mapf(
    vector<int> starts,
    vector<int> goals,
    int search_depth,
    int window_size,
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

    vector<Agent> agents;
    for (size_t agent_id = 0; agent_id < starts.size(); agent_id++) {
        int start = starts[agent_id];
        int goal = goals[agent_id];
        agents.push_back({start, goal, st_a_star_.reverse_resumable_search(goal)});
        reservation_table.add_vertex_constraint(agent_id, 0, start);
    }

    int time = 0;
    while (time <= search_depth) {

        bool finished = true;
        for (auto &agent: agents) {
            if (agent.active && agent.position(time) != agent.goal) {
                finished = false;
                break;   
            }
        }

        if (finished) {
            break;
        }

        int w = std::min(window_size, search_depth - time);

        for (size_t agent_id = 0; agent_id < starts.size(); agent_id++) {
            Agent &agent = agents[agent_id];

            if (!agent.active || agent.is_moving(time)) {
                continue;
            }

            vector<int> path = st_a_star_.find_path(
                time,
                agent.position(),
                agent.goal,
                w,
                agent.rrs,
                &reservation_table
            );
            if (path.empty()) {
                agent.active = false;
                continue;
            }

            if (path.size() == 1) {
                // pause action
                agent.add_path(path);
                reservation_table.add_path(agent_id, time + 1, path, false, edge_collision);
            }
            else {
                agent.full_path.pop_back();
                agent.add_path(path);
                reservation_table.add_path(agent_id, time, path, false, edge_collision);
            }

            if (despawn_at_destination && agent.position() == agent.goal)
                agent.active = false;
        }

        time++;
    }

    vector<vector<int>> paths;
    for (auto &agent: agents) {
        paths.push_back(agent.full_path);
        delete agent.rrs;
    }

    return paths;
}
