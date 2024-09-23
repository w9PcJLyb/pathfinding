#include "include/whc_a_star.h"


WHCAStar::WHCAStar(AbsGraph *graph) : graph(graph), st_a_star_(graph) {
}

vector<vector<int>> WHCAStar::mapf(vector<int> starts, vector<int> goals) {
    return mapf(starts, goals, 100, 16, nullptr);
}

vector<vector<int>> WHCAStar::mapf(
    vector<int> starts,
    vector<int> goals,
    int search_depth,
    int window_size,
    const ReservationTable *rt
) {
    assert(starts.size() == goals.size());

    if (starts.size() == 0)
        return {};

    ReservationTable reservation_table(graph->size());
    if (rt) 
        reservation_table = *rt;

    vector<Agent> agents;
    agents.reserve(starts.size());
    for (size_t agent_id = 0; agent_id < starts.size(); agent_id++) {
        int start = starts[agent_id];
        int goal = goals[agent_id];
        agents.emplace_back(start, goal, st_a_star_.reverse_resumable_search(goal));
        reservation_table.add_vertex_constraint(0, start);
    }

    auto paths = mapf_(agents, search_depth, window_size, reservation_table);

    return paths;
}

vector<vector<int>> WHCAStar::mapf_(
    vector<Agent> &agents,
    int search_depth,
    int window_size,
    ReservationTable &rt
) {
    bool edge_collision = graph->edge_collision();

    int min_search_depth = 0;
    for (auto &agent: agents)
        min_search_depth = std::max(min_search_depth, rt.last_time_reserved(agent.goal));

    int time = 0;
    bool solved = false;
    while (time <= search_depth) {

        if (time >= min_search_depth) {
            solved = true;
            for (auto &agent: agents) {
                if (agent.position(time) != agent.goal) {
                    solved = false;
                    break;
                }
            }

            if (solved)
                break;
        }

        int w = std::min(window_size, search_depth - time);
        if (w == 0)
            break;

        for (size_t agent_id = 0; agent_id < agents.size(); agent_id++) {
            Agent &agent = agents[agent_id];

            if (agent.is_moving(time)) {
                continue;
            }

            vector<int> path = st_a_star_.find_path(
                time,
                agent.position(),
                agent.goal,
                w,
                agent.rrs,
                &rt
            ).first;
            if (path.empty())
                return {};

            if (path.size() == 1) {
                // pause action
                agent.add_path(path);
                rt.add_path(time + 1, path, false, edge_collision);
            }
            else {
                agent.full_path.pop_back();
                agent.add_path(path);
                rt.add_path(time, path, false, edge_collision);
            }
        }

        time++;
    }

    vector<vector<int>> paths;
    if (solved) {
        for (auto &agent: agents) {
            vector<int> path = agent.full_path;
            while (path.size() > 1 && path.back() == agent.goal && path[path.size() - 2] == agent.goal)
                path.pop_back();
            paths.push_back(path);
        }
    }

    return paths;
}
