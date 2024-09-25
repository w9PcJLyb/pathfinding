#include "unordered_map"

#include "include/whc_a_star.h"


WHCAStar::WHCAStar(AbsGraph *graph) : graph(graph), st_a_star_(graph) {
}

vector<Path> WHCAStar::mapf(vector<int> starts, vector<int> goals) {
    return mapf(starts, goals, 100, 16, nullptr);
}

vector<Path> WHCAStar::mapf(
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
    }

    return mapf_(agents, search_depth, window_size, reservation_table);
}

vector<Path> WHCAStar::mapf_(
    vector<Agent> &agents,
    int search_depth,
    int window_size,
    ReservationTable &rt
) {
    bool edge_collision = graph->edge_collision();

    priority_queue<pair<int, int>, vector<pair<int, int>>, std::greater<pair<int, int>>> agent_queue;
    for (size_t agent_id = 0; agent_id < agents.size(); agent_id++)
        agent_queue.push({0, agent_id});

    std::unordered_map<int, int> destinations;
    destinations.reserve(agents.size());

    while (!agent_queue.empty()) {
        auto [time, agent_id] = agent_queue.top();
        agent_queue.pop();

        Agent& agent = agents[agent_id];

        int active_window = std::min(window_size, search_depth - time);
        if (active_window <= 0) {
            if (agent.position() == agent.goal) {
                destinations[agent.goal] = agent_id;
                continue;
            }
            break;
        }

        Path path = st_a_star_.find_path(
            time,
            agent.position(),
            agent.goal,
            active_window,
            agent.rrs,
            &rt
        ).first;
        if (path.empty())
            return {};

        if (path.size() == 1) {
            // the agent is at his destination and does not need to move
            agent.add_path(path);
            rt.add_path(time + 1, path, false, edge_collision);
            destinations[agent.goal] = agent_id;
        }
        else {
            if (!destinations.empty()) {
                for (int p : path) {
                    if (!destinations.count(p))
                        continue;
                    // the path disturbed another agent that was at the destination,
                    // so we need to add the another agent to the queue to avoid any collisions
                    int other_agent_id = destinations[p];
                    destinations.erase(p);

                    Agent& other_agent = agents[other_agent_id];
                    int last_action_time = other_agent.path.size() - 1;
                    if (last_action_time > time) {
                        agent_queue.push({last_action_time, other_agent_id});
                    }
                    else {
                        if (time > last_action_time) {
                            Path rest(time - last_action_time, other_agent.goal);
                            other_agent.add_path(rest);
                        }
                        agent_queue.push({time, other_agent_id});
                    }
                }
            }

            agent.path.pop_back();
            agent.add_path(path);
            rt.add_path(time, path, false, edge_collision);
            agent_queue.push({time + path.size() - 1, agent_id});
        }
    }

    if (destinations.size() != agents.size())
        return {};

    vector<Path> paths;
    paths.reserve(agents.size());
    for (auto &agent: agents) {
        Path& path = agent.path;
        while (path.size() > 1 && path.back() == agent.goal && path[path.size() - 2] == agent.goal)
            path.pop_back();

        paths.push_back(path);
    }

    return paths;
}
