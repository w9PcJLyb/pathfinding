#include "include/cbs.h"
#include "unordered_map"
#include "unordered_set"


CBS::CBS(AbsGraph *graph) : graph(graph), st_a_star_(graph) {
}

vector<vector<int>> CBS::mapf(vector<int> starts, vector<int> goals) {
    return mapf(starts, goals, 100, 100, false, nullptr);
}

pair<vector<int>, double> CBS::low_level(Agent &agent, ReservationTable &rt, int search_depth) {
    return st_a_star_.find_path(0, agent.start, agent.goal, search_depth, agent.rrs, &rt);
}

CBS::ConflictResult CBS::find_conflict(vector<vector<int>> &paths, bool despawn_at_destination) {
    int num_agents = paths.size();

    size_t time = 0;
    std::unordered_map<int, vector<int>> node_to_agents;
    bool edge_collision = graph->edge_collision();
    bool end = false;

    while (true) {
        if (edge_collision && time > 0) {
            for (int agent_id = 0; agent_id < num_agents; agent_id++) {
                if (time >= paths[agent_id].size())
                    continue;

                int p2 = paths[agent_id][time];
                if (!node_to_agents.count(p2))
                    continue;

                int p1 = paths[agent_id][time - 1];
                for (int other_agent_id : node_to_agents[p2]) {
                    if (other_agent_id == agent_id)
                        continue;

                    if (time >= paths[other_agent_id].size())
                        continue;

                    if (p1 == paths[other_agent_id][time]) {
                        // edge conflict
                        return ConflictResult(time - 1, p1, p2, {agent_id, other_agent_id});
                    }
                }
            }
        } 

        node_to_agents.clear();

        end = true;
        for (int agent_id = 0; agent_id < num_agents; agent_id++) {
            if (time < paths[agent_id].size()) {
                node_to_agents[paths[agent_id][time]].push_back(agent_id);
                end = false;
            }
            else if (!despawn_at_destination) {
                node_to_agents[paths[agent_id].back()].push_back(agent_id);
            }
        }

        if (end)
            break;

        for (auto &[node_id, agent_ids] : node_to_agents) {
            if (agent_ids.size() > 1) {
                // vertex conflict
                return ConflictResult(time, node_id, agent_ids);
            }
        }

        time++;
    }

    // there are no conflicts
    return ConflictResult();
}

void CBS::expand_paths(vector<vector<int>> &paths) {
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

vector<CBS::CTNode> CBS::split_node(
    CTNode &ct_node, vector<Agent> &agents, ConflictResult &conflict, int search_depth
) {
    vector<CTNode> nodes;

    for (size_t i = 0; i < conflict.agent_ids.size(); i++) {
        int agent_id = conflict.agent_ids[i];

        CTNode new_node = ct_node;
        ReservationTable& rt = new_node.constraints[agent_id];

        if (conflict.is_edge_conflict()) {
            if (i == 0)
                rt.add_edge_constraint(
                    0, conflict.time, conflict.node1, conflict.node2
                );
            else
                rt.add_edge_constraint(
                    0, conflict.time, conflict.node2, conflict.node1
                );
        }
        else {
            rt.add_vertex_constraint(0, conflict.time, conflict.node1);
        }

        auto [new_path, new_cost] = low_level(agents[agent_id], rt, search_depth);
        if (new_cost == -1)
            continue;

        new_node.solutions[agent_id] = new_path;
        new_node.total_cost = new_node.total_cost + new_cost - new_node.costs[agent_id];
        new_node.costs[agent_id] = new_cost;

        nodes.push_back(new_node);
    }
    return nodes;
}

vector<vector<int>> CBS::mapf(
    vector<int> starts,
    vector<int> goals,
    int search_depth,
    int max_iter,
    bool despawn_at_destination,
    const ReservationTable *rt
) {
    assert(starts.size() == goals.size());

    if (starts.size() == 0)
        return {};

    if (!despawn_at_destination) {
        std::unordered_set<int> goals_set;
        for (int g: goals) {
            if (goals_set.count(g))
                return {};
            goals_set.insert(g);
        }
    }

    vector<Agent> agents;
    for (size_t agent_id = 0; agent_id < starts.size(); agent_id++) {
        int start = starts[agent_id];
        int goal = goals[agent_id];
        agents.push_back({start, goal, st_a_star_.reverse_resumable_search(goal)});
    }

    Queue openset;
    {
        CTNode root;
        for (size_t i = 0; i < agents.size(); i++) {
            ReservationTable reservation_table(graph->size());
            if (rt) 
                reservation_table = *rt;

            auto [path, cost] = low_level(agents[i], reservation_table, search_depth);
            if (cost == -1) {
                for (Agent &agent: agents)
                    delete agent.rrs;
                return {};
            }
            root.constraints.push_back(reservation_table);
            root.solutions.push_back(path);
            root.costs.push_back(cost);
            root.total_cost += cost;
        }
        openset.push(root);
    }

    int iter = 0;
    while (!openset.empty()) {
        CTNode ct_node = openset.top();

        openset.pop();

        ConflictResult result = find_conflict(ct_node.solutions, despawn_at_destination);
        if (!result.has_conflict()) {
            if (!despawn_at_destination)
                expand_paths(ct_node.solutions);
            for (Agent &agent: agents)
                delete agent.rrs;
            return ct_node.solutions;
        }

        for (CTNode &new_node : split_node(ct_node, agents, result, search_depth)) {
            openset.push(new_node);
        }

        iter++;
        if (iter >= max_iter)
            break;
    }

    for (Agent &agent: agents)
        delete agent.rrs;

    return {};
}
