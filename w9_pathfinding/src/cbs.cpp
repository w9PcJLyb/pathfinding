#include <chrono>

#include "include/cbs.h"
#include "unordered_map"
#include "unordered_set"

using std::chrono::duration;
using std::chrono::high_resolution_clock;


CBS::CBS(AbsGraph *graph) : graph(graph), st_a_star_(graph) {
}

vector<vector<int>> CBS::mapf(vector<int> starts, vector<int> goals) {
    return mapf(starts, goals, 100, 1.0, false, nullptr);
}

pair<vector<int>, double> CBS::low_level(Agent &agent, ReservationTable &rt, int search_depth) {
    return st_a_star_.find_path(0, agent.start, agent.goal, search_depth, agent.rrs, &rt);
}

void CBS::print_node(CTNode &node) {
    cout << "Node: cost=" << node.total_cost << ", solution=";
    size_t t = 0;
    while (true) {
        bool end = true;
        std::string s;
        for (auto &path : node.solutions) {
            if (t < path.size()) {
                s += std::to_string(path[t]) + " ";
                end = false;
            }
            else
                s += "- ";
        }
        if (end)
            break;
        s.pop_back();
        cout << "(" << s << ") ";
        t++;
    }
    cout << " costs: ";
    for (auto x : node.costs) {
        cout << x << " ";
    }
    cout << endl;
}

void CBS::print_conflict(ConflictResult &r) {
    if (!r.has_conflict()) {
        cout << "No conflicts" << endl;
    }
    else if (r.is_edge_conflict()) {
        cout << "Edge conflict at time=" << r.time;
        cout << ", edge=" << r.node1 << "->" << r.node2 << endl;
    }
    else {
        cout << "Vertex conflict at time=" << r.time << ", node=" << r.node1 << endl;
    }
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
                rt.add_edge_constraint(conflict.time, conflict.node1, conflict.node2);
            else
                rt.add_edge_constraint(conflict.time, conflict.node2, conflict.node1);
        }
        else {
            rt.add_vertex_constraint(conflict.time, conflict.node1);
        }

        auto [new_path, new_cost] = low_level(agents[agent_id], rt, search_depth);
        if (new_path.empty() || new_path.back() != agents[agent_id].goal)
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
    double max_time,
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

    auto paths = mapf_(
        agents,
        search_depth,
        max_time,
        despawn_at_destination,
        rt
    );

    for (Agent &agent: agents)
        delete agent.rrs;

    return paths;
}

vector<vector<int>> CBS::mapf_(
    vector<Agent> &agents,
    int search_depth,
    double max_time,
    bool despawn_at_destination,
    const ReservationTable *rt
) {
    auto begin_time = high_resolution_clock::now();

    Queue openset;
    {
        CTNode root;
        for (size_t i = 0; i < agents.size(); i++) {
            ReservationTable reservation_table(graph->size());
            if (rt) 
                reservation_table = *rt;

            auto [path, cost] = low_level(agents[i], reservation_table, search_depth);
            if (cost == -1)
                return {};

            root.constraints.push_back(reservation_table);
            root.solutions.push_back(path);
            root.costs.push_back(cost);
            root.total_cost += cost;
        }
        openset.push(root);
    }

    while (!openset.empty()) {
        CTNode ct_node = openset.top();

        openset.pop();

        ConflictResult result = find_conflict(ct_node.solutions, despawn_at_destination);
        if (!result.has_conflict())
            return ct_node.solutions;

        for (CTNode &new_node : split_node(ct_node, agents, result, search_depth))
            openset.push(new_node);

        if (duration<double>(high_resolution_clock::now() - begin_time).count() > max_time)
            throw timeout_exception("Timeout");
    }

    return {};
}
