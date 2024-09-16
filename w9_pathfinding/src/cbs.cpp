#include <chrono>

#include "include/cbs.h"
#include "unordered_map"
#include "unordered_set"

using std::chrono::duration;
using std::chrono::high_resolution_clock;


CBS::CBS(AbsGraph *graph) : graph(graph), st_a_star_(graph) {
}

vector<vector<int>> CBS::mapf(vector<int> starts, vector<int> goals) {
    return mapf(starts, goals, 100, 1.0, nullptr);
}

pair<vector<int>, double> CBS::low_level(Agent &agent, ReservationTable &rt, int search_depth) {
    return st_a_star_.find_path(0, agent.start, agent.goal, search_depth, agent.rrs, &rt);
}

void CBS::print_node(CTNode &node) {
    cout << "Node: parent=" << node.parent << ", costs=";
    for (auto x : node.costs) {
        cout << x << ",";
    }
    cout << " solutions:" << endl;
    for (auto & path : node.solutions) {
        cout << " - ";
        graph->print_path(path);
    }
}

void CBS::print_conflict(Conflict &r) {
    cout << "Conflict: agent_id=" << r.agent_id << ", time=" << r.time;
    if (r.is_edge_conflict())
        cout << ", edge=" << r.node1 << "->" << r.node2 << endl;
    else
        cout << ", node=" << r.node1 << endl;
}

vector<CBS::Conflict> CBS::find_conflict(vector<vector<int>> &paths) {
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
                        return {
                            Conflict(agent_id, time - 1, p1, p2),
                            Conflict(other_agent_id, time - 1, p2, p1)
                        };
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
            else {
                node_to_agents[paths[agent_id].back()].push_back(agent_id);
            }
        }

        if (end)
            break;

        for (auto &[node_id, agent_ids] : node_to_agents) {
            if (agent_ids.size() > 1) {
                // vertex conflict
                vector<Conflict> conflicts;
                for (int agent_id : agent_ids)
                    conflicts.push_back({agent_id, int(time), node_id});
                return conflicts;
            }
        }

        time++;
    }

    // there are no conflicts
    return {};
}

bool CBS::resolve_conflict(
    CTNode &node, ConstraintTree &tree, Agent &agent, ReservationTable rt, int search_depth
) {
    {
        Conflict &c = node.conflict;
        if (c.is_edge_conflict())
            rt.add_edge_constraint(c.time, c.node1, c.node2);
        else
            rt.add_vertex_constraint(c.time, c.node1);
    }

    int agent_id = node.conflict.agent_id;
    int node_id = node.parent;

    while (node_id > 0) {
        Conflict &c = tree[node_id].conflict;
        if (c.agent_id == agent_id) {
            if (c.is_edge_conflict())
                rt.add_edge_constraint(c.time, c.node1, c.node2);
            else
                rt.add_vertex_constraint(c.time, c.node1);
        }
        node_id = tree[node_id].parent;
    }

    auto [path, cost] = low_level(agent, rt, search_depth);
    if (path.empty() || path.back() != agent.goal)
        return false;

    node.solutions[agent_id] = path;
    node.costs[agent_id] = cost;
    return true;
}

vector<vector<int>> CBS::mapf(
    vector<int> starts,
    vector<int> goals,
    int search_depth,
    double max_time,
    const ReservationTable *rt
) {
    assert(starts.size() == goals.size());

    if (starts.size() == 0)
        return {};

    std::unordered_set<int> goals_set;
    for (int g: goals) {
        if (goals_set.count(g))
            return {};
        goals_set.insert(g);
    }

    vector<Agent> agents;
    agents.reserve(starts.size());
    for (size_t agent_id = 0; agent_id < starts.size(); agent_id++) {
        int start = starts[agent_id];
        int goal = goals[agent_id];
        agents.emplace_back(start, goal, st_a_star_.reverse_resumable_search(goal));
    }

    ReservationTable reservation_table(graph->size());
    if (rt)
        reservation_table = *rt;

    auto paths = mapf_(
        agents,
        search_depth,
        max_time,
        reservation_table
    );

    return paths;
}

vector<vector<int>> CBS::mapf_(
    vector<Agent> &agents,
    int search_depth,
    double max_time,
    ReservationTable &rt
) {
    auto begin_time = high_resolution_clock::now();

    Queue openset;
    ConstraintTree tree;
    tree.reserve(graph->size());

    {
        CTNode root;
        double total_cost = 0;
        for (size_t i = 0; i < agents.size(); i++) {
            auto [path, cost] = low_level(agents[i], rt, search_depth);
            if (path.empty() || path.back() != agents[i].goal) {
                // there is no path from start to goal, or the path length is greater than search_depth
                return {};
            }

            root.solutions.push_back(path);
            root.costs.push_back(cost);
            total_cost += cost;
        }

        tree.push_back(root);
        openset.push({total_cost, tree.size() - 1});
    }

    while (!openset.empty()) {
        auto [cost, node_id] = openset.top();
        openset.pop();


        vector<Conflict> conflicts = find_conflict(tree[node_id].solutions);
        if (conflicts.empty())
            return tree[node_id].solutions;

        if (duration<double>(high_resolution_clock::now() - begin_time).count() > max_time)
            throw timeout_exception("Timeout");

        for (Conflict& conflict : conflicts) {
            int agent_id = conflict.agent_id;

            CTNode new_node(node_id, conflict);
            new_node.costs = tree[node_id].costs;
            new_node.solutions = tree[node_id].solutions;

            if (resolve_conflict(new_node, tree, agents[agent_id], rt, search_depth)) {
                double new_cost = cost - tree[node_id].costs[agent_id] + new_node.costs[agent_id];
                tree.push_back(new_node);
                openset.push({new_cost, tree.size() - 1});
            }
        }

        tree[node_id].costs.clear();
        tree[node_id].solutions.clear();
    }

    return {};
}
