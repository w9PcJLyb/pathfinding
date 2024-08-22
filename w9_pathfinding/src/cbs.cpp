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
    cout << "Node: solution=";
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

pair<vector<int>, CBS::ConflictResult> CBS::find_conflict(vector<vector<int>> &paths, bool despawn_at_destination) {
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
                        return {{agent_id, other_agent_id}, ConflictResult(time - 1, p1, p2)};
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
                return {agent_ids, ConflictResult(time, node_id)};
            }
        }

        time++;
    }

    // there are no conflicts
    return {{}, ConflictResult()};
}

bool CBS::resolve_conflict(
    CTNode *node, Agent &agent, ReservationTable rt, int search_depth
) {
    int agent_id = node->agent_id;

    CTNode *node_ = node;
    while (node_) {
        if (node_->agent_id == agent_id) {
            ConflictResult &c = node_->conflict;
            if (c.is_edge_conflict())
                rt.add_edge_constraint(c.time, c.node1, c.node2);
            else
                rt.add_vertex_constraint(c.time, c.node1);
        }
        node_ = node_->parent;
    }

    auto [path, cost] = low_level(agent, rt, search_depth);
    if (path.empty() || path.back() != agent.goal)
        return false;

    node->solutions[agent_id] = path;
    node->costs[agent_id] = cost;
    return true;
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

    ReservationTable reservation_table(graph->size());
    if (rt)
        reservation_table = *rt;

    auto paths = mapf_(
        agents,
        search_depth,
        max_time,
        despawn_at_destination,
        reservation_table
    );

    for (Agent &agent: agents)
        delete agent.rrs;

    return paths;
}

void CBS::release_nodes(vector<CTNode*> nodes)
{
    for (CTNode *node : nodes)
        delete node;
    nodes.clear();
}

vector<vector<int>> CBS::mapf_(
    vector<Agent> &agents,
    int search_depth,
    double max_time,
    bool despawn_at_destination,
    ReservationTable &rt
) {
    auto begin_time = high_resolution_clock::now();

    Queue openset;
    vector<CTNode*> workset;
    {
        CTNode* root = new CTNode();
        double total_cost = 0;
        for (size_t i = 0; i < agents.size(); i++) {
            auto [path, cost] = low_level(agents[i], rt, search_depth);
            if (cost == -1)
                return {};

            root->solutions.push_back(path);
            root->costs.push_back(cost);
            total_cost += cost;
        }
        openset.push({total_cost, root});
        workset.push_back(root);
    }

    while (!openset.empty()) {
        auto [cost, node] = openset.top();

        openset.pop();

        auto [conflicting_agents, conflict] = find_conflict(node->solutions, despawn_at_destination);
        if (conflicting_agents.empty()) {
            auto paths = node->solutions;
            release_nodes(workset);
            return paths;
        }

        if (duration<double>(high_resolution_clock::now() - begin_time).count() > max_time) {
            release_nodes(workset);
            throw timeout_exception("Timeout");
        }

        for (size_t i = 0; i < conflicting_agents.size(); i++) {
            int agent_id = conflicting_agents[i];

            CTNode* new_node = new CTNode(node, agent_id, conflict);
            if (i > 0 && conflict.is_edge_conflict())
                std::swap(new_node->conflict.node1, new_node->conflict.node2);

            if (resolve_conflict(new_node, agents[agent_id], rt, search_depth)) {
                double new_cost = cost - node->costs[agent_id] + new_node->costs[agent_id];
                openset.push({new_cost, new_node});
                workset.push_back(new_node);
            }
            else
                delete new_node;
        }

        node->solutions.clear();
        node->costs.clear();
    }

    release_nodes(workset);
    return {};
}
