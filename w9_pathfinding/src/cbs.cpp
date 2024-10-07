#include <chrono>

#include "include/cbs.h"
#include "unordered_map"
#include "unordered_set"

using std::chrono::duration;
using std::chrono::high_resolution_clock;


CBS::CBS(AbsGraph *graph) : graph(graph), st_a_star_(graph), generator_(std::random_device{}()) {
}

vector<Path> CBS::mapf(vector<int> starts, vector<int> goals) {
    return mapf(starts, goals, 100, 1.0, true, nullptr);
}

void CBS::print_node(CTNode &node) {
    cout << "Node: parent=" << node.parent << ", costs=";
    for (auto x : node.costs)
        cout << x << ", ";
    cout << "total=" << node.total_cost() << ", solutions:" << endl;
    for (auto & path : node.solutions) {
        cout << " - ";
        graph->print_path(path);
    }
}

void CBS::print_constraint(Constraint &constraint) {
    Conflict& conflict = constraint.conflict;
    std::string type = "Negative constraint";
    if (constraint.is_positive)
        type = "Positive constraint";
    cout << type << ": agent_id=" << constraint.agent_id;
    cout << ", time=" << conflict.time;
    if (conflict.is_edge_conflict()) {
        cout << ", edge=" << graph->node_to_string(conflict.node1);
        cout << "->" << graph->node_to_string(conflict.node2) << endl;
    }
    else
        cout << ", node=" << graph->node_to_string(conflict.node1) << endl;
}

int CBS::random_int(int max_value) {
    std::uniform_int_distribution<> distribution(0, max_value - 1);
    return distribution(generator_);
}

bool CBS::is_point_at_time(Path& path, int point, int time) {
    if ((int)path.size() > time)
        return path[time] == point;
    return path.back() == point;
}

vector<CBS::Constraint> CBS::find_conflict(vector<Path> &paths, bool find_random, bool disjoint_splitting) {
    int num_agents = paths.size();
    size_t time = 0;
    std::unordered_map<int, vector<int>> node_to_agents;
    bool edge_collision = graph->edge_collision();
    bool end = false;

    vector<pair<vector<int>, Conflict>> conflicts;  // [{conflicting agents, Conflict}, ...]

    while (true) {
        if (edge_collision && time > 0) {
            for (int agent_id = 0; agent_id < num_agents; agent_id++) {
                if (time >= paths[agent_id].size())
                    continue;

                int p2 = paths[agent_id][time];
                if (!node_to_agents.count(p2))
                    continue;

                int p1 = paths[agent_id][time - 1];
                if (p1 == p2)
                    continue;

                for (int other_agent_id : node_to_agents[p2]) {
                    if (other_agent_id == agent_id)
                        continue;

                    if (time >= paths[other_agent_id].size())
                        continue;

                    if (p1 == paths[other_agent_id][time]) {
                        // edge conflict
                        Conflict conflict(time - 1, p1, p2);
                        vector<int> agent_ids = {agent_id, other_agent_id};

                        if (!find_random)
                            return split_conflict(paths, agent_ids, conflict, disjoint_splitting);

                        conflicts.emplace_back(agent_ids, conflict);
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
                Conflict conflict(int(time), node_id);

                if (!find_random)
                    return split_conflict(paths, agent_ids, conflict, disjoint_splitting);

                conflicts.emplace_back(agent_ids, conflict);
            }
        }

        time++;
    }

    if (conflicts.empty())
        return {};

    auto& [agent_ids, random_conflict] = conflicts[random_int(conflicts.size())];

    return split_conflict(paths, agent_ids, random_conflict, disjoint_splitting);
}

vector<CBS::Constraint> CBS::split_conflict(
    vector<Path> &paths,
    vector<int> &agent_ids,
    Conflict& conflict,
    bool disjoint_splitting
) {
    vector<Constraint> constraints;

    if (!disjoint_splitting) {
        constraints.reserve(agent_ids.size());

        if (conflict.is_edge_conflict()) {
            Conflict reversed_conflict(conflict.time, conflict.node2, conflict.node1);
            constraints.emplace_back(agent_ids[0], conflict);
            constraints.emplace_back(agent_ids[1], reversed_conflict);
        }
        else {
            for (int agent_id : agent_ids)
                constraints.emplace_back(agent_id, conflict);
        }
    }
    else {
        int i = random_int(agent_ids.size());
        int agent_id = agent_ids[i];
        if (i == 1 && conflict.is_edge_conflict())
            std::swap(conflict.node1, conflict.node2);

        vector<int> conflicting_agents; // this is necessary in case of a positive constraint
        if (conflict.is_vertex_conflict()) {
            for (int a : agent_ids) {
                if (a != agent_id)
                    conflicting_agents.push_back(a);
            }
        }
        else {
            conflicting_agents.push_back(agent_ids[1 - i]);

            // we should find all agents that have a conflict with node1 or node2
            for (int a = 0; a < (int)paths.size(); a++) {
                if (a == agent_id)
                    continue;

                if (
                    is_point_at_time(paths[a], conflict.node1, conflict.time)
                    || is_point_at_time(paths[a], conflict.node2, conflict.time + 1)
                )
                    conflicting_agents.push_back(a);
            }
        }

        // negative constraint
        constraints.emplace_back(agent_id, conflict, false);

        // positive constraint
        constraints.emplace_back(agent_id, conflict, true);
        constraints.back().conflicting_agents = conflicting_agents;
    }

    return constraints;
}

void CBS::add_constraint(ReservationTable& rt, Conflict& c, bool reverse) {
    if (!reverse) {
        if (c.is_edge_conflict())
            rt.add_edge_constraint(c.time, c.node1, c.node2);
        else
            rt.add_vertex_constraint(c.time, c.node1);
    }
    else {
        if (c.is_edge_conflict()) {
            rt.add_vertex_constraint(c.time, c.node1);
            rt.add_vertex_constraint(c.time + 1, c.node2);
            rt.add_edge_constraint(c.time, c.node2, c.node1);
        }
        else
            rt.add_vertex_constraint(c.time, c.node1);
    }
}

vector<int> CBS::populate_reservation_table(
    ReservationTable& rt,
    CTNode& node,
    ConstraintTree& tree,
    int agent_id
) {
    add_constraint(rt, node.constraint.conflict, node.constraint.is_positive);

    vector<int> landmarks;  // list of times with positive constraint
    int node_id = node.parent;
    while (node_id > 0) {
        Constraint& constraint = tree[node_id].constraint;
        Conflict& conf = constraint.conflict;

        if (constraint.agent_id == agent_id && constraint.is_positive) {
            landmarks.push_back(conf.time);
            if (conf.is_edge_conflict())
                landmarks.push_back(conf.time + 1);
        }
        else if (constraint.agent_id == agent_id || constraint.is_positive)
            add_constraint(rt, conf, constraint.is_positive);

        node_id = tree[node_id].parent;
    }

    return landmarks;
}

bool CBS::low_level(
    CTNode& node,
    ConstraintTree& tree,
    Agent& agent,
    ReservationTable& rt,
    int max_length
) {
    int agent_id = node.constraint.agent_id;

    ReservationTable rt_ = rt;
    populate_reservation_table(rt_, node, tree, agent_id);

    Path path = st_a_star_.find_path_with_length_limit(
        agent.start,
        agent.goal,
        max_length,
        &rt_,
        agent.rrs,
        rt_.last_time_reserved(agent.goal)
    );
    if (path.empty())
        return false;

    node.solutions[agent_id] = path;
    node.costs[agent_id] = graph->calculate_cost(path);
    return true;
}

bool CBS::low_level_with_disjoint_splitting(
    CTNode& node,
    ConstraintTree& tree,
    vector<Agent>& agents,
    ReservationTable& rt,
    int max_length
) {
    int agent_id = node.constraint.agent_id;
    Constraint& constraint = node.constraint;

    if (!constraint.is_positive) {
        Path path = find_new_path(node, tree, agent_id, agents[agent_id], rt, max_length);
        if (path.empty())
            return false;
        node.solutions[agent_id] = path;
        node.costs[agent_id] = graph->calculate_cost(path);
    }
    else {
        // a positive constraint, we run the low-level search for every agent
        // whose path violates its corresponding negative constraint
        for (int other_agent : node.constraint.conflicting_agents) {
            Path path = find_new_path(node, tree, other_agent, agents[other_agent], rt, max_length);
            if (path.empty())
                return false;
            node.solutions[other_agent] = path;
            node.costs[other_agent] = graph->calculate_cost(path);
        }
    }

    return true;
}

Path CBS::find_new_path(
    CTNode& node,
    ConstraintTree& tree,
    int agent_id,
    Agent& agent,
    ReservationTable& rt,
    int max_length
) {
    ReservationTable rt_ = rt;
    vector<int> landmarks = populate_reservation_table(rt_, node, tree, agent_id);

    int conflict_time = node.constraint.conflict.time;
    int left_landmark = -1, right_landmark = -1;
    for (int time : landmarks) {
        if (time <= conflict_time)
            left_landmark = std::max(time, left_landmark);
        else
            right_landmark = right_landmark == -1 ? time : std::min(time, right_landmark);
    }

    if (node.constraint.conflict.is_vertex_conflict())
        assert(conflict_time != left_landmark);

    Path path = node.solutions[agent_id];

    if (right_landmark >= (int)path.size() - 1) {
        // right_landmark is located at the agent's goal position,
        // need to update the last part of the path with a length restriction
        left_landmark = std::max(0, left_landmark);
        ensure_path_length(path, left_landmark);

        Path part = st_a_star_.find_path_with_length_limit(
            path[left_landmark],
            agent.goal,
            right_landmark - left_landmark,
            &rt_,
            agent.rrs,
            rt_.last_time_reserved(agent.goal),
            left_landmark
        );
        if (part.empty())
            return {};

        path.resize(left_landmark);
        path.insert(path.end(), part.begin(), part.end());

        return path;
    }

    ensure_path_length(path, std::max(left_landmark, right_landmark));

    if (left_landmark == -1 && right_landmark == -1) {
        // there are no landmarks, need to find a completely new path
        path = st_a_star_.find_path_with_length_limit(
            agent.start,
            agent.goal,
            max_length,
            &rt_,
            agent.rrs,
            rt_.last_time_reserved(agent.goal)
        );
        if (path.empty())
            return {};
    }
    else if (left_landmark == -1 && right_landmark >= 0) {
        // need to update the first part of the path
        Path part = st_a_star_.find_path_with_exact_length(
            agent.start, path[right_landmark], right_landmark, &rt_
        );
        if (part.empty())
            return {};
        std::copy(part.begin(), part.end(), path.begin());
    }
    else if (left_landmark >= 0 && right_landmark == -1) {
        // need to update the last part of the path
        Path part = st_a_star_.find_path_with_length_limit(
            path[left_landmark],
            agent.goal,
            max_length - left_landmark,
            &rt_,
            agent.rrs,
            rt_.last_time_reserved(agent.goal),
            left_landmark
        );
        if (part.empty())
            return {};
        path.resize(left_landmark);
        path.insert(path.end(), part.begin(), part.end());
    }
    else {
        // need to update the middle part of the path
        Path part = st_a_star_.find_path_with_exact_length(
            path[left_landmark],
            path[right_landmark],
            right_landmark - left_landmark,
            &rt_,
            left_landmark
        );
        if (part.empty())
            return {};
        std::copy(part.begin(), part.end(), path.begin() + left_landmark);
    }

    while (path.size() > 1 && path.back() == path.at(path.size() - 2))
        path.pop_back();

    return path;
}

vector<Path> CBS::mapf(
    vector<int> starts,
    vector<int> goals,
    int max_length,
    double max_time,
    bool disjoint_splitting,
    const ReservationTable *rt
) {
    num_closed_nodes = 0;
    num_generated_nodes = 0;

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
        max_length,
        max_time,
        disjoint_splitting,
        reservation_table
    );

    return paths;
}

vector<Path> CBS::mapf_(
    vector<Agent> &agents,
    int max_length,
    double max_time,
    bool disjoint_splitting,
    ReservationTable &rt
) {
    auto begin_time = high_resolution_clock::now();

    Queue openset;
    ConstraintTree tree;
    tree.reserve(graph->size());

    {
        CTNode root;
        for (size_t i = 0; i < agents.size(); i++) {
            Agent& agent = agents[i];
            Path path = st_a_star_.find_path_with_length_limit(
                agent.start,
                agent.goal,
                max_length,
                &rt,
                agent.rrs,
                rt.last_time_reserved(agent.goal)
            );
            if (path.empty())
                return {};

            root.solutions.push_back(path);
            double cost = graph->calculate_cost(path);
            root.costs.push_back(cost);
        }

        tree.push_back(root);
        num_generated_nodes++;
        openset.push({root.total_cost(), tree.size() - 1});
    }

    while (!openset.empty()) {
        auto [cost, node_id] = openset.top();
        openset.pop();

        vector<Constraint> constraints = find_conflict(
            tree[node_id].solutions, disjoint_splitting, disjoint_splitting
        );
        if (constraints.empty()) {
            num_closed_nodes++;
            return tree[node_id].solutions;
        }

        if (duration<double>(high_resolution_clock::now() - begin_time).count() > max_time)
            throw timeout_exception("Timeout");

        for (Constraint& constraint : constraints) {
            int agent_id = constraint.agent_id;

            CTNode new_node(node_id, constraint);
            new_node.costs = tree[node_id].costs;
            new_node.solutions = tree[node_id].solutions;

            bool resolved;
            if (!disjoint_splitting)
                resolved = low_level(new_node, tree, agents[agent_id], rt, max_length);
            else
                resolved = low_level_with_disjoint_splitting(new_node, tree, agents, rt, max_length);

            if (resolved) {
                tree.push_back(new_node);
                num_generated_nodes++;
                openset.push({new_node.total_cost(), tree.size() - 1});
            }
        }

        tree[node_id].costs.clear();
        tree[node_id].solutions.clear();
        num_closed_nodes++;
    }

    return {};
}
