#include <chrono>

#include "include/multi_agent_a_star.h"

using std::chrono::duration;
using std::chrono::high_resolution_clock;


maas::State::State(AbsGraph* graph, int node_id, Tree& tree, vector<int>& goal, int time, ReservationTable &rt) {
    vector<int>& positions = tree[node_id].positions;

    neighbors_.resize(positions.size());
    next_positions_.resize(positions.size(), 0);
    ids_.resize(positions.size(), 0);
    ids_.back() = -1;

    double pause_action_cost = graph->get_pause_action_cost();
    for (size_t i = 0; i < positions.size(); i++) {
        int p = positions[i];

        if (p == goal[i]) {
            int waiting_time = 0;
            int node_id_ = tree[node_id].parent;
            while (node_id_ >= 0 && tree[node_id_].positions[i] == p) {
                waiting_time++;
                node_id_ = tree[node_id_].parent;
            }

            if (!rt.is_reserved(time + 1, p))
                neighbors_[i].push_back({p, 0});

            auto reserved_edges = rt.get_reserved_edges(time, p);
            for (auto &[n, cost] : graph->get_neighbors(p)) {
                if (!reserved_edges.count(n) && !rt.is_reserved(time + 1, n))
                    neighbors_[i].push_back({n, cost + waiting_time * pause_action_cost});
            }
        }
        else {
            if (!rt.is_reserved(time + 1, p))
                neighbors_[i].push_back({p, pause_action_cost});

            auto reserved_edges = rt.get_reserved_edges(time, p);
            for (auto &[n, cost] : graph->get_neighbors(p)) {
                if (!reserved_edges.count(n) && !rt.is_reserved(time + 1, n))
                    neighbors_[i].push_back({n, cost});
            }
        }
    }
}

bool maas::State::update_indexes_() {
    int i = ids_.size() - 1;
    while (i >= 0) {
        if (ids_[i] < (int)neighbors_[i].size() - 1) {
            ids_[i]++;
            return true;
        }
        else {
            ids_[i] = 0;
            i--;
        }
    }

    return false;
}

pair<vector<int>, double> maas::State::next() {
    if (!update_indexes_())
        return {{}, 0};

    double cost = 0;
    for (size_t i = 0; i < ids_.size(); i++) {
        auto [p, c] = neighbors_[i][ids_[i]];
        next_positions_[i] = p;
        cost += c;
    }

    return {next_positions_, cost};
}

bool maas::State::allowed(vector<int>& positions, vector<int>& next_positions, bool edge_collision) {
    std::unordered_map<int, int> node_to_agent;
    for (size_t i = 0; i < next_positions.size(); i++) {
        int node_id = next_positions[i];
        if (node_to_agent.count(node_id))
            return false;
        else
            node_to_agent[node_id] = i;
    }

    if (edge_collision) {
        for (size_t i = 0; i < positions.size(); i++) {
            int p = positions[i];
            int next_p = next_positions[i];
            if (p == next_p)
                continue;
            if (node_to_agent.count(p) && positions[node_to_agent[p]] == next_p)
                return false;
        }
    }

    return true;
}

maas::MultiAgentAStar::MultiAgentAStar(AbsGraph *graph) : graph(graph), st_a_star_(graph) {
}

vector<vector<int>> maas::MultiAgentAStar::mapf(vector<int> starts, vector<int> goals) {
    return mapf(starts, goals, 1.0, nullptr);
}

vector<vector<int>> maas::MultiAgentAStar::mapf(vector<int> starts, vector<int> goals, double max_time, const ReservationTable *rt) {
    assert(starts.size() == goals.size());

    if (starts.size() == 0)
        return {};

    vector<Agent> agents;
    for (size_t agent_id = 0; agent_id < starts.size(); agent_id++) {
        int start = starts[agent_id];
        int goal = goals[agent_id];
        agents.push_back({start, goal, st_a_star_.reverse_resumable_search(goal)});
        if (agents.back().rrs->distance(start) == -1)
            return {};
    }

    ReservationTable reservation_table(graph->size());
    if (rt)
        reservation_table = *rt;

    auto paths = mapf_(agents, max_time, reservation_table);

    if (paths.size() == agents.size()) {
        for (size_t i = 0; i < agents.size(); i++) {
            vector<int>& path = paths[i];
            int goal = agents[i].goal;
            while (path.size() > 1 && path.back() == goal && path[path.size() - 2] == goal)
                path.pop_back();
        }
    }

    for (Agent &agent: agents)
        delete agent.rrs;

    return paths;
}

vector<vector<int>> maas::MultiAgentAStar::reconstruct_paths(int node_id, Tree& tree) {
    vector<vector<int>> paths(tree[node_id].positions.size(), vector<int>(0));
    while (node_id >= 0) {
        Node& node = tree[node_id];
        for (size_t i = 0; i < paths.size(); i++) {
            paths[i].push_back(node.positions[i]);
        }
        node_id = node.parent;
    }
    for (vector<int>& path : paths) {
        std::reverse(path.begin(), path.end());
    }
    return paths;
}

void maas::MultiAgentAStar::print_node(Node& node) {
    cout << "Node: parent=" << node.parent << " distance=" << node.distance;
    cout << " time=" << node.time << " positions=";
    for (int x : node.positions) {
        cout << x << " ";
    }
    cout << endl;
}

std::string maas::MultiAgentAStar::positions_to_string(vector<int>& positions) {
    std::string s;
    for (int p : positions)
        s += std::to_string(p) + "_";
    return s;
}

double maas::MultiAgentAStar::heuristic(vector<int>& positions, vector<Agent> &agents) {
    double h = 0;
    for (size_t i = 0; i < agents.size(); i++) {
        double min_distance = agents[i].rrs->distance(positions[i]);
        if (min_distance < 0)  // unreachable
            return -1;
        h += min_distance;
    }
    return h;
}

vector<vector<int>> maas::MultiAgentAStar::mapf_(vector<Agent> &agents, double max_time, ReservationTable &rt) {
    auto begin_time = high_resolution_clock::now();

    Queue openset;
    Tree tree;
    tree.reserve(graph->size());
    std::unordered_map<std::string, double> node_to_distance;
    vector<int> goal;
    bool edge_collision = graph->edge_collision();

    {
        vector<int> start;
        for (Agent& agent : agents) {
            start.push_back(agent.start);
            goal.push_back(agent.goal);
        }

        Node root(-1, 0, 0, start);

        tree.push_back(root);
        openset.push({0, tree.size() - 1});
        node_to_distance[positions_to_string(root.positions)] = 0;
    }

    while (!openset.empty()) {
        auto [f, node_id] = openset.top();
        openset.pop();
 
        Node node = tree[node_id];

        if (node.positions == goal)
            return reconstruct_paths(node_id, tree);

        State state(graph, node_id, tree, goal, node.time, rt);

        int counter = 0;
        while (true) {
            if (counter % 1000 == 0) {
                if (duration<double>(high_resolution_clock::now() - begin_time).count() > max_time)
                    throw timeout_exception("Timeout");
            }

            counter++;

            auto [new_positions, cost] = state.next();
            if (new_positions.empty())
                break;

            std::string key = positions_to_string(new_positions);
            if (node_to_distance.count(key) && node.distance + cost >= node_to_distance[key])
                continue;

            if (!state.allowed(node.positions, new_positions, edge_collision))
                continue;

            double h = heuristic(new_positions, agents);
            if (h < 0)
                continue;

            Node new_node(node_id, node.time + 1, node.distance + cost, new_positions);
            tree.push_back(new_node);
            openset.push({new_node.distance + h, tree.size() - 1});
            node_to_distance[key] = new_node.distance;

        }
    }

    return {};
}
