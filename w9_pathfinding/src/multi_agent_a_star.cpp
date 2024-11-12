#include <chrono>

#include "include/multi_agent_a_star.h"

using std::chrono::duration;
using std::chrono::high_resolution_clock;


maas::MultiAgentAStar::MultiAgentAStar(AbsGraph *graph) : graph(graph) {
}

vector<Path> maas::MultiAgentAStar::mapf(vector<int> starts, vector<int> goals) {
    return mapf(starts, goals, 100, 1.0, true, nullptr);
}

vector<Path> maas::MultiAgentAStar::mapf(
    vector<int> starts,
    vector<int> goals,
    int max_length,
    double max_time,
    bool operator_decomposition,
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

    vector<Path> paths;
    if (!operator_decomposition) {
        AStarSolver solver(graph, starts, goals, rt);
        paths = solver.find_paths(max_length, max_time);
    }
    else {
        AStarODSolver solver(graph, starts, goals, rt);
        paths = solver.find_paths(max_length, max_time);
    }

    return paths;
}

maas::AStarSolver::AStarSolver(AbsGraph* graph, vector<int>& starts, vector<int>& goals, const ReservationTable* rt) :
    graph_(graph), rt_(rt) {

    SpaceTimeAStar st_a_star(graph_);

    agents_.reserve(starts.size());
    for (size_t agent_id = 0; agent_id < starts.size(); agent_id++) {
        int start = starts[agent_id];
        int goal = goals[agent_id];
        agents_.emplace_back(start, goal, st_a_star.reverse_resumable_search(goal));
    }

    num_agents_ = agents_.size();
}

vector<Path> maas::AStarSolver::find_paths(int max_length, double max_time) {
    auto begin_time = high_resolution_clock::now();

    Queue openset;
    tree_.clear();
    tree_.reserve(graph_->size());
    std::unordered_map<std::vector<int>, double, SpaceHash> node_to_distance;

    bool edge_collision = graph_->edge_collision();
    bool dynamic_obstacles = false;
    int min_search_depth = 0;
    if (rt_ && !rt_->empty()) {
        dynamic_obstacles = true;
        for (Agent& a : agents_)
            min_search_depth = std::max(min_search_depth, rt_->last_time_reserved(a.goal));
    }

    {
        vector<int> start;
        for (Agent& agent : agents_)
            start.push_back(agent.start);

        Node root = create_node(-1, 0, 0, start);

        double h = heuristic(start);
        if (h == -1)
            return {};
        openset.push({h, root.id});
        if (!dynamic_obstacles)
            node_to_distance[to_hash(root.positions, -1)] = 0;
    }

    while (!openset.empty()) {
        auto [f, node_id] = openset.top();
        openset.pop();
 
        Node node = tree_[node_id];

        if (node.time >= min_search_depth && is_goal(node.positions))
            return reconstruct_paths(node_id);

        if (node.time >= max_length)
            continue;

        vector<vector<pair<int, double>>> neighbors = get_neighbors(node);
        vector<pair<int, double>> combination;
        vector<int> indices;

        int counter = 0;
        while (generate_next_combination(neighbors, combination, indices)) {

            if (counter % 1000 == 0) {
                if (duration<double>(high_resolution_clock::now() - begin_time).count() > max_time)
                    throw timeout_exception("Timeout");
            }

            counter++;

            double cost = 0;
            vector<int> new_positions(num_agents_);
            for (int i = 0; i < num_agents_; i++) {
                cost += combination[i].second;
                new_positions[i] = combination[i].first;
            }

            if (has_collision(node.positions, new_positions, edge_collision))
                continue;

            vector<int> key = to_hash(new_positions, node_id);
            if (dynamic_obstacles)
                key.push_back(node.time);

            if (node_to_distance.count(key) && node.distance + cost >= node_to_distance[key])
                continue;

            double h = heuristic(new_positions);
            if (h < 0)
                continue;

            Node new_node = create_node(node_id, node.time + 1, node.distance + cost, new_positions);
            openset.push({new_node.distance + h, new_node.id});
            node_to_distance[key] = new_node.distance;
        }
    }

    return {};
}

void maas::AStarSolver::print_node(const Node& node) const {
    cout << "Node " << node.id << " : parent=" << node.parent << " distance=" << node.distance;
    cout << " time=" << node.time << " positions=";
    graph_->print_path(node.positions);
}

bool maas::AStarSolver::is_goal(vector<int>& positions) {
    for (int i = 0; i < num_agents_; i++) {
        if (positions[i] != agents_[i].goal)
            return false;
    }
    return true;
}

vector<vector<pair<int, double>>> maas::AStarSolver::get_neighbors(Node& node) {
    vector<vector<pair<int, double>>> neighbors(num_agents_);

    int time = node.time;
    double pause_action_cost = graph_->get_pause_action_cost();

    for (int i = 0; i < num_agents_; i++) {
        int p = node.positions[i];

        double pause_cost_here = 0;
        double additional_moving_cost_here = 0;
        if (p != agents_[i].goal)
            pause_cost_here = pause_action_cost;
        else
            additional_moving_cost_here = get_waiting_time(node.id, i) * pause_action_cost;

        if (!rt_) {
            neighbors[i].push_back({p, pause_cost_here});
            for (auto &[n, cost] : graph_->get_neighbors(p))
                neighbors[i].push_back({n, cost + additional_moving_cost_here});
        }
        else {
            if (!rt_->is_reserved(time + 1, p))
                neighbors[i].push_back({p, pause_cost_here});

            auto reserved_edges = rt_->get_reserved_edges(time, p);
            for (auto &[n, cost] : graph_->get_neighbors(p)) {
                if (!reserved_edges.count(n) && !rt_->is_reserved(time + 1, n))
                    neighbors[i].push_back({n, cost + additional_moving_cost_here});
            }
        }

        if (neighbors[i].empty()) {
            for (auto& n : neighbors)
                n.clear();
            return neighbors;
        }
    }

    return neighbors;
}

vector<Path> maas::AStarSolver::reconstruct_paths(int node_id) {
    vector<Path> paths(num_agents_);
    while (node_id >= 0) {
        Node& node = tree_.at(node_id);
        for (int i = 0; i < num_agents_; i++)
            paths[i].push_back(node.positions[i]);
        node_id = node.parent;
    }

    for (int i = 0; i < num_agents_; i++) {
        vector<int>& path = paths[i];
        std::reverse(path.begin(), path.end());

        int goal = agents_[i].goal;
        while (path.size() > 1 && path.back() == goal && path[path.size() - 2] == goal)
            path.pop_back();
    }
    return paths;
}

maas::Node maas::AStarSolver::create_node(int parent, int time, double distance, vector<int> &positions) {
    int node_id = tree_.size();
    tree_.emplace_back(node_id, parent, time, distance, positions);
    return tree_.back();
}

int maas::AStarSolver::get_waiting_time(int node_id, int agent_id) {
    int waiting_time = 0;
    int goal = agents_[agent_id].goal;
    node_id = tree_.at(node_id).parent;
    while (node_id >= 0 && tree_[node_id].positions[agent_id] == goal) {
        waiting_time++;
        node_id = tree_[node_id].parent;
    }
    return waiting_time;
}

double maas::AStarSolver::heuristic(vector<int>& positions) {
    double h = 0;
    for (int i = 0; i < num_agents_; i++) {
        double min_distance = agents_[i].rrs->distance(positions[i]);
        if (min_distance < 0)
            return -1;
        h += min_distance;
    }
    return h;
}

vector<int> maas::AStarSolver::to_hash(vector<int>& positions, int parent) {
    vector<int> hash(num_agents_);
    for (int i = 0; i < num_agents_; i++) {
        if (positions[i] != agents_[i].goal)
            hash[i] = positions[i];
        else if (parent >= 0)
            hash[i] = -1 - get_waiting_time(parent, i);
        else
            hash[i] = -1;
    }
    return hash;
}


vector<Path> maas::AStarODSolver::find_paths(int max_length, double max_time) {
    auto begin_time = high_resolution_clock::now();

    Queue openset;
    tree_.clear();
    tree_.reserve(graph_->size());
    std::unordered_map<std::vector<int>, double, SpaceHash> node_to_distance;

    bool dynamic_obstacles = false;
    int min_search_depth = 0;
    if (rt_ && !rt_->empty()) {
        dynamic_obstacles = true;
        for (Agent& a : agents_)
            min_search_depth = std::max(min_search_depth, rt_->last_time_reserved(a.goal));
    }

    {
        vector<int> start;
        for (Agent& agent : agents_)
            start.push_back(agent.start);

        Node root = create_node(-1, 0, 0, start);

        double h = heuristic(start);
        if (h == -1)
            return {};

        openset.push({h, root.id});
        if (!dynamic_obstacles)
            node_to_distance[to_hash(root.positions, -1)] = 0;
    }

    while (!openset.empty()) {
        auto [f, node_id] = openset.top();
        openset.pop();

        Node node = tree_[node_id];

        int agent_id = node.time % num_agents_;
        int standard_time = node.time / num_agents_;

        if (standard_time >= min_search_depth && is_goal(node.positions))
            return reconstruct_paths(node_id);

        if (standard_time >= max_length)
            continue;

        if (duration<double>(high_resolution_clock::now() - begin_time).count() > max_time)
            throw timeout_exception("Timeout");

        int parent = agent_id == 0 ? node_id : node.parent;

        vector<pair<int, double>> neighbors = get_neighbors(node);
        std::unordered_set<int> occupied_nodes = get_occupied_nodes(node);

        for (auto &[n, cost] : neighbors) {
            if (occupied_nodes.count(n))
                continue;

            vector<int> new_positions = node.positions;
            new_positions[agent_id] = n;

            double new_distance = node.distance + cost;

            if (agent_id == num_agents_ - 1) {
                vector<int> key = to_hash(new_positions, node_id);
                if (dynamic_obstacles)
                    key.push_back(node.time);

                if (node_to_distance.count(key) && new_distance >= node_to_distance[key])
                    continue;

                node_to_distance[key] = new_distance;
            }

            double h = heuristic(new_positions);
            if (h < 0)
                continue;

            Node new_node = create_node(parent, node.time + 1, new_distance, new_positions);
            openset.push({new_distance + h, new_node.id});
        }
    }

    return {};
}

maas::AStarODSolver::AStarODSolver(AbsGraph* graph, vector<int>& starts, vector<int>& goals, const ReservationTable* rt) :
    AStarSolver(graph, starts, goals, rt) {
};

vector<pair<int, double>> maas::AStarODSolver::get_neighbors(Node& node) {
    int agent_id = node.time % num_agents_;
    int p = node.positions[agent_id];

    double pause_cost_here = 0;
    double additional_moving_cost_here = 0;
    if (p != agents_[agent_id].goal)
        pause_cost_here = graph_->get_pause_action_cost();
    else {
        int parent = agent_id == 0 ? node.id : node.parent;
        additional_moving_cost_here = get_waiting_time(parent, agent_id) * graph_->get_pause_action_cost();
    }

    vector<pair<int, double>> neighbors;

    if (!rt_) {
        neighbors.push_back({p, pause_cost_here});
        for (auto &[n, cost] : graph_->get_neighbors(p))
            neighbors.push_back({n, cost + additional_moving_cost_here});
    }
    else {
        int time = node.time / num_agents_;

        if (!rt_->is_reserved(time + 1, p))
            neighbors.push_back({p, pause_cost_here});

        auto reserved_edges = rt_->get_reserved_edges(time, p);
        for (auto &[n, cost] : graph_->get_neighbors(p)) {
            if (!reserved_edges.count(n) && !rt_->is_reserved(time + 1, n))
                neighbors.push_back({n, cost + additional_moving_cost_here});
        }
    }

    return neighbors;
}

std::unordered_set<int> maas::AStarODSolver::get_occupied_nodes(Node& node) {
    int agent_id = node.time % num_agents_;

    std::unordered_set<int> occupied_nodes;
    if (agent_id == 0) {
        // no agent has made a move yet
        return occupied_nodes;
    }

    if (node.parent < 0 || !graph_->edge_collision()) {
        for (int i = 0; i < agent_id; i++)
            occupied_nodes.insert(node.positions[i]); // vertex conflict
        return occupied_nodes;
    }

    int p = node.positions[agent_id];
    vector<int>& previous_positions = tree_[node.parent].positions;
    for (int i = 0; i < agent_id; i++) {
        occupied_nodes.insert(node.positions[i]); // vertex conflict
        if (node.positions[i] == p)
            occupied_nodes.insert(previous_positions[i]); // edge conflict
    }
    return occupied_nodes;
}
