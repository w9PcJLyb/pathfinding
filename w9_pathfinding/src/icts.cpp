#include "include/icts.h"


icts::ResumableBFS::ResumableBFS(AbsGraph* graph, int start, int goal, const ReservationTable* rt) :
    start(start), goal(goal), graph_(graph), depth_(1), rt_(rt)
{
    queue_.push({0, start});
    update_data(depth_);
}

void icts::ResumableBFS::set_depth(int depth) {
    if (depth > depth_)
        update_data(depth);
    depth_ = depth;
}

void icts::ResumableBFS::update_data(int depth) {
    while (!queue_.empty()) {
        auto [time, node_id] = queue_.front();
        queue_.pop();

        if (rt_) {
            if (!rt_->is_reserved(time + 1, node_id))
                add_record(time + 1, node_id, node_id);

            auto reserved_edges = rt_->get_reserved_edges(time, node_id);
            for (auto& [neighbor_id, cost] : graph_->get_neighbors(node_id)) {
                if (!reserved_edges.count(neighbor_id) && !rt_->is_reserved(time + 1, neighbor_id))
                    add_record(time + 1, neighbor_id, node_id);
            }
        }
        else {
            add_record(time + 1, node_id, node_id);
            for (auto& [neighbor_id, cost] : graph_->get_neighbors(node_id))
                add_record(time + 1, neighbor_id, node_id);
        }

        if (time >= depth)
            break;
    }
}

void icts::ResumableBFS::add_record(int time, int node_id, int parent) {
    dint node = {time, node_id};
    data[node].push_back(parent);
    if (!visited_.count(node)) {
        queue_.push(node);
        visited_.insert(node);
    }
}

icts::MDD::MDD(ResumableBFS& bfs) : start(bfs.start), goal(bfs.goal), depth(bfs.depth()) {
    std::queue<dint> queue; 
    std::set<dint> visited;
    queue.push({depth, goal});

    while (!queue.empty()) {
        dint nt = queue.front();
        queue.pop();

        if (!bfs.data.count(nt) || visited.count(nt))
            continue;

        auto [time, node_id] = nt;
        visited.insert(nt);
        for (int parent : bfs.data[nt]) {
            queue.push({time - 1, parent});
            data[{time - 1, parent}].insert(node_id);
        }
    }
}

void icts::MDD::print(AbsGraph* graph) {
    cout << "MDD(start=" << graph->node_to_string(start);
    cout << ", goal=" << graph->node_to_string(goal);
    cout << ", depth=" << depth << ":" << endl;
    if (data.empty())
        cout << " - empty" << endl;
    else {
        for (auto const& x : data) {
            int time = x.first.first;
            int node_id = x.first.second;
            cout << " - node=" << graph->node_to_string(node_id) << ", time=" << time << " -> ";
            for (auto& n : x.second) {
                cout << graph->node_to_string(n) << " ";
            }
            cout << endl;
        }
    }
}

icts::MDD2::MDD2(MDD& mdd1, MDD& mdd2, bool edge_collision) :
    starts({mdd1.start, mdd2.start}),
    goals({mdd1.goal, mdd2.goal}),
    depths({mdd1.depth, mdd2.depth}),
    depth(std::max(mdd1.depth, mdd2.depth))
{
    std::queue<pair<int, dint>> queue;
    queue.push({0, starts});
    std::set<pair<int, dint>> visited;

    resolved = false;
    while (!queue.empty()) {
        auto [time, positions] = queue.front();
        queue.pop();

        auto [n1, n2] = positions;

        std::unordered_set<int> neighbors_1, neighbors_2;

        if (time < mdd1.depth)
            neighbors_1 = mdd1.data.at({time, n1});
        else
            neighbors_1 = {{n1}};

        if (time < mdd2.depth)
            neighbors_2 = mdd2.data.at({time, n2});
        else
            neighbors_2 = {{n2}};

        vector<dint> neighbors;
        for (int next_n1 : neighbors_1) {
            for (int next_n2 : neighbors_2) {
                if (next_n1 == next_n2) {
                    // vertex conflict
                    continue;
                }

                if (edge_collision && next_n1 == n2 && next_n2 == n1) {
                    // edge conflict
                    continue;
                }

                neighbors.push_back({next_n1, next_n2});
            }
        }

        if (neighbors.empty())
            continue;

        data[{time, positions}] = neighbors;
        if (time < depth - 1)
            for (dint& positions : neighbors) {
                pair<int, dint> key = {time + 1, positions};
                if (visited.count(key))
                    continue;
                visited.insert(key);
                queue.push(key);
            }
        else if (time == depth - 1 && !resolved) {
            for (dint& positions : neighbors)
                if (positions == goals)
                    resolved = true;
        }
    }

    prune();
}

pair<icts::MDD, icts::MDD> icts::MDD2::unfold() {
    MDD mdd1(starts.first, goals.first, depths.first);
    MDD mdd2(starts.second, goals.second, depths.second);

    for (auto& [key, next_positions] : data) {
        int time = key.first;
        auto [p1, p2] = key.second;

        if (time < mdd1.depth) {
            auto& d = mdd1.data[{time, p1}];
            for (auto& [n1, n2] : next_positions)
                d.insert(n1);
        }

        if (time < mdd2.depth) {
            auto& d = mdd2.data[{time, p2}];
            for (auto& [n1, n2] : next_positions) {
                d.insert(n2);
            }
        }
    }

    return {mdd1, mdd2};
}

void icts::MDD2::print(AbsGraph* graph) {

    auto positions_to_str = [&] (dint positions) {
        std::string s;
        s += "[" + graph->node_to_string(positions.first);
        s += ", " + graph->node_to_string(positions.second) + "]";
        return s;
    };

    cout << "MDD2(start=" << positions_to_str(starts);
    cout << ", goal=" << positions_to_str(goals);
    cout << ", depth=(" << depths.first << ", " << depths.second << ")" << ":" << endl;
    if (data.empty())
        cout << " - empty" << endl;
    else {
        for (auto const& x : data) {
            int time = x.first.first;
            dint positions = x.first.second;
            cout << " - node=" << positions_to_str(positions) << ", time=" << time << " -> ";
            for (auto& n : x.second) {
                cout << positions_to_str(n) << " ";
            }
            cout << endl;
        }
    }
}

void icts::MDD2::prune() {
    std::map<pair<int, dint>, bool> mem;
    get_result_and_prune(0, starts, mem);
}

bool icts::MDD2::get_result_and_prune(int time, dint positions, std::map<pair<int, dint>, bool>& mem) {
    pair<int, dint> key = {time, positions};
    if (mem.count(key))
        return mem[key];

    if (!data.count(key)) {
        mem[key] = false;
        return false;
    }

    vector<dint>& next_positions = data.at(key);
    if (next_positions.empty()) {
        data.erase(key);
        mem[key] = false;
        return false;
    }

    bool result = false;

    std::unordered_set<int> indexes_to_delete;
    for (size_t i = 0; i < next_positions.size(); i++) {
        if (time == depth - 1 && next_positions[i] == goals) {
            result = true;
            continue;
        }

        int n = get_result_and_prune(time + 1, next_positions[i], mem);
        if (n > 0)
            result = true;
        else
            indexes_to_delete.insert(i);
    }

    if (!indexes_to_delete.empty()) {
        vector<dint> next_positions_pruned;
        for (size_t i = 0; i < next_positions.size(); i++) {
            if (!indexes_to_delete.count(i))
                next_positions_pruned.push_back(next_positions[i]);
        }

        if (next_positions_pruned.empty())
            data.erase(key);
        else
            data[key] = next_positions_pruned;
    }

    mem[key] = result;
    return result;
}

icts::ICTNode::ICTNode(vector<int>& costs) : costs(costs) {
}

std::string icts::ICTNode::to_str() {
    std::string s;
    for (int x : costs) {
        if (!s.empty())
            s += " ";
        s += std::to_string(x);
    }
    return s;
}

icts::LowLevel::LowLevel(
    AbsGraph* graph,
    vector<int>& starts,
    vector<int>& goals,
    bool ict_pruning,
    const ReservationTable *rt,
    time_point<high_resolution_clock> terminate_time
) : graph_(graph), starts_(starts), goals_(goals), ict_pruning_(ict_pruning), terminate_time_(terminate_time)
{
    num_agents_ = starts_.size();
    edge_collision_ = graph->edge_collision();

    bfses_.reserve(num_agents_);
    for (int i = 0; i < num_agents_; i++)
        bfses_.emplace_back(graph_, starts_[i], goals_[i], rt);

    mdds_.resize(num_agents_);
}

vector<Path> icts::LowLevel::search(vector<int>& costs) {
    int max_depth = 0;
    for (int i = 0; i < num_agents_; i++) {
        int depth = costs[i];
        bfses_[i].set_depth(depth);
        mdds_[i] = MDD(bfses_[i]);
        max_depth = std::max(max_depth, depth);
    }

    if (ict_pruning_ && num_agents_ > 2 && enhanced_pairwise_pruning())
        return {};

    return find_solution(max_depth);
}

bool icts::LowLevel::enhanced_pairwise_pruning() {
    for (int i=0; i < num_agents_ - 1; i++) {
        if (mdds_[i].depth == 0)
            continue;

        for (int j=i+1; j < num_agents_; j++) {
            if (mdds_[j].depth == 0)
                continue;

            if (high_resolution_clock::now() > terminate_time_)
                throw timeout_exception("Timeout");

            MDD2 mdd2(mdds_[i], mdds_[j], edge_collision_);
            if (!mdd2.resolved)
                return true;
            auto [mdd1_new, mdd2_new] = mdd2.unfold();
            mdds_[i] = mdd1_new;
            mdds_[j] = mdd2_new;
        }
    }
    return false;
}

vector<Path> icts::LowLevel::get_paths() {
    vector<Path> paths(num_agents_);
    for (int i = 0; i < num_agents_; i++) {
        vector<int> path;
        for (vector<int>& positions : solution_)
            path.push_back(positions[i]);

        int goal = goals_[i];
        while (path.size() > 1 && path.back() == goal && path[path.size() - 2] == goal)
            path.pop_back();

        paths[i] = path;
    }

    return paths;
}

vector<vector<int>> icts::LowLevel::get_neighbors(int time, vector<int>& positions) {
    vector<vector<int>> neighbors(num_agents_);
    for (int i = 0; i < num_agents_; i++) {
        if (time >= mdds_[i].depth) {
            if (positions[i] == goals_[i])
                neighbors[i].push_back(goals_[i]);
            continue;
        }

        auto& s = mdds_[i].data[{time, positions[i]}];
        neighbors[i] = {s.begin(), s.end()};
    }
    return neighbors;
}

vector<Path> icts::LowLevel::find_solution(int target_depth) {
    solution_.clear();
    solution_.push_back(starts_);
    std::unordered_set<pair<int, vector<int>>, SpaceTimeHash> checked;
    if (explore(0, starts_, target_depth, checked))
        return get_paths();
    return {};
}

bool icts::LowLevel::explore(int time, vector<int>& positions, int target_depth, std::unordered_set<pair<int, vector<int>>, SpaceTimeHash>& checked) {
    if (time >= target_depth)
        return positions == goals_;

    pair<int, vector<int>> key = {time, positions};
    if (checked.count(key))
        return false;

    if (high_resolution_clock::now() > terminate_time_)
        throw timeout_exception("Timeout");

    vector<vector<int>> neighbors = get_neighbors(time, positions);

    vector<int> next_positions, indices;
    while (generate_next_combination(neighbors, next_positions, indices)) {

        if (has_collision(positions, next_positions, edge_collision_))
            continue;

        solution_.push_back(next_positions);

        if (explore(time + 1, next_positions, target_depth, checked))
            return true;

        solution_.pop_back();
    }

    checked.insert(key);
    return false;
}

icts::ICTS::ICTS(AbsGraph *graph) : graph(graph), st_a_star_(graph) {
}

vector<Path> icts::ICTS::mapf(vector<int> starts, vector<int> goals) {
    return mapf(starts, goals, 100, 1.0, true, nullptr);
}

vector<Path> icts::ICTS::mapf(
    vector<int> starts,
    vector<int> goals,
    int max_length,
    double max_time,
    bool ict_pruning,
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

    auto begin_time = high_resolution_clock::now();
    auto terminate_time = begin_time + duration_cast<milliseconds>(duration<double>(max_time));
    int num_agents = starts.size();

    std::queue<ICTNode> queue;
    std::unordered_set<std::string> visited;
    LowLevel low_level(graph, starts, goals, ict_pruning, rt, terminate_time);

    {
        vector<int> min_depths(num_agents);
        for (size_t i = 0; i < starts.size(); i++) {
            int start = starts[i];
            int goal = goals[i];
            int min_length = 0;
            if (rt)
                min_length = rt->last_time_reserved(goal);
            Path path = st_a_star_.find_path_with_length_limit(
                start,
                goal,
                max_length,
                rt,
                nullptr,
                min_length
            );
            if (path.empty())
                return {};

            min_depths[i] = path.size() - 1;
        }

        ICTNode root(min_depths);
        queue.push(root);
        num_generated_nodes++;
        visited.insert(root.to_str());
    }

    while (!queue.empty()) {
        ICTNode node = queue.front();
        queue.pop();

        vector<Path> paths = low_level.search(node.costs);
        if ((int)paths.size() == num_agents) {
            num_closed_nodes++;
            return paths;
        }

        for (int i = 0; i < num_agents; i++) {
            if (node.costs[i] + 1 > max_length)
                continue;

            ICTNode new_node(node.costs);
            new_node.costs[i]++;

            if (visited.count(new_node.to_str()))
                continue;

            queue.push(new_node);
            num_generated_nodes++;
            visited.insert(new_node.to_str());
        }

        num_closed_nodes++;
    }

    return {};
}
