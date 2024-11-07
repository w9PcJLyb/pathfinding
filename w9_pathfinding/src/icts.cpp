#include <chrono>

#include "include/icts.h"

using std::chrono::duration;
using std::chrono::high_resolution_clock;


icts::ResumableBFS::ResumableBFS(AbsGraph* graph, int start, int goal, const ReservationTable* rt) :
    start(start), goal(goal), graph_(graph), depth_(1), rt_(rt)
{
    queue_.push({start, 0});
    update_data(depth_);
}

void icts::ResumableBFS::set_depth(int depth) {
    if (depth > depth_)
        update_data(depth);
    depth_ = depth;
}

void icts::ResumableBFS::update_data(int depth) {
    while (!queue_.empty()) {
        auto [node_id, time] = queue_.front();
        queue_.pop();

        if (rt_) {
            if (!rt_->is_reserved(time + 1, node_id))
                add_record(node_id, time + 1, node_id);

            auto reserved_edges = rt_->get_reserved_edges(time, node_id);
            for (auto& [neighbor_id, cost] : graph_->get_neighbors(node_id)) {
                if (!reserved_edges.count(neighbor_id) && !rt_->is_reserved(time + 1, neighbor_id))
                    add_record(neighbor_id, time + 1, node_id);
            }
        }
        else {
            add_record(node_id, time + 1, node_id);
            for (auto& [neighbor_id, cost] : graph_->get_neighbors(node_id))
                add_record(neighbor_id, time + 1, node_id);
        }

        if (time >= depth)
            break;
    }
}

void icts::ResumableBFS::add_record(int node_id, int time, int parent) {
    dint node = {node_id, time};
    data[node].push_back(parent);
    if (!visited_.count(node)) {
        queue_.push(node);
        visited_.insert(node);
    }
}

icts::MDD::MDD(ResumableBFS& bfs) : start(bfs.start), goal(bfs.goal), depth(bfs.depth()) {
    std::queue<dint> queue; 
    std::set<dint> visited;
    queue.push({goal, depth});

    while (!queue.empty()) {
        dint nt = queue.front();
        queue.pop();

        if (!bfs.data.count(nt) || visited.count(nt))
            continue;

        auto [node_id, time] = nt;
        visited.insert(nt);
        for (int parent : bfs.data[nt]) {
            queue.push({parent, time - 1});
            data[{parent, time - 1}].insert(node_id);
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
            int node_id = x.first.first;
            int time = x.first.second;
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
    std::queue<pair<dint, int>> queue;
    queue.push({starts, 0});

    resolved = false;
    while (!queue.empty()) {
        auto [positions, time] = queue.front();
        queue.pop();

        int n1 = positions.first;
        int n2 = positions.second;

        std::unordered_set<int> neighbors_1, neighbors_2;

        if (time < mdd1.depth) {
            if (!mdd1.data.count({n1, time}))
                cout << "1 " << time << " " << n1 << endl;
            neighbors_1 = mdd1.data.at({n1, time});
        }
        else
            neighbors_1 = {{n1}};

        if (time < mdd2.depth) {
            if (!mdd2.data.count({n2, time}))
                cout << "2 " << time << " " << n2 << endl;
            neighbors_2 = mdd2.data.at({n2, time});
            }
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

        data[{{n1, n2}, time}] = neighbors;
        if (time < depth - 1)
            for (dint& positions : neighbors)
                queue.push({positions, time + 1});
        else if (time == depth - 1 && !resolved) {
            for (dint& positions : neighbors)
                if (positions == goals)
                    resolved = true;
        }
    }

    prune(starts, 0);
}

pair<icts::MDD, icts::MDD> icts::MDD2::unfold() {
    MDD mdd1(starts.first, goals.first, depths.first);
    MDD mdd2(starts.second, goals.second, depths.second);

    for (auto& [key, next_positions] : data) {
        auto [p1, p2] = key.first;
        int time = key.second;

        if (time < mdd1.depth) {
            auto& d = mdd1.data[{p1, time}];
            for (auto& [n1, n2] : next_positions)
                d.insert(n1);
        }

        if (time < mdd2.depth) {
            auto& d = mdd2.data[{p2, time}];
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
            dint positions = x.first.first;
            int time = x.first.second;
            cout << " - node=" << positions_to_str(positions) << ", time=" << time << " -> ";
            for (auto& n : x.second) {
                cout << positions_to_str(n) << " ";
            }
            cout << endl;
        }
    }
}

int icts::MDD2::prune(dint positions, int time) {
    int num_childrens = 0;

    if (!data.count({positions, time}))
        return num_childrens;

    vector<dint> next_positions_pruned;
    for (dint next_positions : data.at({positions, time})) {
        if (time == depth - 1 && next_positions == goals) {
            num_childrens += 1;
            next_positions_pruned.push_back(next_positions);
            continue;
        }

        int n = prune(next_positions, time + 1);
        if (n > 0) {
            num_childrens += n;
            next_positions_pruned.push_back(next_positions);
        }
    }

    if (next_positions_pruned.empty())
        data.erase({positions, time});
    else
        data[{positions, time}] = next_positions_pruned;

    return num_childrens;
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
    AbsGraph* graph, vector<int>& starts, vector<int>& goals, bool ict_pruning, const ReservationTable *rt
) : graph_(graph), starts_(starts), goals_(goals), ict_pruning(ict_pruning)
{
    num_agents_ = starts_.size();
    edge_collision_ = graph->edge_collision();

    bfses_.reserve(num_agents_);
    for (int i = 0; i < num_agents_; i++)
        bfses_.emplace_back(graph_, starts_[i], goals_[i], rt);

    mdds_.resize(num_agents_);
}

vector<Path> icts::LowLevel::search(vector<int>& costs) {
    solution_.clear();
    solution_.push_back(starts_);

    int max_depth = 0;
    for (int i = 0; i < num_agents_; i++) {
        int depth = costs[i];
        bfses_[i].set_depth(depth);
        mdds_[i] = MDD(bfses_[i]);
        max_depth = std::max(max_depth, depth);
    }

    if (ict_pruning && num_agents_ > 2 && enhanced_pairwise_pruning())
        return {};

    if (explore(starts_, 0, max_depth))
        return get_paths();

    return {};
}

bool icts::LowLevel::enhanced_pairwise_pruning() {
    for (int i=0; i < num_agents_ - 1; i++) {
        for (int j=i+1; j < num_agents_; j++) {
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

bool icts::LowLevel::explore(vector<int>& positions, int depth, int target_depth) {
    if (depth >= target_depth)
        return positions == goals_;

    vector<vector<int>> neighbors(num_agents_);
    vector<int> num_neighbors(num_agents_);
    for (int i = 0; i < num_agents_; i++) {
        if (depth >= mdds_[i].depth) {
            if (positions[i] == goals_[i]) {
                neighbors[i].push_back(positions[i]);
                num_neighbors[i] = 1;
                continue;
            }
            else
                return false;
        }
        
        auto& s = mdds_[i].data[{positions[i], depth}];
        if (s.size() == 0)
            return false;
        
        for (int p : s)
            neighbors[i].push_back(p);
        num_neighbors[i] = s.size();
    }

    vector<int> indexes(positions.size(), 0);
    indexes.back() = -1;

    auto update_indexes = [&] () {
        int i = indexes.size() - 1;
        while (i >= 0) {
            if (indexes[i] < num_neighbors[i] - 1) {
                indexes[i]++;
                return true;
            }
            else {
                indexes[i] = 0;
                i--;
            }
        }
        return false;
    };

    vector<int> next_positions(num_agents_);
    while (true) {
        if (!update_indexes())
            break;

        for (int i = 0; i < num_agents_; i++)
            next_positions[i] = neighbors[i][indexes[i]];

        if (has_collision(positions, next_positions))
            continue;

        solution_.push_back(next_positions);

        if (explore(next_positions, depth + 1, target_depth))
            return true;

        solution_.pop_back();
    }

    return false;
}

bool icts::LowLevel::has_collision(vector<int>& positions, vector<int>& next_positions) {
    std::unordered_map<int, int> node_to_agent;
    for (size_t i = 0; i < next_positions.size(); i++) {
        int node_id = next_positions[i];
        if (node_to_agent.count(node_id)) {
            // vertex conflict
            return true;
        }
        else
            node_to_agent[node_id] = i;
    }

    if (edge_collision_) {
        for (size_t i = 0; i < positions.size(); i++) {
            int p = positions[i];
            int next_p = next_positions[i];
            if (p == next_p)
                continue;
            if (node_to_agent.count(p) && positions[node_to_agent[p]] == next_p) {
                // edge conflict
                return true;
            }
        }
    }

    return false;
}

icts::ICTS::ICTS(AbsGraph *graph) : graph(graph), st_a_star_(graph) {
}

vector<Path> icts::ICTS::mapf(vector<int> starts, vector<int> goals) {
    return mapf(starts, goals, 100, 1.0, false, nullptr);
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
    int num_agents = starts.size();

    std::queue<ICTNode> queue;
    std::unordered_set<std::string> visited;
    LowLevel low_level(graph, starts, goals, ict_pruning, rt);

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

        if (duration<double>(high_resolution_clock::now() - begin_time).count() > max_time)
            throw timeout_exception("Timeout");

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
