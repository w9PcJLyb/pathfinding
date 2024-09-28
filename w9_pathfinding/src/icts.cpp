#include <chrono>

#include "include/icts.h"

using std::chrono::duration;
using std::chrono::high_resolution_clock;


icts::MDD::MDD(AbsGraph* graph, int start, int goal, const ReservationTable* rt) :
    graph_(graph), start_(start), goal_(goal), depth_(1), bfs_depth_(1), rt_(rt)
{
    bfs_queue_.push({start_, 0});
    update_bfs_tree(bfs_depth_);
    generate_mdd(depth_);
}

int icts::MDD::depth() {
    return depth_; 
}

void icts::MDD::set_depth(int depth) {
    if (depth > bfs_depth_)
        update_bfs_tree(depth);

    if (depth != depth_)
        generate_mdd(depth);
}

void icts::MDD::update_bfs_tree(int depth) {
    bfs_depth_ = depth;
    while (!bfs_queue_.empty()) {
        dint nt = bfs_queue_.front();
        int node_id = nt.first;
        int d = nt.second;
        bfs_queue_.pop();

        if (rt_) {
            if (!rt_->is_reserved(d + 1, node_id)) {
                bfs_tree_[{node_id, d + 1}].insert(nt);
                bfs_queue_.push({node_id, d + 1});
            }

            auto reserved_edges = rt_->get_reserved_edges(d, node_id);
            for (auto& [neighbor_id, cost] : graph_->get_neighbors(node_id)) {
                if (!reserved_edges.count(neighbor_id) && !rt_->is_reserved(d + 1, neighbor_id))
                    add_node(neighbor_id, d + 1, nt);  
            }
        }
        else {
            bfs_tree_[{node_id, d + 1}].insert(nt);
            bfs_queue_.push({node_id, d + 1});

            for (auto& [neighbor_id, cost] : graph_->get_neighbors(node_id))
                add_node(neighbor_id, d + 1, nt);
        }

        if (d >= depth)
            break;
    }
}

void icts::MDD::add_node(int node_id, int depth, dint& parent) {
    dint node = {node_id, depth};
    bfs_tree_[node].insert(parent);
    if (!bfs_visited_.count(node)) {
        bfs_queue_.push(node);
        bfs_visited_.insert(node);
    }
}

void icts::MDD::generate_mdd(int depth) {
    depth_ = depth;

    data.clear();

    std::queue<dint> queue; 
    std::set<dint> visited;
    queue.push({goal_, depth});

    while (!queue.empty()) {
        dint nt = queue.front();
        queue.pop();

        if (!bfs_tree_.count(nt) || visited.count(nt))
            continue;

        visited.insert(nt);
        for (dint child : bfs_tree_[nt]) {
            queue.push(child);
            data[child].insert(nt);
        }
    }
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
    AbsGraph* graph, vector<int>& starts, vector<int>& goals, const ReservationTable *rt
) : graph_(graph), starts_(starts), goals_(goals)
{
    num_agents_ = starts_.size();

    edge_collision_ = graph->edge_collision();

    mdd_list_.reserve(num_agents_);
    for (int i = 0; i < num_agents_; i++)
        mdd_list_.emplace_back(graph_, starts_[i], goals_[i], rt);
}

vector<Path> icts::LowLevel::search(vector<int>& costs) {
    solution_.clear();
    solution_.push_back(starts_);

    int max_depth = 0;
    for (int i = 0; i < num_agents_; i++) {
        mdd_list_[i].set_depth(costs[i]);
        max_depth = std::max(max_depth, costs[i]);
    }

    if (explore(starts_, 0, max_depth))
        return get_paths();

    return {};
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
        if (depth >= mdd_list_[i].depth()) {
            if (positions[i] == goals_[i]) {
                neighbors[i].push_back(positions[i]);
                num_neighbors[i] = 1;
                continue;
            }
            else
                return false;
        }
        
        auto& s = mdd_list_[i].data[{positions[i], depth}];
        if (s.size() == 0)
            return false;
        
        for (auto [p, d] : s)
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
    return mapf(starts, goals, 100, 1.0, nullptr);
}

vector<Path> icts::ICTS::mapf(
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

    auto begin_time = high_resolution_clock::now();
    int num_agents = starts.size();

    std::queue<ICTNode> queue;
    std::unordered_set<std::string> visited;
    LowLevel low_level(graph, starts, goals, rt); 

    {
        vector<int> min_depths(num_agents);
        for (size_t i = 0; i < starts.size(); i++) {
            int start = starts[i];
            int goal = goals[i];
            int min_length = 0;
            if (rt)
                min_length = rt->last_time_reserved(goal);
            Path path = st_a_star_.find_path_with_depth_limit(
                start,
                goal,
                search_depth,
                rt,
                nullptr,
                min_length
            );
            if (path.empty() || path.back() !=  goal) {
                // there is no path from start to goal, or the path length is greater than search_depth
                return {};
            }

            min_depths[i] = path.size() - 1;
        }

        ICTNode root(min_depths);
        queue.push(root);
        visited.insert(root.to_str());
    }

    while (!queue.empty()) {
        ICTNode node = queue.front();
        queue.pop();

        vector<Path> paths = low_level.search(node.costs);
        if ((int)paths.size() == num_agents)
            return paths;

        if (duration<double>(high_resolution_clock::now() - begin_time).count() > max_time)
            throw timeout_exception("Timeout");

        for (int i = 0; i < num_agents; i++) {
            if (node.costs[i] + 1 > search_depth)
                continue;

            ICTNode new_node(node.costs);
            new_node.costs[i]++;

            if (visited.count(new_node.to_str()))
                continue;

            queue.push(new_node);
            visited.insert(new_node.to_str());
        }
    }

    return {};
}
