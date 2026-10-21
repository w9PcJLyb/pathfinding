#pragma once

#include "cbs.h"


class GCBS : public CBS {
    // Greedy Conflict Based Search
    // Fast and complete but not optimal

    // Barer, M.; Sharon, G.; Stern, R.; and Felner, A. 2014.
    // Sub-optimal Variants of the Conflict-Based Search Algorithm for
    // the Multi-Agent Pathfinding Problem.

    public:
        using CBS::CBS;

    protected:
        double high_level_heuristics(CTNode &ct_node) override {
            auto paths = ct_node.solutions;
            int num_agents = paths.size();
            size_t time = 0;
            std::unordered_map<int, vector<int>> node_to_agents;
            bool edge_collision = env->edge_collision();
            bool end = false;

            int num_conflicts = 0;
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
                                num_conflicts += 1;
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
                        num_conflicts += 1;
                    }
                }

                time++;
            }

            return num_conflicts;
        }
};
