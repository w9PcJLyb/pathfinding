#pragma once

#include "core.h"
#include "space_time_a_star.h"
#include "resumable_search.h"
#include "reservation_table.h"


class CBS : public AbsMAPF {
    // Conflict Based Search
    // Sharon et al. 2012 Conflict-Based Search For Optimal Multi-Agent Path Finding

    struct Agent {
        int start, goal;
        ResumableSearch *rrs;

        Agent(int start, int goal, ResumableSearch *rrs) : start(start), goal(goal), rrs(rrs) {};
    };

    struct CTNode {
        // Constraint Tree Node
        vector<ReservationTable> constraints;
        vector<vector<int>> solutions;
        vector<double> costs;
        double total_cost = 0;

        CTNode& operator=(const CTNode& other) {
            constraints = other.constraints;
            solutions = other.solutions;
            costs = other.costs;
            total_cost = other.total_cost;
            return *this;
        }

        bool operator> (const CTNode& other) const {
            return total_cost > other.total_cost;
        }

        friend std::ostream& operator << (std::ostream& os, const CTNode& node) {
            os << "Node: cost=" << node.total_cost << ", solution=";
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
                os << "(" << s << ") ";
                t++;
            }
            os << " costs: ";
            for (auto x : node.costs) {
                os << x << " ";
            }
            return os;
        }
    };

    struct ConflictResult {
        int time = -1, node1 = -1, node2 = -1;
        vector<int> agent_ids;

        // there are no conflicts
        ConflictResult() {}

        // vertex conflict
        ConflictResult(int time, int node_id, vector<int> agent_ids) :
            time(time), node1(node_id), agent_ids(agent_ids) {}

        // edge conflict
        ConflictResult(int time, int node1, int node2, vector<int> agent_ids) :
            time(time), node1(node1), node2(node2), agent_ids(agent_ids) {}

        bool has_conflict() const {
            return time >= 0;
        };

        bool is_edge_conflict() const {
            return node2 >= 0;
        };

        friend std::ostream& operator << (std::ostream& os, const ConflictResult& r) {
            if (!r.has_conflict()) {
                os << "No conflicts";
            }
            else if (r.is_edge_conflict()) {
                os << "Edge conflict at time=" << r.time;
                os << ", edge=" << r.node1 << "->" << r.node2 << ")";
            }
            else {
                os << "Vertex conflict at time=" << r.time << ", node=" << r.node1;
            }
            return os;
        }
    };

    typedef priority_queue<CTNode, vector<CTNode>, std::greater<CTNode>> Queue;

    public:
        AbsGraph* graph;
        CBS(AbsGraph* graph);

        vector<vector<int>> mapf(vector<int> starts, vector<int> goals);
        vector<vector<int>> mapf(
            vector<int> starts,
            vector<int> goals,
            int search_depth,
            int max_iter,
            bool despawn_at_destination,
            const ReservationTable *rt
        );

    private:
        SpaceTimeAStar st_a_star_;

        ConflictResult find_conflict(vector<vector<int>> &paths, bool despawn_at_destination);
        pair<vector<int>, double> low_level(Agent &agent, ReservationTable &rt, int search_depth);
        void expand_paths(vector<vector<int>> &paths);
        vector<CTNode> split_node(
            CTNode &ct_node, vector<Agent> &agents, ConflictResult &conflict, int search_depth
        );
};
