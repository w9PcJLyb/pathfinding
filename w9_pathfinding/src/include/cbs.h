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

    struct ConflictResult {
        int time = -1, node1 = -1, node2 = -1;

        // there are no conflicts
        ConflictResult() {}

        // vertex conflict
        ConflictResult(int time, int node_id) :
            time(time), node1(node_id) {}

        // edge conflict
        ConflictResult(int time, int node1, int node2) :
            time(time), node1(node1), node2(node2) {}

        bool has_conflict() const {
            return time >= 0;
        };

        bool is_edge_conflict() const {
            return node2 >= 0;
        };
    };

    struct CTNode {
        // Constraint Tree Node
        vector<vector<int>> solutions;
        vector<double> costs;
        CTNode *parent = nullptr;
        int agent_id = -1;
        ConflictResult conflict = ConflictResult();

        CTNode() {};
        CTNode(CTNode *parent_node, int agent_id, ConflictResult &conflict) :
            solutions(parent_node->solutions),
            costs(parent_node->costs),
            parent(parent_node),
            agent_id(agent_id),
            conflict(conflict) {};
    };

    typedef pair<double, CTNode*> key;
    typedef priority_queue<key, vector<key>, std::greater<key>> Queue;

    public:
        AbsGraph* graph;
        CBS(AbsGraph* graph);

        vector<vector<int>> mapf(vector<int> starts, vector<int> goals);
        vector<vector<int>> mapf(
            vector<int> starts,
            vector<int> goals,
            int search_depth,
            double max_time,
            bool despawn_at_destination,
            const ReservationTable *rt
        );

    private:
        SpaceTimeAStar st_a_star_;

        pair<vector<int>, ConflictResult> find_conflict(vector<vector<int>> &paths, bool despawn_at_destination);
        pair<vector<int>, double> low_level(Agent &agent, ReservationTable &rt, int search_depth);
        bool resolve_conflict(CTNode *ct_node, Agent &agent, ReservationTable rt, int search_depth);
        void print_node(CTNode &ct_node);
        void print_conflict(ConflictResult &conflict);
        void release_nodes(vector<CTNode*> nodes);
        vector<vector<int>> mapf_(
            vector<Agent> &agents,
            int search_depth,
            double max_time,
            bool despawn_at_destination,
            ReservationTable &rt
        );
};
