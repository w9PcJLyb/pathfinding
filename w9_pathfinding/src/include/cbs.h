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

    struct Conflict {
        int agent_id, time, node1, node2 = -1;

        // vertex conflict
        Conflict(int agent_id, int time, int node_id) :
            agent_id(agent_id), time(time), node1(node_id) {}

        // edge conflict
        Conflict(int agent_id, int time, int node1, int node2) :
            agent_id(agent_id), time(time), node1(node1), node2(node2) {}

        bool is_edge_conflict() {
            return node2 >= 0;
        };
    };

    struct CTNode {
        // Constraint Tree Node
        int parent;
        vector<vector<int>> solutions;
        vector<double> costs;
        Conflict conflict;

        CTNode() : parent(-1), conflict(Conflict(-1, -1, -1)) {};
        CTNode(int parent, Conflict conflict) : parent(parent), conflict(conflict) {};
    };

    typedef pair<double, int> key;
    typedef priority_queue<key, vector<key>, std::greater<key>> Queue;
    typedef vector<CTNode> ConstraintTree;

    public:
        AbsGraph* graph;
        CBS(AbsGraph* graph);

        vector<vector<int>> mapf(vector<int> starts, vector<int> goals);
        vector<vector<int>> mapf(
            vector<int> starts,
            vector<int> goals,
            int search_depth,
            double max_time,
            const ReservationTable *rt
        );

    private:
        SpaceTimeAStar st_a_star_;

        vector<Conflict> find_conflict(vector<vector<int>> &paths);
        pair<vector<int>, double> low_level(Agent &agent, ReservationTable &rt, int search_depth);
        bool resolve_conflict(CTNode &ct_node, ConstraintTree &tree, Agent &agent, ReservationTable rt, int search_depth);
        void print_node(CTNode &ct_node);
        void print_conflict(Conflict &conflict);
        vector<vector<int>> mapf_(
            vector<Agent> &agents,
            int search_depth,
            double max_time,
            ReservationTable &rt
        );
};
