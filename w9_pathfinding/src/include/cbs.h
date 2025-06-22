#pragma once

#include <memory>
#include "env.h"
#include "space_time_a_star.h"
#include "resumable_search.h"
#include "reservation_table.h"


class CBS : public AbsMAPF {
    /*
    Conflict Based Search
    Sharon et al. 2012 Conflict-Based Search For Optimal Multi-Agent Path Finding
    Li et al. 2019 Disjoint Splitting for Multi-Agent Path Finding with Conflict-Based Search
    */

    struct Agent {
        int start, goal;
        std::unique_ptr<ResumableSearch> rrs;

        Agent(int start, int goal, std::unique_ptr<ResumableSearch> rrs)
            : start(start), goal(goal), rrs(std::move(rrs)) {}
    };

    struct Conflict {
        int time, node1, node2 = -1;

        Conflict() : time(-1), node1(-1), node2(-1) {}

        // vertex conflict
        Conflict(int time, int node_id) :
            time(time), node1(node_id) {}

        // edge conflict
        Conflict(int time, int node1, int node2) :
            time(time), node1(node1), node2(node2) {}

        bool is_edge_conflict() {
            return node2 >= 0;
        };

        bool is_vertex_conflict() {
            return node2 < 0;
        };

        bool operator == (const Conflict& c) const {
            return time == c.time && node1 == c.node1 && node2 == c.node2;
        }
    };

    struct Constraint {
        int agent_id;
        Conflict conflict;
        vector<int> conflicting_agents;

        // a positive constraint requires the agent to be at the conflict
        // a negative constraint compels the agent to avoid it
        bool is_positive;

        Constraint() : agent_id(-1), conflict(), is_positive(false) {}
        Constraint(int agent_id, Conflict& conflict, bool is_positive = false) :
            agent_id(agent_id), conflict(conflict), is_positive(is_positive) {}
    };

    struct CTNode {
        // Constraint Tree Node
        int parent;
        vector<Path> solutions;
        vector<double> costs;
        Constraint constraint;

        CTNode() : parent(-1), constraint() {};
        CTNode(int parent, Constraint constraint) : parent(parent), constraint(constraint) {};

        double total_cost() {
            return std::accumulate(costs.begin(), costs.end(), 0.);
        }
    };

    typedef pair<double, int> key;
    typedef priority_queue<key, vector<key>, std::greater<key>> Queue;
    typedef vector<CTNode> ConstraintTree;

    public:
        Env* env;
        CBS(Env* env);
        int num_generated_nodes = 0;
        int num_closed_nodes = 0;

        vector<Path> mapf(vector<int> starts, vector<int> goals);
        vector<Path> mapf(
            vector<int> starts,
            vector<int> goals,
            int max_length,
            double max_time,
            bool disjoint_splitting,
            const ReservationTable *rt
        );

    private:
        SpaceTimeAStar st_a_star_;
        std::mt19937 generator_;

        vector<Constraint> find_conflict(vector<Path> &paths, bool find_random, bool disjoint_splitting);
        vector<Constraint> split_conflict(vector<Path> &paths, vector<int>& agent_ids, Conflict& conflict, bool disjoint_splitting);
        void add_constraint(ReservationTable& rt, Conflict& c, bool reverse = false);
        vector<int> populate_reservation_table(ReservationTable& rt, CTNode& node, ConstraintTree& tree, int agent_id);
        bool low_level(CTNode &ct_node, ConstraintTree &tree, Agent &agent, ReservationTable& rt, int max_length);
        bool low_level_with_disjoint_splitting(CTNode &ct_node, ConstraintTree &tree, vector<Agent>& agents, ReservationTable& rt, int max_length);
        Path find_new_path(CTNode &ct_node, ConstraintTree &tree, int node_id, Agent &agent, ReservationTable& rt, int max_length);
        void print_node(CTNode &ct_node);
        void print_constraint(Constraint &constraint);
        int random_int(int max_value);
        bool is_point_at_time(Path& path, int point, int time);
        vector<Path> mapf_(
            vector<Agent> &agents,
            int max_length,
            double max_time,
            bool disjoint_splitting,
            ReservationTable &rt
        );
};
