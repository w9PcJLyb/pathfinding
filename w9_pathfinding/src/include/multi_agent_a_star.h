#pragma once

#include "core.h"
#include "resumable_search.h"
#include "space_time_a_star.h"

namespace maas {

    struct Agent {
        int start, goal;
        ResumableSearch *rrs;

        Agent(int start, int goal, ResumableSearch *rrs) : start(start), goal(goal), rrs(rrs) {};
    };

    struct Node {
        int parent, time;
        double distance;
        vector<int> positions;
        
        Node(int parent, int time, double distance, vector<int> &positions) :
            parent(parent),
            time(time),
            distance(distance),
            positions(positions) {};
    };

    typedef pair<double, int> key;
    typedef priority_queue<key, vector<key>, std::greater<key>> Queue;
    typedef vector<Node> Tree;

    class State {
        public:
            State(AbsGraph* graph, int node_id, Tree& tree, vector<int>& goal, int time, ReservationTable& rt);

            pair<vector<int>, double> next();
            bool allowed(vector<int>& positions, vector<int>& next_positions, bool edge_collision);

        private:
            vector<vector<pair<int, double>>> neighbors_;
            vector<int> next_positions_;
            vector<int> ids_;
            bool update_indexes_();
    };

    class MultiAgentAStar : public AbsMAPF {

        public:
            AbsGraph* graph;
            MultiAgentAStar(AbsGraph* graph);

            vector<vector<int>> mapf(vector<int> starts, vector<int> goals);
            vector<vector<int>> mapf(vector<int> starts, vector<int> goals, double max_time, const ReservationTable *rt);

        private:
            SpaceTimeAStar st_a_star_;
            vector<vector<int>> mapf_(vector<Agent> &agents, double max_time, ReservationTable &rt);
            vector<vector<int>> reconstruct_paths(int node_id, Tree& tree);
            void print_node(Node& node);
            std::string positions_to_string(vector<int>& positions);
            double heuristic(vector<int>& positions, vector<Agent> &agents);
    };

}
