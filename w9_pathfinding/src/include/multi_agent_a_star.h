#pragma once

#include "core.h"
#include "resumable_search.h"
#include "space_time_a_star.h"

namespace maas {

    struct Agent {
        int start, goal;
        ResumableSearch *rrs;

        Agent(int start, int goal, ResumableSearch *rrs) : start(start), goal(goal), rrs(rrs) {};
        ~Agent() {delete rrs;};
    };

    struct StandardNode {
        int parent, time;
        double distance;
        vector<int> positions;
        
        StandardNode(int parent, int time, double distance, vector<int> &positions) :
            parent(parent),
            time(time),
            distance(distance),
            positions(positions) {};
    };

    struct ODNode {
        int parent, standard_parent, time;
        double distance;
        vector<int> positions;

        ODNode(int parent, int standard_parent, int time, double distance, vector<int> &positions) :
            parent(parent),
            standard_parent(standard_parent),
            time(time),
            distance(distance),
            positions(positions) {};
    };

    typedef pair<double, int> key;
    typedef priority_queue<key, vector<key>, std::greater<key>> Queue;

    class MAState {
        public:
            MAState(AbsGraph* graph, int node_id, vector<StandardNode>& tree, vector<int>& goal, int time, ReservationTable& rt);

            pair<vector<int>, double> next();
            bool allowed(vector<int>& positions, vector<int>& next_positions, bool edge_collision);

        private:
            vector<vector<pair<int, double>>> neighbors_;
            vector<int> next_positions_;
            vector<int> ids_;
            bool update_indexes_();
    };

    class MultiAgentAStar : public AbsMAPF {
        // A* with Operator Decomposition
        // Standley, T.S.: Finding optimal solutions to cooperative pathfinding problems.
        // In: AAAI Conference on Artificial Intelligence. pp. 173–178 (2010)

        public:
            AbsGraph* graph;
            MultiAgentAStar(AbsGraph* graph);

            vector<Path> mapf(vector<int> starts, vector<int> goals);
            vector<Path> mapf(vector<int> starts, vector<int> goals, double max_time, bool operator_decomposition, const ReservationTable *rt);

        private:
            SpaceTimeAStar st_a_star_;
            vector<Path> mapf_standard(vector<Agent> &agents, double max_time, ReservationTable &rt);
            vector<Path> mapf_od(vector<Agent> &agents, double max_time, ReservationTable &rt);
            vector<Path> reconstruct_paths(int node_id, vector<StandardNode>& tree);
            vector<Path> reconstruct_paths(int node_id, vector<ODNode>& tree);
            void print_node(StandardNode& node);
            void print_node(ODNode& node);
            std::string positions_to_string(vector<int>& positions);
            double heuristic(vector<int>& positions, vector<Agent> &agents);
    };

}
