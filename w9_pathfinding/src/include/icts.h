#pragma once

#include "core.h"
#include "set"
#include "map"
#include "reservation_table.h"
#include "space_time_a_star.h"


namespace icts {

    typedef pair<int, int> dint;

    class ResumableBFS {

        public:
            int start, goal;
            std::map<dint, vector<int>> data;

            ResumableBFS(AbsGraph* graph, int start, int goal, const ReservationTable* rt);
            void set_depth(int depth);
            int depth() {return depth_;};

        private:
            AbsGraph* graph_;
            int depth_;
            const ReservationTable* rt_;
            std::queue<dint> queue_;
            std::set<dint> visited_;

            void update_data(int depth);
            void add_record(int node_id, int time, int parent);
    };

    class MDD {
        // Multi-value Decision Diagram

        public:
            int start, goal, depth;

            // (node_id, time) -> list of available moves at time + 1
            std::map<dint, std::unordered_set<int>> data;

            MDD() : start(-1), goal(-1), depth(-1) {};
            MDD(int start, int goal, int depth) : start(start), goal(goal), depth(depth) {};
            MDD(ResumableBFS& bfs);
            void print(AbsGraph* graph);
    };

    struct MDD2 {
        // Multi-value Decision Diagram for two agents

        public:
            dint starts, goals, depths;
            int depth;
            bool resolved;

            // ((n1, n2), time) -> list of available moves at time + 1
            std::map<pair<dint, int>, vector<dint>> data;

            MDD2(MDD& mdd1, MDD& mdd2, bool edge_collision);
            pair<MDD, MDD> unfold();
            void print(AbsGraph* graph);

        private:
            int prune(dint positions, int time);
    };

    struct ICTNode {
        // Increasing Cost Tree Node

        public:
            vector<int> costs;

            ICTNode(vector<int>& costs);

            std::string to_str();
    };

    class LowLevel {
        // Low-level search

        public:
            LowLevel(AbsGraph* graph, vector<int>& starts, vector<int>& goals, bool ict_pruning, const ReservationTable *rt);
            vector<Path> search(vector<int>& costs);

        private:
            AbsGraph* graph_;
            vector<int> starts_, goals_;
            bool ict_pruning;
            int num_agents_;
            bool edge_collision_;
            vector<ResumableBFS> bfses_;
            vector<MDD> mdds_;
            vector<vector<int>> solution_;

            bool enhanced_pairwise_pruning();
            bool explore(vector<int>& positions, int depth, int target_depth);
            vector<Path> get_paths();
            bool has_collision(vector<int>& positions, vector<int>& next_positions);
    };

    class ICTS : public AbsMAPF {
        // Increasing Cost Tree Search
        // Sharon, G., Stern, R., Goldenberg, M., Felner, A.: The increasing cost tree search
        // for optimal multi-agent pathfinding. Artificial Intelligence 195, 470–495 (2013)

        public:
            AbsGraph* graph;
            ICTS(AbsGraph* graph);
            int num_generated_nodes = 0;
            int num_closed_nodes = 0;

            vector<Path> mapf(vector<int> starts, vector<int> goals);
            vector<Path> mapf(
                vector<int> starts,
                vector<int> goals,
                int max_length,
                double max_time,
                bool ict_pruning,
                const ReservationTable *rt
            );

        private:
            SpaceTimeAStar st_a_star_;
    };
}
