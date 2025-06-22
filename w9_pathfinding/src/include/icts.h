#pragma once

#include <chrono>
#include "set"
#include "map"
#include "env.h"
#include "reservation_table.h"
#include "space_time_a_star.h"

using namespace std::chrono;


namespace icts {

    typedef pair<int, int> dint;

    class ResumableBFS {

        public:
            int start, goal;
            std::map<dint, vector<int>> data;

            ResumableBFS(Env* env, int start, int goal, const ReservationTable* rt);
            void set_depth(int depth);
            int depth() {return depth_;};

        private:
            Env* env_;
            int depth_;
            const ReservationTable* rt_;
            std::queue<dint> queue_;
            std::set<dint> visited_;

            void update_data(int depth);
            void add_record(int time, int node_id, int parent);
    };

    class MDD {
        // Multi-value Decision Diagram

        public:
            int start, goal, depth;

            // (time, node_id) -> list of available moves at time + 1
            std::map<dint, std::unordered_set<int>> data;

            MDD() : start(-1), goal(-1), depth(-1) {};
            MDD(int start, int goal, int depth) : start(start), goal(goal), depth(depth) {};
            MDD(ResumableBFS& bfs);
            void print(Env* env);
    };

    struct MDD2 {
        // Multi-value Decision Diagram for two agents

        public:
            dint starts, goals, depths;
            int depth;
            bool resolved;

            // (time, (n1, n2)) -> list of available moves at time + 1
            std::map<pair<int, dint>, vector<dint>> data;

            MDD2(MDD& mdd1, MDD& mdd2, bool edge_collision);
            pair<MDD, MDD> unfold();
            void print(Env* env);

        private:
            void prune();
            bool get_result_and_prune(int time, dint positions, std::map<pair<int, dint>, bool>& mem);
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
            LowLevel(
                Env* env,
                vector<int>& starts,
                vector<int>& goals,
                bool ict_pruning,
                const ReservationTable *rt,
                time_point<high_resolution_clock> terminate_time
            );
            vector<Path> search(vector<int>& costs);

        private:
            Env* env_;
            vector<int> starts_, goals_;
            bool ict_pruning_;
            time_point<high_resolution_clock> terminate_time_;
            int num_agents_;
            bool edge_collision_;
            vector<ResumableBFS> bfses_;
            vector<MDD> mdds_;
            vector<vector<int>> solution_;

            bool enhanced_pairwise_pruning();
            vector<Path> find_solution(int target_depth);
            bool explore(int time, vector<int>& positions, int target_depth, std::unordered_set<pair<int, vector<int>>, SpaceTimeHash>& checked);
            vector<vector<int>> get_neighbors(int time, vector<int>& positions);
            vector<Path> get_paths();
    };

    class ICTS : public AbsMAPF {
        // Increasing Cost Tree Search
        // Sharon, G., Stern, R., Goldenberg, M., Felner, A.: The increasing cost tree search
        // for optimal multi-agent pathfinding. Artificial Intelligence 195, 470–495 (2013)

        public:
            Env* env;
            ICTS(Env* env);
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
