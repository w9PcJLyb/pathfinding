#pragma once

#include "core.h"
#include "set"
#include "map"
#include "reservation_table.h"
#include "space_time_a_star.h"


namespace icts {

    typedef pair<int, int> dint;

    class MDD {
        // Multi-value Decision Diagram

        public:
            std::map<dint, std::set<dint>> data;

            MDD(AbsGraph* graph, int start, int goal, const ReservationTable* rt);
            void set_depth(int depth);
            int depth();

        private:
            AbsGraph* graph_;
            int start_, goal_, depth_, bfs_depth_;
            const ReservationTable* rt_;
            std::queue<dint> bfs_queue_;
            std::map<dint, std::set<dint>> bfs_tree_;
            std::set<dint> bfs_visited_;

            void generate_mdd(int depth);
            void update_bfs_tree(int depth);
            void add_node(int node_id, int depth, dint& parent);
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
            LowLevel(AbsGraph* graph, vector<int>& starts, vector<int>& goals, const ReservationTable *rt);
            vector<Path> search(vector<int>& costs);

        private:
            AbsGraph* graph_;
            vector<int> starts_, goals_;
            int num_agents_;
            bool edge_collision_;
            vector<MDD> mdd_list_;
            vector<vector<int>> solution_;

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

            vector<Path> mapf(vector<int> starts, vector<int> goals);
            vector<Path> mapf(
                vector<int> starts,
                vector<int> goals,
                int max_length,
                double max_time,
                const ReservationTable *rt
            );

        private:
            SpaceTimeAStar st_a_star_;
    };
}
