#pragma once

#include <memory>
#include "mapf.h"
#include "resumable_search.h"
#include "space_time_a_star.h"

namespace maas {

    class MultiAgentAStar : public AbsMAPF {
        // A* with Operator Decomposition
        // Standley, T.S.: Finding optimal solutions to cooperative pathfinding problems.
        // In: AAAI Conference on Artificial Intelligence. pp. 173–178 (2010)

        public:
            Env* env;
            MultiAgentAStar(Env* env);

            vector<Path> mapf(vector<int> starts, vector<int> goals);
            vector<Path> mapf(
                vector<int> starts,
                vector<int> goals,
                int max_length,
                double max_time,
                bool operator_decomposition,
                const ReservationTable *rt
            );
    };

    struct Agent {
        int start, goal;
        std::unique_ptr<ResumableSearch> rrs;
        double goal_pause_cost;

        Agent(int start, int goal, std::unique_ptr<ResumableSearch> rrs, double goal_pause_cost)
            : start(start), goal(goal), rrs(std::move(rrs)), goal_pause_cost(goal_pause_cost) {}
    };

    struct Node {
        int id, parent, time;
        double distance;
        vector<int> positions;
        
        Node(int id, int parent, int time, double distance, vector<int>& positions) :
            id(id),
            parent(parent),
            time(time),
            distance(distance),
            positions(positions) {};
    };

    typedef vector<Node> Tree;
    typedef pair<double, int> key;
    typedef priority_queue<key, vector<key>, std::greater<key>> Queue;

    class AStarSolver {
        public:
            AStarSolver(Env* env, vector<int>& starts, vector<int>& goals, const ReservationTable* rt);

            vector<Path> find_paths(int max_length, double max_time);

        protected:
            Env* env_;
            const ReservationTable* rt_;
            Tree tree_;
            vector<Agent> agents_;
            int num_agents_;

            void print_node(const Node& node) const;

            bool is_goal(vector<int>& positions);

            // returns a list of free adjacent tiles for each position and the cost of moving to that tile
            vector<vector<pair<int, double>>> get_neighbors(Node& node);

            // returns a path that ends with node_id
            vector<Path> reconstruct_paths(int node_id);

            // creates a new node and adds it to the tree
            Node create_node(int parent, int time, double distance, vector<int> &positions);

            // returns the amount of time that an agent has been at its goal position
            int get_waiting_time(int node_id, int agent_id);

            // the minimum total cost of paths from this position to the goal
            // returns -1 if at least one path is not possible
            double heuristic(vector<int>& positions);

            vector<int> to_hash(vector<int>& positions, int parent);
    };

    class AStarODSolver : AStarSolver {
        public:
            AStarODSolver(Env* env, vector<int>& starts, vector<int>& goals, const ReservationTable* rt);

            vector<Path> find_paths(int max_length, double max_time);

        private:
            vector<pair<int, double>> get_neighbors(Node& node);
            std::unordered_set<int> get_occupied_nodes(Node& node);
    };

}
