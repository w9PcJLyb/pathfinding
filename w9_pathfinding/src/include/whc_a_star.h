#pragma once

#include "hc_a_star.h"


class WHCAStar : public HCAStar {
    // Windowed Hierarchical Cooperative A*
    // Silver, D. 2005. Cooperative pathfinding. In AIIDE, 117–122.

    struct Agent {
        int start, goal;
        ResumableAStar rra;
        bool active = true;

        vector<int> full_path;

        Agent(int start, int goal, ResumableAStar &rra) : start(start), goal(goal), rra(rra) {
            full_path.push_back(start);
            if (distance() == -1) {
                // can't reach the goal
                goal = start;
            }
        }

        bool is_moving(size_t time) {
            return time < full_path.size() - 1;
        }

        int position() {
            return full_path.back();
        }

        int position(int time) {
            return full_path[time];
        }

        void add_path(vector<int> &path) {
            if (path.size() < 2)
                return;
            full_path.insert(full_path.end(), path.begin() + 1, path.end());
        }

        double distance() {
            return rra.distance(position());
        }
    };

    public:

        WHCAStar(AbsGraph* graph);

        vector<vector<int>> mapf(
            vector<int> starts,
            vector<int> goals,
            int search_depth,
            int window_size,
            bool despawn_at_destination,
            const ReservationTable *rt
        );
};
