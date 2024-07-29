#pragma once

#include "core.h"
#include "space_time_a_star.h"
#include "resumable_search.h"
#include "reservation_table.h"


class WHCAStar : public AbsMAPF {
    // Windowed Hierarchical Cooperative A*
    // Silver, D. 2005. Cooperative pathfinding. In AIIDE, 117–122.

    struct Agent {
        int start, goal;
        ResumableSearch *rrs;
        bool active = true;

        vector<int> full_path;

        Agent(int start, int goal, ResumableSearch *rrs) : start(start), goal(goal), rrs(rrs) {
            full_path.push_back(start);
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
            full_path.insert(full_path.end(), path.begin(), path.end());
        }
    };

    public:
        AbsGraph* graph;
        WHCAStar(AbsGraph* graph);

        vector<vector<int>> mapf(vector<int> starts, vector<int> goals);
        vector<vector<int>> mapf(
            vector<int> starts,
            vector<int> goals,
            int search_depth,
            int window_size,
            bool despawn_at_destination,
            const ReservationTable *rt
        );

    private:
        SpaceTimeAStar st_a_star_;
};
