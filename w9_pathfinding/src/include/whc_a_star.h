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
        Path path;

        Agent(int start, int goal, ResumableSearch *rrs) : start(start), goal(goal), rrs(rrs) {
            path.push_back(start);
        }
        ~Agent() {delete rrs;};

        int position() {
            return path.back();
        }

        int position(int time) {
            return path[time];
        }

        void add_path(Path& path_) {
            path.insert(path.end(), path_.begin(), path_.end());
        }
    };

    public:
        AbsGraph* graph;
        WHCAStar(AbsGraph* graph);

        vector<Path> mapf(vector<int> starts, vector<int> goals);
        vector<Path> mapf(
            vector<int> starts,
            vector<int> goals,
            int max_length,
            int window_size,
            const ReservationTable *rt
        );

    private:
        SpaceTimeAStar st_a_star_;
        vector<Path> mapf_(
            vector<Agent> &agents,
            int max_length,
            int window_size,
            ReservationTable &rt
        );
};
