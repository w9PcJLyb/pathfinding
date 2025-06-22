#pragma once

#include "env.h"
#include "space_time_a_star.h"
#include "resumable_search.h"
#include "reservation_table.h"


class HCAStar : public AbsMAPF {
    // Hierarchical Cooperative A*
    // Silver, D. 2005. Cooperative pathfinding. In AIIDE, 117–122.

    public:
        Env* env;
        HCAStar(Env* env);

        vector<vector<int>> mapf(vector<int> starts, vector<int> goals);
        vector<vector<int>> mapf(
            vector<int> starts,
            vector<int> goals,
            int max_length,
            const ReservationTable *rt
        );

    private:
        SpaceTimeAStar st_a_star_;
};
