#pragma once

#include "core.h"
#include "space_time_a_star.h"
#include "resumable_search.h"
#include "reservation_table.h"


class HCAStar : public AbsMAPF {
    // Hierarchical Cooperative A*
    // Silver, D. 2005. Cooperative pathfinding. In AIIDE, 117–122.

    public:
        AbsGraph* graph;
        HCAStar(AbsGraph* graph);

        vector<vector<int>> mapf(vector<int> starts, vector<int> goals);
        vector<vector<int>> mapf(
            vector<int> starts,
            vector<int> goals,
            int search_depth,
            bool despawn_at_destination,
            const ReservationTable *rt
        );

    private:
        SpaceTimeAStar st_a_star_;

        void expand_paths(vector<vector<int>> &paths);
};
