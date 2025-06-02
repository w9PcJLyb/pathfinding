#pragma once

#include <memory>
#include "core.h"
#include "resumable_search.h"
#include "reservation_table.h"


class SpaceTimeAStar {

    struct Node {
        Node* parent;
        int node_id;
        int time;
        double distance;
        double f;

        Node(Node* parent, int node_id, int time, double distance, double f) :
            parent(parent), node_id(node_id), time(time), distance(distance), f(f) {}
    };

    public:
        AbsGraph* graph;
        SpaceTimeAStar(AbsGraph* graph);

        std::unique_ptr<ResumableSearch> reverse_resumable_search(int node_id);
        Path find_path(int start, int end);

        /*
        finds a path from the starting point to the goal. search_depth is an early stopping
        condition, for quick responses when the entire path is not necessary.
        Thus if the shortest path is longer than search_depth, the function returns
        only the first search_depth elements of the path.
        */
        Path find_path_with_depth_limit(
            int start,
            int goal,
            int search_depth,
            const ReservationTable *rt,  // dynamic obstacles
            ResumableSearch *rrs = nullptr,  // reverse search for a near perfect heuristic
            int min_terminal_time = 0,  // the search must continue until we reach this time.
            int start_time = 0   // the time of the first step
        );

        /*
        finds a path from the starting point to the goal with a specific length
        */
        Path find_path_with_exact_length(
            int start,
            int goal,
            int length,
            const ReservationTable *rt,
            int start_time = 0
        );

        /*
        finds the optimal path from the starting point to the goal with a
        length less than max_length
        */
        Path find_path_with_length_limit(
            int start,
            int goal,
            int max_length,
            const ReservationTable *rt,
            ResumableSearch* rrs = nullptr,
            int min_terminal_time = 0,
            int start_time = 0
        );

    private:
        std::unique_ptr<ResumableSearch> rrs_;

        ResumableSearch* ensure_rrs(ResumableSearch* rrs, int goal);
        Path reconstruct_path(int start, Node* node);
        Path find_path_with_length_limit__static(int start, int goal, int max_length);
};
