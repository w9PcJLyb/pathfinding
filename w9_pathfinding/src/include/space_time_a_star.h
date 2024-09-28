#pragma once

#include "core.h"
#include "resumable_search.h"
#include "reservation_table.h"


class SpaceTimeAStar : public AbsPathFinder {

    struct Node {
        Node* parent;
        int node_id;
        int time;
        double distance;
        double f;

        Node(Node *parent, int node_id, int time, double distance, double f) : 
            parent(parent), node_id(node_id), time(time), distance(distance), f(f) {}
    };

    typedef pair<double, Node*> key;
    typedef priority_queue<key, vector<key>, std::greater<key>> Queue;

    public:
        AbsGraph* graph;
        SpaceTimeAStar(AbsGraph* graph);
        ~SpaceTimeAStar();

        ResumableSearch* reverse_resumable_search(int node_id);
        Path find_path(int start, int end);

        /*
        find_path_with_depth_limit finds a path from the starting point to the goal
        taking into account the reservation table. search_depth is an early stopping
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

    private:
        ResumableSearch* rrs_;

        ResumableSearch* ensure_rrs(ResumableSearch* rrs, int goal);
        Path reconstruct_path(int start, Node* node);
};
