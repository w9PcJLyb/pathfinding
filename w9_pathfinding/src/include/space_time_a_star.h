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
        vector<int> find_path(int start, int end);
        vector<int> find_path(int start, int end, int search_depth, const ReservationTable *rt);
        pair<vector<int>, double> find_path(
            int start_time,
            int start,
            int goal,
            int search_depth,
            ResumableSearch *rrs,
            const ReservationTable *rt
        );

    private:
        AbsGraph* reversed_graph_;
        vector<int> reconstruct_path(int start, Node* node);
};
