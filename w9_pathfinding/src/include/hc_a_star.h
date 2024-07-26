#pragma once

#include "grid.h"
#include "resumable_search.h"
#include "reservation_table.h"
#include "unordered_map"


class HCAStar : public AbsMAPF {
    // Hierarchical Cooperative A*
    // Silver, D. 2005. Cooperative pathfinding. In AIIDE, 117–122.

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
        HCAStar(AbsGraph* graph);
        ~HCAStar();

        vector<int> find_path(int start, int end);
        vector<int> find_path(int start, int end, int search_depth, const ReservationTable *rt);
        vector<vector<int>> mapf(vector<int> starts, vector<int> goals);
        vector<vector<int>> mapf(
            vector<int> starts,
            vector<int> goals,
            int search_depth,
            bool despawn_at_destination,
            const ReservationTable *rt
        );

    protected:
        AbsGraph* reversed_graph_;
        ResumableSearch* reverse_resumable_search(int node_id);
        vector<int> reconstruct_path(int start, Node* node);
        vector<int> find_path_(
            int start_time,
            int start,
            int goal,
            int search_depth,
            ResumableSearch *rrs,
            const ReservationTable &rt
        );
};
