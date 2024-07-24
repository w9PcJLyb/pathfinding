#pragma once

#include "grid.h"
#include "reservation_table.h"
#include "unordered_map"


class ResumableAStar {

    typedef pair<double, int> key;
    typedef priority_queue<key, vector<key>, std::greater<key>> Queue;

    struct Node {
        double distance;
        double f;
        bool closed;

        Node() : distance(-1), f(0), closed(false) {};

        void clear() {
            distance = -1;
            f = 0;
            closed = false;
        }
    };

    public:
        AbsGraph* graph;
        ResumableAStar(AbsGraph* graph, int start, int end);

        double distance(int node_id);  // true distance from the start to this node

    private:
        int start_, end_;
        vector<Node> nodes_;
        Queue openset_;
        void search(int node_id);
        void clear();
};


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
        vector<int> reconstruct_path(int start, Node* node);
        vector<int> find_path_(
            int start_time,
            int start,
            int goal,
            int search_depth,
            ResumableAStar &rra,
            const ReservationTable &rt
        );
};
