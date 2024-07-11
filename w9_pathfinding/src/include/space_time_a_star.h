#pragma once

#include "grid.h"


class AStarBackwardSearch {
    // Backwards search for space-time A* algorithm

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
        AStarBackwardSearch(AbsGraph* graph);
        ~AStarBackwardSearch();

        void set_direction(int start, int end);
        double distance(int node_id);

    private:
        int start_, end_;
        vector<Node> nodes_;
        vector<Node*> workset_;
        Queue openset_;
        void search(int node_id);
        void clear();
};


class SpaceTimeAStar : public AbsPathFinder {
    // Implementation of space-time A* algorithm from 
    // https://www.davidsilver.uk/wp-content/uploads/2020/03/coop-path-AIWisdom.pdf

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

        vector<int> find_path(int start, int end);
        vector<int> find_path(int start, int end, int max_steps);

    private:
        AStarBackwardSearch backward_search_;
        vector<int> reconstruct_path(int start, Node* node); 
};
