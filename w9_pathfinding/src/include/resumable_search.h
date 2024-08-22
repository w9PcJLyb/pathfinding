#pragma once

#include "grid.h"


class ResumableSearch {
    public:
        AbsGraph* graph;
        ResumableSearch(AbsGraph* graph, int start) : graph(graph), start_(start) {};
        virtual ~ResumableSearch() {};

        virtual double distance(int node_id) = 0;  // true distance from the start to this node
        int start_node();

    protected:
        int start_;
};


class ResumableDijkstra : public ResumableSearch {
    typedef pair<double, int> key;
    typedef priority_queue<key, vector<key>, std::greater<key>> Queue;

    struct Node {
        double distance;
        int parent;
        bool closed;

        Node() : distance(-1), parent(-1), closed(false) {};

        void clear() {
            distance = -1;
            parent = -1;
            closed = false;
        }
    };

    public:
        ResumableDijkstra(AbsGraph* graph, int start);
        double distance(int node_id);
        vector<int> find_path(int node_id);
        void set_start_node(int start);

    private:
        vector<Node> nodes_;
        Queue openset_;
        void search(int node_id);
        void clear();
        vector<int> reconstruct_path(int node_id);
};


class ResumableAStar : public ResumableSearch {

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
        ResumableAStar(AbsGraph* graph, int start);
        double distance(int node_id);
        void set_start_node(int start);

    private:
        int end_ = -1;
        vector<Node> nodes_;
        Queue openset_;
        void search(int node_id);
        void clear();
};
