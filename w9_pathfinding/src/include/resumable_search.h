#pragma once

#include "grid.h"


class ResumableSearch {
    public:
        AbsGraph* graph;
        ResumableSearch(AbsGraph* graph, int start) : graph(graph), start_(start) {};
        virtual ~ResumableSearch() {};

        virtual double distance(int node_id) = 0;  // true distance from the start to this node

    private:
        const int start_;
};


class ResumableDijkstra : public ResumableSearch {
    typedef pair<double, int> key;
    typedef priority_queue<key, vector<key>, std::greater<key>> Queue;

    struct Node {
        double distance;
        bool closed;

        Node() : distance(-1), closed(false) {};

        void clear() {
            distance = -1;
            closed = false;
        }
    };

    public:
        ResumableDijkstra(AbsGraph* graph, int start);
        double distance(int node_id);

    private:
        vector<Node> nodes_;
        Queue openset_;
        void search(int node_id);
        void clear();
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

    private:
        int end_ = -1;
        vector<Node> nodes_;
        Queue openset_;
        void search(int node_id);
        void clear();
};
