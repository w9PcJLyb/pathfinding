#pragma once

#include "core.h"


class ResumableSearch {
    public:
        AbsGraph* graph;
        ResumableSearch(AbsGraph* graph, int start, bool reverse=false) : graph(graph), start_(start), reverse_(reverse) {};
        virtual ~ResumableSearch() {};

        int start_node() {return start_;};
        virtual void set_start_node(int start) = 0;
        virtual double distance(int node_id) = 0;
        virtual Path find_path(int node_id) = 0;

    protected:
        int start_;
        bool reverse_;
};


class ResumableBFS : public ResumableSearch {
    struct Node {
        double distance;
        int parent;

        Node() : distance(-1), parent(-1) {};

        void clear() {
            distance = -1;
            parent = -1;
        }
    };

    public:
        ResumableBFS(AbsGraph* graph, int start, bool reverse=false);
        double distance(int node_id);
        Path find_path(int node_id);
        void set_start_node(int start);

    private:
        vector<Node> nodes_;
        std::queue<int> openset_;
        void search(int node_id);
        Path reconstruct_path(int node_id);
        void clear();
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
        ResumableDijkstra(AbsGraph* graph, int start, bool reverse=false);
        double distance(int node_id);
        Path find_path(int node_id);
        void set_start_node(int start);

    private:
        vector<Node> nodes_;
        Queue openset_;
        void search(int node_id);
        void clear();
        Path reconstruct_path(int node_id);
};


class ResumableAStar : public ResumableSearch {

    typedef pair<double, int> key;
    typedef priority_queue<key, vector<key>, std::greater<key>> Queue;

    struct Node {
        double distance;
        int parent;
        double f;
        bool closed;

        Node() : distance(-1), parent(-1), f(0), closed(false) {};

        void clear() {
            distance = -1;
            parent = -1;
            f = 0;
            closed = false;
        }
    };

    public:
        ResumableAStar(AbsGraph* graph, int start, bool reverse=false);
        double distance(int node_id);
        Path find_path(int node_id);
        void set_start_node(int start);

    private:
        int end_ = -1;
        vector<Node> nodes_;
        Queue openset_;
        void search(int node_id);
        void clear();
        Path reconstruct_path(int node_id);
};
