#pragma once

#include "core.h"
#include "resumable_search.h"
#include "reservation_table.h"


class LimitedSearch {   
    
    typedef pair<double, int> key;
    typedef priority_queue<key, vector<key>, std::greater<key>> Queue;

    struct Node {
        int parent;
        int time;
        double distance;
        double f;

        Node() : parent(-1), time(0), distance(-1), f(0) {};

        void clear() {
            parent = -1;
            time = 0;
            distance = -1;
            f = 0;
        }
    };

    public:
        AbsGraph* graph;
        LimitedSearch(AbsGraph* graph);
        ~LimitedSearch();

        vector<int> find_path(int start, int goal, int max_steps);
        vector<int> find_path_to_moving_goal(int start, vector<int> goal, int max_steps);

    private:
        AbsGraph* reversed_graph_;
        ResumableBFS* bfs_;

        vector<Node> nodes_;
        vector<Node*> workset_;
        vector<int> reconstruct_path(int start, int goal); 
        void clear();
};
