#include "include/components.h"


void dfs_with_order_(AbsGraph *graph, vector<bool> &visited, vector<int> &order, int start) {
    visited[start] = true;
    for (auto& [n, cost] : graph->get_neighbors(start, true)) {
        if (!visited[n])
            dfs_with_order_(graph, visited, order, n);
    }
    order.push_back(start);
}


vector<int> dfs_sort_(AbsGraph *graph) {
    int graph_size = graph->size();

    vector<bool> visited(graph_size, false);
    vector<int> order;

    int offset = 0;
    while (true) {

        int start = -1;
        for (int i = offset; i < graph_size; i++) {
            if (!visited[i]) {
                start = i;
                offset = i + 1;
                break;
            }
        }

        if (start == -1)
            break;

        dfs_with_order_(graph, visited, order, start);
    }

    return order;
}


vector<int> find_component_(AbsGraph *graph, vector<bool> &visited, int start) {
    visited[start] = true;
    vector<int> component = {start};

    vector<int> stack = {start};
    while (!stack.empty()) {
        int x = stack.back();
        stack.pop_back();
        for (auto& [n, cost] : graph->get_neighbors(x)) {
            if (!visited[n]) {
                visited[n] = true;
                component.push_back(n);
                stack.push_back(n);
            }
        }
    }

    return component;
}


vector<vector<int>> find_scc(AbsGraph* graph) {
    // Kosaraju's algorithm

    vector<int> order = dfs_sort_(graph);
    std::reverse(order.begin(), order.end());

    int graph_size = graph->size();
    vector<bool> visited(graph_size, false);
    vector<vector<int>> scc;

    int offset = 0;
    while (true) {

        int start = -1;
        for (int i = offset; i < graph_size; i++) {
            if (!visited[order[i]]) {
                start = order[i];
                offset = i + 1;
                break;
            }
        }

        if (start == -1)
            break;

        scc.push_back(find_component_(graph, visited, start));
    }

    return scc;
} 


vector<vector<int>> find_components(AbsGraph* graph) {

    int graph_size = graph->size();
    vector<bool> visited(graph_size, false);
    vector<vector<int>> components;

    int offset = 0;
    while (true) {

        int start = -1;
        for (int i = offset; i < graph_size; i++) {
            if (!visited[i]) {
                start = i;
                offset = i + 1;
                break;
            }
        }

        if (start == -1)
            break;

        components.push_back(find_component_(graph, visited, start));
    }

    return components;
}
