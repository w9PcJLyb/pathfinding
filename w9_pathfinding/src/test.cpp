#include <iostream>
#include "include/graph.h"
#include "include/grid.h"
#include "include/dfs.h"
#include "include/bfs.h"
#include "include/bi_bfs.h"
#include "include/dijkstra.h"
#include "include/bi_dijkstra.h"
#include "include/a_star.h"
#include "include/bi_a_star.h"
#include "include/hc_a_star.h"


void print_path(vector<int> nodes) {
    for (int node_id: nodes) {
       cout << node_id << " ";
    }
    cout << endl;
}

void print_path(Grid &grid, vector<int> nodes) {
    for (int node_id: nodes) {
       cout << grid.get_coordinates(node_id) << " ";
    }
    cout << endl;
}

void test_graph() {
    cout << "\nTest graph" << endl;

    Graph graph(6, true);

    graph.add_edge(0, 1, 1);
    graph.add_edge(0, 2, 3);
    graph.add_edge(0, 3, 1);
    graph.add_edge(1, 2, 1);
    graph.add_edge(2, 3, 4);
    graph.add_edge(2, 5, 1);
    graph.add_edge(3, 2, 4);
    graph.add_edge(3, 4, 2);
    graph.add_edge(4, 5, 1);

    int start_node = 0;
    int end_node = 5;

    cout << "\nFinding path in the graph from " << start_node << " to " << end_node << endl;

    vector<int> min_vertices_path = {0, 2, 5};
    vector<int> min_distance_path = {0, 1, 2, 5};

    DFS dfs(&graph);
    BFS bfs(&graph);
    BiBFS bibfs(&graph);
    Dijkstra dijkstra(&graph);
    BiDijkstra bidijkstra(&graph);

    vector<pair<std::string, AbsPathFinder*>> finders = {
        {"DFS", &dfs},
        {"BFS", &bfs},
        {"Bidirectional BFS", &bibfs},
        {"Dijkstra", &dijkstra},
        {"Bidirectional Dijkstra", &bidijkstra}
    };

    for (auto &[name, finder] : finders) {
        vector<int> path = finder->find_path(start_node, end_node);
        cout << name << ": ";
        print_path(path);
    }
}

void test_grid() {
    cout << "\nTest grid" << endl;

    Grid grid(4, 5);
    grid.passable_left_right_border = false;
    grid.passable_up_down_border = false;
    grid.set_diagonal_movement(0);

    grid.add_obstacle(grid.get_node_id({1, 2}));
    grid.add_obstacle(grid.get_node_id({2, 1}));
    grid.add_obstacle(grid.get_node_id({2, 2}));

    cout << "Obstacle map:" << endl;
    grid.show_obstacle_map();

    cout << "neighbours (1, 1) :: ";
    for (auto &[n, cost]: grid.get_neighbours(grid.get_node_id({1, 1}))) {
        cout << grid.get_coordinates(n) << " ";
    }
    cout << endl;

    cout << "neighbours (1, 0) :: ";
    for (auto &[n, cost]: grid.get_neighbours(grid.get_node_id({1, 0}))) {
        cout << grid.get_coordinates(n) << " ";
    }
    cout << endl;

    Grid::Point start = {0, 0}, end = {3, 3};
    int start_node = grid.get_node_id(start);
    int end_node = grid.get_node_id(end);
    cout << "\nFinding path in the grid from " << start << " to " << end << endl;

    DFS dfs(&grid);
    BFS bfs(&grid);
    BiBFS bibfs(&grid);
    Dijkstra dijkstra(&grid);
    BiDijkstra bidijkstra(&grid);
    AStar astar(&grid);
    BiAStar biastar(&grid);
    HCAStar stastar(&grid);

    vector<pair<std::string, AbsPathFinder*>> finders = {
        {"DFS", &dfs},
        {"BFS", &bfs},
        {"Bidirectional BFS", &bibfs},
        {"Dijkstra", &dijkstra},
        {"Bidirectional Dijkstra", &bidijkstra},
        {"A*", &astar},
        {"Bidirectional A*", &biastar},
        {"Space-Time A*", &stastar}
    };

    vector<int> path;
    for (auto &[name, finder] : finders) {
        path = finder->find_path(start_node, end_node);
        cout << name << ": ";
        print_path(grid, path);
    }

    cout << "\nSet passable_left_right_border = true" << endl;
    grid.passable_left_right_border = true;
    path = astar.find_path(start_node, end_node);
    cout << "A*: ";
    print_path(grid, path);

    cout << "\nSet diagonal_movement = 3 (always)" << endl;
    grid.set_diagonal_movement(3);
    path = astar.find_path(start_node, end_node);
    cout << "A*: ";
    print_path(grid, path);

    cout << "\nSet passable_up_down_border = true" << endl;
    grid.passable_up_down_border = true;
    path = astar.find_path(start_node, end_node);
    cout << "A*: ";
    print_path(grid, path);
}

int main() {
    test_graph();
    test_grid();
    return 0;
}
