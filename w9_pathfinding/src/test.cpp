#include <iostream>
#include "include/graph.h"
#include "include/grid.h"
#include "include/dfs.h"
#include "include/bfs.h"
#include "include/a_star.h"
#include "include/dijkstra.h"
#include "include/bidijkstra.h"


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

    Graph graph(6);
    
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

    vector<int> path;

    DFS dfs(&graph);
    path = dfs.find_path(start_node, end_node);
    cout << "DFS: ";
    print_path(path);

    BFS bfs(&graph);
    path = bfs.find_path(start_node, end_node);
    cout << "BFS: ";
    print_path(path);

    Dijkstra dijkstra(&graph);
    path = dijkstra.find_path(start_node, end_node);
    cout << "Dijkstra: ";
    print_path(path);

    BiDijkstra bidijkstra(&graph);
    path = bidijkstra.find_path(start_node, end_node);
    cout << "BiDijkstra: ";
    print_path(path);
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

    vector<int> path;

    DFS dfs(&grid);
    path = dfs.find_path(start_node, end_node);
    cout << "DFS: ";
    print_path(grid, path);

    BFS bfs(&grid);
    path = bfs.find_path(start_node, end_node);
    cout << "BFS: ";
    print_path(grid, path);

    Dijkstra dijkstra(&grid);
    path = dijkstra.find_path(start_node, end_node);
    cout << "Dijkstra: ";
    print_path(grid, path);

    BiDijkstra bidijkstra(&grid);
    path = bidijkstra.find_path(start_node, end_node);
    cout << "BiDijkstra: ";
    print_path(grid, path);

    AStar astar(&grid);
    path = astar.find_path(start_node, end_node);
    cout << "A*: ";
    print_path(grid, path);

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