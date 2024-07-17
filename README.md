# pathfinding

Implementation of some pathfinding algorithms.

Currently implemented:

| Algorithm   | Class name  | Finds the shortest path on an unweighted graph | Finds the shortest path on a weighted graph |
| ----------- | ----------- |----------- | ----------- |
| Depth-first search | DFS | False | False |
| Breadth-first search | BFS | True | False |
| Bidirectional Breadth-first search | BiBFS | True | False |
| Dijkstra | Dijkstra | True | True |
| Bidirectional Dijkstra | BiDijkstra | True | True |
| A* | AStar | True | True |
| Bidirectional A* | BiAStar | True | True |

There are several types of graphs available:

 - Graph - Directed or undirected graph
 - Grid - Two-dimensional grid
 - Grid3D - Three-dimensional grid
 - HexGrid - Hexagonal grid

# Installation

1. Setup virtual environment

2. Install Cython, it is needed to wrap the C++ code:

    ```bash
    pip install cython
    ```

3. Clone this repository and install pathfinding from the local filesystem:

    ```bash
    pip install pathfinding/
    ```

# Examples

Example 1. Weighted directed graph and Dijkstra

```python
from w9_pathfinding import Graph, Dijkstra

graph = Graph(num_vertices=4)
graph.add_edges(
    [
        (0, 1, 1),  # start, end, cost
        (0, 2, 3),
        (0, 3, 4),
        (1, 3, 1),
        (2, 3, 1),
    ]
)

dijkstra = Dijkstra(graph)
path = dijkstra.find_path(start=0, end=3)
print(path)  # [0, 1, 3]
```

Example 2. Two-dimensional grid and A*

```python
from w9_pathfinding import Grid, AStar, DiagonalMovement

grid = Grid(
    weights =[
        # -1 - unwalkable cell
        # >= 0 - walkable, value is the cost of moving to this cell
        [1,  1,  1, -1],
        [-1, 1,  1, -1],
        [1,  1, -1, -1],
        [1,  1,  1,  1],
    ],
    diagonal_movement=DiagonalMovement.never,
    passable_left_right_border=False,
    passable_up_down_border=False,
)

astar = AStar(grid)
path = astar.find_path(start=(0, 0), end=(3, 3))
print(path)  # [(0, 0), (1, 0), (1, 1), (1, 2), (1, 3), (2, 3), (3, 3)]
```
