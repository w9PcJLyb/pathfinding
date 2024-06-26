# pathfinding

Implementation of some pathfinding algorithms.

Currently implemented:

- DFS (Depth-first search)
- BFS (Breadth-first search)
- Dijkstra
- BiDijkstra (Bidirectional Dijkstra)
- AStar (A*)

# Installation

1. Setup virtual environment

2. Install Cython, it is needed to wrap the C++ code:

    ```bash
    pip install cython
    ```

3. Install pathfinding:

    ```bash
    python setup.py develop
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

Example 2. 2D Grid and A*

```python
from w9_pathfinding import Grid, AStar, DiagonalMovement

grid = Grid(
    [
        [0, 0, 0, 1],
        [1, 0, 0, 1],
        [0, 0, 1, 1],
        [0, 0, 0, 0],
    ],
    diagonal_movement=DiagonalMovement.never,
    passable_left_right_border=False,
    passable_up_down_border=False,
)

astar = AStar(grid)
path = astar.find_path(start=(0, 0), end=(3, 3))
print(path)  # [(0, 0), (1, 0), (1, 1), (1, 2), (1, 3), (2, 3), (3, 3)]
```
