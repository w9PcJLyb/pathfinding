# W9-Pathfinding

W9-Pathfinding is a versatile pathfinding library written in C++ with a Python interface provided by Cython. It offers a variety of pathfinding algorithms for navigating different types of maps, including grids, graphs, and 3D spaces. The library includes both classic pathfinding algorithms and multi-agent pathfinding algorithms.

[![Apache-2.0 license](https://img.shields.io/github/license/w9PcJLyb/pathfinding)](https://github.com/w9PcJLyb/pathfinding/blob/main/LICENSE)
[![PyPI](https://img.shields.io/pypi/v/w9-pathfinding)](https://pypi.org/project/w9-pathfinding/)

### Key Features:

- Pathfinding Algorithms: Implements classical algorithms like BFS, Dijkstra, and A*, as well as multi-agent pathfinding algorithms like CBS and ICTS.
- Multiple Map Types: Works with different map structures, including Graphs (directed and undirected), Grids (2D and 3D), Hexagonal Grids (pointy top and flat top).
- Weighted and Unweighted Maps: Supports both weighted and non-weighted maps (graphs and grids).
- Pathfinding with Dynamic Obstacles: Allows pathfinding in environments where some agents are dynamic obstacles—you know their paths but cannot control them. It computes optimal paths for the remaining agents while avoiding collisions with both dynamic obstacles and each other.

### Why "W9"?

The name w9-pathfinding comes from the **weighted 9-connected grid** you can use for **pathfinding** in the library. The "W" stands for Weighted, indicating that the library supports both weighted and non-weighted maps, where movement costs may vary. The "9" refers to the 9 possible movement directions: the 4 cardinal directions (North, South, East, West), the 4 diagonal directions (NE, SE, SW, NW), and a center movement (staying in place, which is crucial in multi-agent scenarios).

# Pathfinding

Pathfinding is the problem of finding the best route between two points.

<p align="left">
    <img src="https://raw.githubusercontent.com/w9PcJLyb/pathfinding/refs/heads/main/images/pf_grid.png" width="128"/>
</p>

This repository includes several pathfinding algorithms:

| Algorithm   | Class name  | Optimal |
| ----------- | ----------- |----------- |
| Depth-first search | DFS | False |
| Best-first search | GBS | False |
| Breadth-first search | BFS | True (only in an unweighted graph) |
| Bidirectional Breadth-first search | BiBFS | True (only in an unweighted graph) |
| Dijkstra | Dijkstra | True |
| Bidirectional Dijkstra | BiDijkstra | True |
| A* | AStar | True |
| Bidirectional A* | BiAStar | True |
| Iterative deepening A* | IDAStar | True |

Example:

```python
from w9_pathfinding.envs import Graph
from w9_pathfinding.pf import Dijkstra

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
path = dijkstra.find_path(start=0, goal=3)
print(path)  # [0, 1, 3]
```

# Multi-Agent Path Finding (MAPF)

Multi-Agent Path Finding (MAPF) is the problem of finding collision-free paths for a group of agents from their location to an assigned target.

<p align="left">
    <img src="https://raw.githubusercontent.com/w9PcJLyb/pathfinding/refs/heads/main/images/mapf_hex.gif"/>
</p>

Implemented algorithms:

| Algorithm | Class name | Optimal | Complete |
| ----------- | ----------- |----------- | ----------- |
| Hierarchical Cooperative A* | HCAStar | False | False |
| Windowed Hierarchical Cooperative A* | WHCAStar | False | False |
| Conflict Based Search | CBS | True | True |
| Increasing Cost Tree Search | ICTS | True (only in an unweighted graph) | True |
| A* with Operator Decomposition | MultiAgentAStar | True | True |

Here optimality means that the algorithm can find the optimal solution in terms of Sum-of-costs function.

Example:

```python
from w9_pathfinding.envs import Grid
from w9_pathfinding.mapf import WHCAStar

grid = Grid(
    # -1 - unwalkable cell
    # >= 0 - walkable, value is the cost of moving to this cell
    weights =[
        [1,  1,  1, -1],
        [-1, 1,  1, -1],
        [1,  1, -1, -1],
        [1,  1,  1,  1],
    ],
    edge_collision=True, # head to head collisions are not allowed
)

whcastar = WHCAStar(grid)
paths = whcastar.mapf(starts=[(0, 0), (1, 1)], goals=[(2, 0), (1, 0)])
print(paths)  # [[(0, 0), (1, 0), (2, 0)], [(1, 1), (1, 1), (1, 0)]]
```

# Pathfinding with dynamic obstacles

To manage dynamic obstacles in pathfinding, a ReservationTable can be used. This data structure tracks the availability of each cell or edge over time, indicating whether it is free or reserved. In the case of the single-agent pathfinding problem with dynamic obstacles, there is a specialized version of the A* algorithm known as Space-Time A* (SpaceTimeAStar).

Let's look at a simple example. We have three agents: Agent 0, Agent 1, and Agent 2. Agent 0 has a predetermined path that we cannot change, this agent acts as a dynamic obstacle. Agents 1 and 2 each have a starting point and a destination, and we want to find paths for both agents while ensuring they do not collide with each other or with Agent 0. We can achieve this by calling Space-Time A* twice, updating the ReservationTable between the calls:

```python
from w9_pathfinding.envs import Grid
from w9_pathfinding.mapf import SpaceTimeAStar, ReservationTable

grid = Grid(width=5, height=4, edge_collision=True)
grid.add_obstacle((1, 1))  # static obstacle

path0 = [(0, 1), (0, 0), (1, 0), (2, 0), (3, 0), (3, 1)]  # dynamic obstacle
start1, goal1 = (0, 2), (2, 1)  # agent 1
start2, goal2 = (0, 0), (2, 0)  # agent 2

rt = ReservationTable(grid)
rt.add_path(path0, reserve_destination=True)

astar = SpaceTimeAStar(grid)

path1 = astar.find_path(start1, goal1, reservation_table=rt)
rt.add_path(path1, reserve_destination=True)

path2 = astar.find_path(start2, goal2, reservation_table=rt)

print(path1)  # [(0, 2), (1, 2), (2, 2), (2, 1)]
print(path2)  # [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (3, 0), (2, 0)]
```

<p align="left">
    <img src="https://raw.githubusercontent.com/w9PcJLyb/pathfinding/refs/heads/main/images/dynamic_obstacle_1.gif"/>
</p>

This approach works quickly and often finds reasonably good solutions. However, in some cases, it may find solutions that are far from optimal or may not find a solution at all, when one agent prevents any path for another agent. An alternative approach is to use Multi-Agent Pathfinding (MAPF) algorithms, which allow us to find paths for both agents simultaneously. Since all MAPF algorithms in this repository are designed to work with the ReservationTable, we can find an optimal solution while taking dynamic obstacles into account:

```python
from w9_pathfinding.mapf import CBS

rt = ReservationTable(grid)
rt.add_path(path0, reserve_destination=True)

cbs = CBS(grid)
paths = cbs.mapf([start1, start2], [goal1, goal2], reservation_table=rt)

print(paths[0])  # [(0, 2), (1, 2), (2, 2), (2, 2), (2, 1)]
print(paths[1])  # [(0, 0), (1, 0), (2, 0), (2, 1), (2, 0)]
```

<p align="left">
    <img src="https://raw.githubusercontent.com/w9PcJLyb/pathfinding/refs/heads/main/images/dynamic_obstacle_2.gif"/>
</p>

# Types of graphs

There are several types of graphs available:

 - Graph - Generic graph, directed or undirected
 - Grid - Two-dimensional grid
 - Grid3D - Three-dimensional grid
 - HexGrid - Hexagonal grid

Any algorithm can work with any type of graph. But there are a few limitations:

1. Algorithms with a heuristic function (AStar, BiAStar, IDAStar, GBS) will work with generic graph only if coordinates are provided for each vertex. Coordinates can be added using the `set_coordinates` method.
2. An undirected generic graph does not support `edge_collision` option. You still can use MAPF algorithms with this kind of graph, but it's impossible right now to mark head to head collisions as illegal actions.

# Visualization

Visualization is only available for Grid and HexGrid. To use visualization, you need to install `matplotlib`.

Example:

```python
from w9_pathfinding.envs import HexGrid
from w9_pathfinding.visualization import plot_grid, animate_grid

grid = HexGrid(
    weights =[
        [1,  1,  1, -1],
        [-1, 1,  1, -1],
        [1,  1, -1, -1],
        [1,  1,  1,  1],
    ]
)

agents = [
    {'start': (0, 0), 'goal': (2, 0), 'path': [(0, 0), (1, 0), (2, 0)]},
    {'start': (1, 1), 'goal': (1, 0), 'path': [(1, 1), (1, 1), (1, 0)]},
]

# plot_grid returns a static image useful in the pathfinding problem
fig = plot_grid(grid, agents)

# animate_grid returns an animation useful in the mapf problem
anim = animate_grid(grid, agents)
# HTML(anim.to_html5_video())  # visualize
# anim.save("out.gif", fps=10, dpi=200)  # save as a gif
```

# Installation

The easiest way to install w9-pathfinding is via pip:

```bash
pip install w9-pathfinding
```

Alternatively, you can install it manually:

1. Setup virtual environment (optional but recommended)

2. Install Cython:

    ```bash
    pip install cython
    ```

3. Build the Cython extensions:

    ```bash
    python setup.py build_ext --inplace
    ```

4. Finally, install the package:

    ```bash
    pip install -e .
    ```

Note: If you are on Linux, you might need to install some basic build tools to compile the Cython extensions:

```bash
sudo apt install --reinstall build-essential gcc g++
```

If you are on Windows, you might need to install MSVC.
