# About W9-Pathfinding

W9-Pathfinding is a versatile pathfinding library written in C++ with a Python interface provided by Cython. It offers a variety of pathfinding algorithms for navigating different types of maps, including grids, graphs, and 3D spaces. The library includes both classic pathfinding algorithms and multi-agent pathfinding algorithms.

## Key Features:

- Pathfinding Algorithms: Implements classical algorithms like BFS, Dijkstra, and A*, as well as multi-agent pathfinding algorithms like CBS and ICTS.
- Multiple Map Types: Works with different map structures, including Graphs (directed and undirected), Grids (2D and 3D), Hexagonal Grids (pointy top and flat top).
- Weighted and Unweighted Maps: Supports both weighted and non-weighted maps (graphs and grids).
- Pathfinding with Dynamic Obstacles: Allows pathfinding in environments where some agents are dynamic obstacles—you know their paths but cannot control them. It computes optimal paths for the remaining agents while avoiding collisions with both dynamic obstacles and each other.

## Why "W9"?

The name w9-pathfinding comes from the **weighted 9-connected grid** you can use for **pathfinding** in the library. The "W" stands for Weighted, indicating that the library supports both weighted and non-weighted maps, where movement costs may vary. The "9" refers to the 9 possible movement directions: the 4 cardinal directions (North, South, East, West), the 4 diagonal directions (NE, SE, SW, NW), and a center movement (staying in place, which is crucial in multi-agent scenarios).


## License

Apache-2.0 license. See [LICENSE](https://github.com/w9PcJLyb/pathfinding/blob/main/LICENSE) for details.
