Environments
=============

In w9-pathfinding, an environment represents the world or space in which agents
move and paths are found. In simple terms, an environment is a collection of nodes and edges,
where each node is a possible location and each edge defines the cost
to move between nodes. This library includes several types of environments:

.. toctree::
   :maxdepth: 1

   Graph
   Grid
   Grid3D
   HexGrid

One of the most powerful features of w9-pathfinding is its environment polymorphism.
All pathfinding and MAPF (Multi-Agent Pathfinding) algorithms operate independently
of the internal structure of the environment. Whether it's a graph, 2D/3D grid, or hex map —
whether it's weighted or not — if it behaves like a collection of nodes and edges, it just works.

This design enables:

- Reusability: All algorithms can be reused across different environments without modification.
- Decoupling: Algorithms focus purely on pathfinding logic and don't depend on specific
  map implementations.
- Extensibility: New environment types can be added with minimal effort.

As long as the environment implements a simple C++ interface, it becomes a fully compatible
component of the pathfinding ecosystem. This means you can define your own environment
type, implement the required methods, and immediately use the full power of the pathfinding
and MAPF algorithms provided by this library.

.. dropdown:: Environment interface in C++

    Internally, environments in the C++ backend are represented as collections of nodes,
    each identified by an integer node_id from 0 to size() - 1.

    To make an environment compatible with the system, you need
    to implement the following methods:

    - **size**

      .. code-block:: c++

        size_t size();

      Returns the total number of nodes in the environment.

    - **get_neighbors**

      .. code-block:: c++

        std::vector<std::pair<int, double>> get_neighbors(
            int node_id, bool reversed = false, bool include_self = false
        );

      Given a node_id, returns a list of neighboring nodes and the cost to reach each of them.

      Parameters:

      * node_id - the ID of the node whose neighbors are requested
      * reversed - if true, returns neighbors with all edges reversed
        (used in bidirectional search algorithms like Bi-A*)
      * include_self - if true, includes a self-loop to the node itself,
        if it exists (used in MAPF to allow the "pause" action)

    - **estimate_distance**

      .. code-block:: c++

        double estimate_distance(int n1, int n2);

      Returns an estimate of the minimum possible distance
      between two nodes. This is typically used as a heuristic in A*-like algorithms.

      This method is optional. If an environment does not implement it,
      A*-based algorithms (which rely on heuristics) will not be available for that environment.
