Usage
===============

Pathfinding in a graph
------------------------------------

For this basic usage example, we'll demonstrate how to perform pathfinding
in the simplest environment: a directed :doc:`graph </envs/Graph>`.

First, we create a small weighted directed graph:

.. code-block:: python

    from w9_pathfinding.envs import Graph

    graph = Graph(
        num_vertices=4,
        edges=[(0, 1, 2.0), (0, 2, 0.1), (1, 3), (2, 3)],  # [(start, end, weight), ...]
    )

This graph looks like the following:

.. mermaid::

     graph LR;
         0 -- 2.0 --> 1;
         0 -- 0.1 -->2;
         1 -- 1 --> 3;
         2 -- 1 -->3;

We want to find the optimal path from node `0` to node `3`.
Since the graph is **weighted**, we need to choose an algorithm that guarantees
an optimal solution in such environments. One such algorithm is
:ref:`Dijkstra's algorithm <Dijkstra>`:

.. code-block:: python

    from w9_pathfinding.pf import Dijkstra

    finder = Dijkstra(graph)
    path = finder.find_path(0, 3)

    print(path)  # [0, 2, 3]
    print(graph.calculate_cost(path))  # 1.1


Pathfinding in a 2D grid
------------------------------------

Every algorithm in w9-pathfinding works with any :doc:`environment </envs/index>` type,
thanks to its environment-agnostic design. In this example, we'll demonstrate how to
perform pathfinding a :doc:`2D grid </envs/Grid>` environment.

First, let's create a small grid with obstacles:

.. code-block:: python

    from w9_pathfinding.envs import Grid

    grid = Grid(width=4, height=3)
    grid.add_obstacle((1, 1))
    grid.add_obstacle((2, 1))

This grid looks like the following:

.. image:: _static/simple_grid.svg
   :width: 200px

Now, we want to find the optimal path from the top-left corner `(0, 0)` to the node `(3, 1)`.
We can use the same Dijkstra's algorithm as we did for graphs. The syntax is nearly identical:

.. code-block:: python

    from w9_pathfinding.pf import Dijkstra

    finder = Dijkstra(grid)
    path = finder.find_path((0, 0), (3, 1))

    print(path)  # [(0, 0), (1, 0), (2, 0), (3, 0), (3, 1)]
    print(grid.calculate_cost(path))  # 4.0


Resumable Search
------------------------------------

todo

Multi-Agent Pathfinding (MAPF)
------------------------------------

todo

Pathfinding with dynamic obstacles
------------------------------------

todo
