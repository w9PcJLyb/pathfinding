Pathfinding
==================

Pathfinding is the process of finding the shortest or most efficient path between
two points in a defined environment.

.. list-table:: Pathfinding algorithms
   :header-rows: 1

   * - Algorithm
     - Class names
     - Optimal
   * - Depth-first search
     - DFS
     - False
   * - Greedy Best-First Search
     - GBS
     - False
   * - Breadth-first search
     - | BFS
       | BiBFS
       | ResumableBFS
     - | True
       | (only in an unweighted graph)
   * - Dijkstra
     - | Dijkstra
       | BiDijkstra
       | ResumableDijkstra
     - True
   * - A*
     - | AStar
       | BiAStar
     - True
   * - Iterative deepening A*
     - IDAStar
     - True

.. note::

  All pathfinding algorithms in w9-pathfinding are environment-agnostic.
  This means they can operate on any :doc:`environment </envs/index>`.

BFS
------------------

.. autoclass:: w9_pathfinding.pf.BFS
   :noindex:
   :exclude-members: graph

BiBFS
------------------

.. autoclass:: w9_pathfinding.pf.BiBFS
   :noindex:
   :exclude-members: graph

DFS
------------------

.. autoclass:: w9_pathfinding.pf.DFS
   :noindex:
   :exclude-members: graph

Dijkstra
------------------

.. autoclass:: w9_pathfinding.pf.Dijkstra
   :noindex:
   :exclude-members: graph

BiDijkstra
------------------

.. autoclass:: w9_pathfinding.pf.BiDijkstra
   :noindex:
   :exclude-members: graph

AStar
------------------

.. autoclass:: w9_pathfinding.pf.AStar
   :noindex:
   :exclude-members: graph

BiAStar
------------------

.. autoclass:: w9_pathfinding.pf.BiAStar
   :noindex:
   :exclude-members: graph

GBS
------------------

.. autoclass:: w9_pathfinding.pf.GBS
   :noindex:
   :exclude-members: graph

IDAStar
------------------

.. autoclass:: w9_pathfinding.pf.IDAStar
   :noindex:
   :exclude-members: graph

ResumableBFS
------------------

.. autoclass:: w9_pathfinding.pf.ResumableBFS
   :noindex:
   :exclude-members: graph

ResumableDijkstra
------------------

.. autoclass:: w9_pathfinding.pf.ResumableDijkstra
   :noindex:
   :exclude-members: graph
