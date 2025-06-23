Pathfinding
==================

Pathfinding is the process of finding the shortest or most efficient path between
two points in a defined environment.

.. list-table:: Pathfinding algorithms
   :header-rows: 1

   * - Algorithm
     - Class
     - Optimal
   * - Depth-first search
     - :ref:`DFS`
     - False
   * - Greedy Best-First Search
     - :ref:`GBS`
     - False
   * - Breadth-first search
     - | :ref:`BFS`
       | :ref:`BiBFS`
       | :ref:`ResumableBFS`
     - | True (only in an
       | unweighted environment)
   * - Dijkstra
     - | :ref:`Dijkstra`
       | :ref:`BiDijkstra`
       | :ref:`ResumableDijkstra`
     - True
   * - A*
     - | :ref:`AStar`
       | :ref:`BiAStar`
     - True
   * - Iterative deepening A*
     - :ref:`IDAStar`
     - True

BFS
------------------

.. autoclass:: w9_pathfinding.pf.BFS
   :noindex:
   :exclude-members: env

BiBFS
------------------

.. autoclass:: w9_pathfinding.pf.BiBFS
   :noindex:
   :exclude-members: env

DFS
------------------

.. autoclass:: w9_pathfinding.pf.DFS
   :noindex:
   :exclude-members: env

Dijkstra
------------------

.. autoclass:: w9_pathfinding.pf.Dijkstra
   :noindex:
   :exclude-members: env

BiDijkstra
------------------

.. autoclass:: w9_pathfinding.pf.BiDijkstra
   :noindex:
   :exclude-members: env

AStar
------------------

.. autoclass:: w9_pathfinding.pf.AStar
   :noindex:
   :exclude-members: env

BiAStar
------------------

.. autoclass:: w9_pathfinding.pf.BiAStar
   :noindex:
   :exclude-members: env

GBS
------------------

.. autoclass:: w9_pathfinding.pf.GBS
   :noindex:
   :exclude-members: env

IDAStar
------------------

.. autoclass:: w9_pathfinding.pf.IDAStar
   :noindex:
   :exclude-members: env

ResumableBFS
------------------

.. autoclass:: w9_pathfinding.pf.ResumableBFS
   :noindex:
   :exclude-members: env

ResumableDijkstra
------------------

.. autoclass:: w9_pathfinding.pf.ResumableDijkstra
   :noindex:
   :exclude-members: env
