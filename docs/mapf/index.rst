Multi-Agent Pathfinding
=========================

Multi-Agent Pathfinding (MAPF) is the problem of finding collision-free paths for multiple agents
moving simultaneously in a shared environment. Each agent has a unique start and goal location,
and the goal is to plan a path for every agent such that they reach their goals without colliding
with one another.

.. list-table:: MAPF algorithms
   :header-rows: 1

   * - Algorithm
     - Class name
     - Optimal
     - Complete
   * - Hierarchical Cooperative A*
     - HCAStar
     - False
     - False
   * - | Windowed Hierarchical
       | Cooperative A*
     - WHCAStar
     - False
     - False
   * - Conflict Based Search
     - CBS
     - True
     - True
   * - Increasing Cost Tree Search
     - ICTS
     - | True
       | (only in an unweighted graph)
     - True
   * - | A* with Operator
       | Decomposition
     - MultiAgentAStar
     - True
     - True

.. note::

  All pathfinding algorithms in w9-pathfinding are environment-agnostic.
  This means they can operate on any :doc:`environment </envs/index>`.

.. toctree::
   :maxdepth: 1

   ReservationTable
   SpaceTimeAStar
   HCAStar
   WHCAStar
   CBS
   ICTS
   MultiAgentAStar
