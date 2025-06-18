Multi-Agent Pathfinding
=========================

Multi-Agent Pathfinding (MAPF) is the problem of finding collision-free paths for multiple agents
moving simultaneously in a shared environment. Each agent has a unique start and goal location,
and the goal is to plan a path for every agent such that they reach their goals without colliding
with one another.

.. list-table:: MAPF algorithms
   :header-rows: 1

   * - Algorithm
     - Class
     - Optimal
     - Complete
   * - Hierarchical Cooperative A*
     - :ref:`HCAStar`
     - False
     - False
   * - | Windowed Hierarchical
       | Cooperative A*
     - :ref:`WHCAStar`
     - False
     - False
   * - Conflict Based Search
     - :ref:`CBS`
     - True
     - True
   * - Increasing Cost Tree Search
     - :ref:`ICTS`
     - | True
       | (only in an unweighted graph)
     - True
   * - | A* with Operator
       | Decomposition
     - :ref:`MultiAgentAStar`
     - True
     - True

.. toctree::
   :maxdepth: 1

   ReservationTable
   SpaceTimeAStar
   HCAStar
   WHCAStar
   CBS
   ICTS
   MultiAgentAStar
