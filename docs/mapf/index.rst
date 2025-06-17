Multi-Agent Pathfinding
=========================

Multi-Agent Pathfinding (MAPF) is the problem of finding collision-free paths for multiple agents
moving simultaneously in a shared environment. Each agent has a unique start and goal location,
and the goal is to plan a path for every agent such that they reach their goals without colliding
with one another.

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
