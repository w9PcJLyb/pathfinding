# distutils: language = c++

from functools import wraps
from libcpp cimport bool
from libcpp.vector cimport vector
from cython.operator cimport dereference

from w9_pathfinding.bindings cimport cdefs
from w9_pathfinding.bindings.envs cimport _AbsGraph


cdef class ReservationTable:
    """
    Data structure to manage dynamic obstacles.

    Can be used to simulate obstacles that move along a known path or
    agents with precomputed paths.

    Parameters
    ----------
    graph : _AbsGraph
        The environment (graph or grid).
    """

    cdef cdefs.ReservationTable* _obj
    cdef readonly _AbsGraph graph

    def __cinit__(self, _AbsGraph graph):
        cdef int graph_size = graph.size

        self.graph = graph
        self._obj = new cdefs.ReservationTable(graph_size)

    def __dealloc__(self):
        del self._obj

    def __repr__(self):
        return f"ReservationTable(graph={self.graph})"

    def is_reserved(self, int time, node):
        """
        Check if a node is reserved at a given time.

        Parameters
        ----------
        time : int
            The time step to query.
        node : node
            The node to check

        Returns
        -------
        bool
            True if the node is reserved at the given time.

        Raises
        ------
        ValueError
            If `node` is not a valid node in the environment.
        """
        cdef int node_id = self.graph._node_mapper.to_id(node)
        return self._obj.is_reserved(time, node_id)

    def is_edge_reserved(self, int time, n1, n2):
        """
        Check if moving from node `n1` to `n2` is reserved at a given time.

        Parameters
        ----------
        time : int
            The time step of the movement.
        n1 : node
            The start node of the edge.
        n2 : node
            The end node of the edge.

        Returns
        -------
        bool
            True if the edge is reserved at the given time.

        Raises
        ------
        ValueError
            If either `n1` or `n2` is not a valid node in the environment.
        """
        cdef int n1_id = self.graph._node_mapper.to_id(n1)
        cdef int n2_id = self.graph._node_mapper.to_id(n2)
        return self._obj.is_reserved_edge(time, n1_id, n2_id)

    def add_path(
        self,
        path,
        int start_time=0,
        bool reserve_destination=False,
    ):
        """
        Reserve all nodes along a given path starting at a specified time.
        If edge_collision is enabled in the environment, this method will
        also reserve edges along the path.

        Parameters
        ----------
        path : List[node]
            The list of nodes representing the path.
        start_time : int, default 0
            The time step at which the path starts.
        reserve_destination : bool, default False
            If True, the destination node remains reserved after arrival.

        Raises
        ------
        ValueError
            If any node in the path is not a valid node in the environment.
        """
        cdef vector[int] node_ids = self.graph._node_mapper.to_ids(path)
        self._obj.add_path(start_time, node_ids, reserve_destination, self.graph.edge_collision)

    def add_vertex_constraint(self, int time, node, bool permanent=False):
        """
        Add a vertex constraint to block access to a node at a specific time.

        Parameters
        ----------
        time : int
            The time step at which the node should be blocked.
        node : node
            The node to constrain.
        permanent : bool, default False
            If True, the node is permanently blocked from `time` onward.

        Raises
        ------
        ValueError
            If `node` is not a valid node in the environment.
        """
        cdef int node_id = self.graph._node_mapper.to_id(node)
        if not permanent:
            self._obj.add_vertex_constraint(time, node_id)
        else:
            self._obj.add_semi_static_constraint(time, node_id)

    def add_edge_constraint(self, int time, n1, n2):
        """
        Add a constraint that prevents movement along a specific edge at a given time.

        Parameters
        ----------
        time : int
            The time step of the movement.
        n1 : node
            The starting node of the edge.
        n2 : node
            The ending node of the edge.

        Raises
        ------
        ValueError
            If either `n1` or `n2` is not a valid node in the environment.
        """
        cdef int n1_id, n2_id
        n1_id = self.graph._node_mapper.to_id(n1)
        n2_id = self.graph._node_mapper.to_id(n2)
        self._obj.add_edge_constraint(time, n1_id, n2_id)

    def __copy__(self):
        rt = ReservationTable(self.graph)
        rt._obj = new cdefs.ReservationTable(dereference(self._obj))
        return rt


def _pathfinding(func):
    @wraps(func)
    def wrap(finder, start, goal, **kwargs):
        map = finder.graph._node_mapper
        start = map.to_id(start)
        goal = map.to_id(goal)
        path = map.from_ids(func(finder, start, goal, **kwargs))
        return path

    return wrap


def _mapf(func):
    @wraps(func)
    def wrap(finder, starts, goals, **kwargs):
        if len(starts) != len(goals):
            raise ValueError("The lengths of `starts` and `goals` must be the same.")

        map = finder.graph._node_mapper
        starts = map.to_ids(starts)
        goals = map.to_ids(goals)
        paths = [map.from_ids(p) for p in func(finder, starts, goals, **kwargs)]
        return paths

    return wrap


cdef class SpaceTimeAStar:
    """
    A space-time A* planner for dynamic pathfinding.

    The planner computes collision-free paths for a single agent in environments
    with known dynamic obstacles.

    Parameters
    ----------
    graph : _AbsGraph
        The spatial structure of the environment
    """

    cdef cdefs.SpaceTimeAStar* _obj
    cdef readonly _AbsGraph graph

    def __cinit__(self, _AbsGraph graph):
        self.graph = graph
        self._obj = new cdefs.SpaceTimeAStar(graph._baseobj)

    def __dealloc__(self):
        del self._obj

    cdef cdefs.ReservationTable* _to_crt(self, ReservationTable reservation_table):
        cdef cdefs.ReservationTable* crt
        if reservation_table is None:
            crt = NULL
        else:
            assert(reservation_table.graph == self.graph)
            crt = reservation_table._obj
        return crt

    def find_path(
        self,
        start,
        goal,
        int max_length=100,
        ReservationTable reservation_table=None,
    ):
        """
        Finds an optimal path from start to goal, constrained by a maximum path length.

        Guarantees persistent goal safety: once the agent reaches the goal, it can
        remain there indefinitely without future collisions.

        Note: There may exist a lower-cost path with a length greater than `max_length`.
        This function only considers paths of length ≤ `max_length`.

        Parameters
        ----------
        start : node
            The starting node in the graph
        goal : node
            The goal node in the graph
        max_length : int, default=100
            Maximum allowed path length (number of time steps).
        reservation_table : ReservationTable, optional
            Dynamic obstacles.

        Returns
        -------
        path : list[node]
            A list of nodes representing the path.
            Returns an empty list if no valid path is found.
        """

        return self.find_path_with_length_limit(
            start,
            goal,
            max_length=max_length,
            stay_at_goal=True,
            reservation_table=reservation_table,
        )

    @_pathfinding
    def find_path_with_length_limit(
        self,
        start,
        goal,
        int max_length,
        bool stay_at_goal=True,
        ReservationTable reservation_table=None,
    ):
        """
        Finds an optimal path from start to goal, constrained by a maximum path length.

        Note: There may exist a lower-cost path with a length greater than `max_length`
        This function only considers paths of length ≤ `max_length`.

        Parameters
        ----------
        start : node
            The starting node in the graph
        goal : node
            The goal node in the graph
        max_length : int
            Maximum number of time steps (path length) allowed
        stay_at_goal : bool, default=True
            If True, the agent must remain safely at the goal after arrival.
            If False, the agent disappears upon reaching the goal.
        reservation_table : ReservationTable, optional
            Dynamic obstacles.

        Returns
        -------
        path : list[node]
            A list of nodes representing the path.
            Returns an empty list if no valid path is found.
        """

        cdef int min_terminal_time = 0
        if stay_at_goal and reservation_table:
            min_terminal_time = reservation_table._obj.last_time_reserved(goal)

        return self._obj.find_path_with_length_limit(
            start,
            goal,
            max_length,
            self._to_crt(reservation_table),
            NULL,
            min_terminal_time
        )

    @_pathfinding
    def find_path_with_exact_length(
        self,
        start,
        goal,
        int length,
        ReservationTable reservation_table=None,
    ):
        """
        Finds an optimal collision-free path from start to goal with an exact number of steps.

        It is useful for tightly synchronized planning where arrival time is fixed.

        The agent is assumed to disappear immediately upon reaching the goal; no further
        collision checks are performed at the goal after arrival.

        Parameters
        ----------
        start : node
            The starting node in the graph
        goal : node
            The goal node in the graph
        length : int
            Required number of time steps for the path
        reservation_table : ReservationTable, optional
            Dynamic obstacles.

        Returns
        -------
        path : list[node]
            A list of nodes representing the path.
            Returns an empty list if no valid path is found.
        """

        return self._obj.find_path_with_exact_length(
            start,
            goal,
            length,
            self._to_crt(reservation_table),
        )

    @_pathfinding
    def find_path_with_depth_limit(
        self,
        start,
        goal,
        int search_depth=100,
        bool stay_at_goal=True,
        ReservationTable reservation_table=None,
    ):
        """
        Performs A* search with a limited planning depth, returning partial paths when needed.

        This function explores the space-time graph up to `search_depth` time steps. If a full
        path to the goal is found within that depth, it is returned. Otherwise,
        a partial path is returned.

        This method is particularly useful in scenarios where the environment may change in
        the near future — for example, in WHCA*-like algorithms or rolling-horizon planning —
        where computing the full path is unnecessary or even wasteful.

        Parameters
        ----------
        start : node
            The starting node in the graph
        goal : node
            The goal node in the graph
        search_depth : int, default=100
            Maximum number of time steps the planner is allowed to search
        stay_at_goal : bool, default=True
            If True, ensures safety at the goal for all future steps.
        reservation_table : ReservationTable, optional
            Dynamic obstacles.

        Returns
        -------
        path : list[node]
            A list of nodes representing the path.
            Returns an empty list if no valid path is found.
        """

        cdef int min_terminal_time = 0
        if stay_at_goal and reservation_table:
            min_terminal_time = reservation_table._obj.last_time_reserved(goal)

        return self._obj.find_path_with_depth_limit(
            start,
            goal,
            search_depth,
            self._to_crt(reservation_table),
            NULL,
            min_terminal_time
        )


cdef class _AbsMAPF():
    """
    Abstract base class for multi-agent pathfinding algorithms
    """

    cdef cdefs.AbsMAPF* _baseobj
    cdef readonly _AbsGraph graph

    def __cinit__(self):
        pass

    def __repr__(self):
        return f"{self.__class__.__name__}(graph={self.graph})"

    cdef cdefs.ReservationTable* _to_crt(self, ReservationTable reservation_table):
        """
        Convert a Python ReservationTable into a C++ object pointer,
        or `NULL` if no reservation table is provided.
        """
        cdef cdefs.ReservationTable* crt
        if reservation_table is None:
            crt = NULL
        else:
            assert(reservation_table.graph == self.graph)
            crt = reservation_table._obj
        return crt

    @_mapf
    def mapf(self, vector[int] starts, vector[int] goals):
        """
        Solve the Multi-Agent Pathfinding (MAPF) problem .

        This method finds non-colliding paths for all agents from their respective
        start nodes to goal nodes. The result is a list of individual paths — one
        per agent.

        Parameters
        ----------
        starts : list[node]
            A list of start nodes, one per agent.

        goals : list[node]
            A list of goal nodes, one per agent. Must be the same length as `starts`.

        Returns
        -------
        list[list[node]]
            A list of paths, one per agent.
            If no collision-free paths are found, returns an empty list.

        Raises
        ------
        ValueError
            If the number of `starts` and `goals` is different, or if any node is invalid.
        """
        return self._baseobj.mapf(starts, goals)


cdef class HCAStar(_AbsMAPF):
    """
    Hierarchical Cooperative A* (HCA*) algorithm for multi-agent pathfinding.

    HCA* plans paths for each agent individually using
    :class:`~w9_pathfinding.mapf.SpaceTimeAStar`, while avoiding collisions
    with previously planned agents by reserving their space-time paths.

    This algorithm is neither optimal nor complete, but it is fast and often
    works well in practice.

    Parameters
    ----------
    graph : _AbsGraph
        The environment in which to search for paths.

    References
    ----------
    Silver, D. 2005. Cooperative pathfinding. In AIIDE, 117-122.
    """

    cdef cdefs.HCAStar* _obj

    def __cinit__(self, _AbsGraph graph):
        self.graph = graph
        self._obj = new cdefs.HCAStar(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj

    @_mapf
    def mapf(
        self,
        vector[int] starts,
        vector[int] goals,
        int max_length=100,
        ReservationTable reservation_table=None,
    ):
        """
        Finds non-colliding paths for all agents from their respective
        start nodes to goal nodes. The result is a list of individual paths — one
        per agent.

        Parameters
        ----------
        starts : list[node]
            A list of start nodes, one per agent.

        goals : list[node]
            A list of goal nodes, one per agent. Must be the same length as `starts`.

        max_length : int, default=100
            The maximum allowed length of any individual agent's path.

        reservation_table : ReservationTable, optional
            Dynamic obstacles.

        Returns
        -------
        list[list[node]]
            A list of paths, one per agent.
            If no collision-free paths are found, returns an empty list.

        Raises
        ------
        ValueError
            If the number of `starts` and `goals` is different, or if any node is invalid.
        """
        return self._obj.mapf(
            starts,
            goals,
            max_length,
            self._to_crt(reservation_table),
        )


cdef class WHCAStar(_AbsMAPF):
    """
    Windowed Hierarchical Cooperative A* (WHCA*) for multi-agent pathfinding.

    WHCA* improves upon HCA* by introducing a fixed time window for planning.
    Within this window, it plans paths for each agent individually using
    :class:`~w9_pathfinding.mapf.SpaceTimeAStar`, while avoiding collisions
    with other agents.

    This algorithm is neither optimal nor complete, but it is fast and often
    works well in practice

    Parameters
    ----------
    graph : _AbsGraph
        The environment in which to search for paths.

    References
    ----------
    Silver, D. 2005. Cooperative pathfinding. In AIIDE, 117-122.
    """

    cdef cdefs.WHCAStar* _obj

    def __cinit__(self, _AbsGraph graph):
        self.graph = graph
        self._obj = new cdefs.WHCAStar(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj

    @_mapf
    def mapf(
        self,
        vector[int] starts,
        vector[int] goals,
        int max_length=100,
        int window_size=16,
        ReservationTable reservation_table=None,
    ):
        """
        Finds non-colliding paths for all agents from their respective
        start nodes to goal nodes. The result is a list of individual paths — one
        per agent.

        Parameters
        ----------
        starts : list[node]
            A list of start nodes, one per agent.

        goals : list[node]
            A list of goal nodes, one per agent. Must be the same length as `starts`.

        max_length : int, default=100
            The maximum allowed length of any individual agent's path.

        window_size : int, default=16
            Time window size for path planning.

        reservation_table : ReservationTable, optional
            Dynamic obstacles.

        Returns
        -------
        list[list[node]]
            A list of paths, one per agent.
            If no collision-free paths are found, returns an empty list.

        Raises
        ------
        ValueError
            If the number of `starts` and `goals` is different, or if any node is invalid.
        """
        return self._obj.mapf(
            starts,
            goals,
            max_length,
            window_size,
            self._to_crt(reservation_table),
        )


cdef class CBS(_AbsMAPF):
    """
    Conflict-Based Search (CBS) algorithm for multi-agent pathfinding.

    CBS is a two-level search algorithm that finds optimal, collision-free paths
    for multiple agents by incrementally resolving conflicts between them.

    1. High-level search — Constructs a Constraint Tree (CT), where each node represents
       a set of constraints and corresponding agent paths. The root node has no constraints.
       The algorithm then performs a Best-First Search over this tree, prioritizing
       nodes by their Sum-of-Costs (SoC) — the total cost of all agents' current paths.

    2. Low-level search - For each high-level CT node, the algorithm finds a path
       for each individual agent using :class:`~w9_pathfinding.mapf.SpaceTimeAStar`,
       respecting all constraints associated with that CT node. If a conflict is
       detected between agents' paths, the node is expanded by branching on the
       conflicting agents and adding new constraints to avoid the collision.

    This algorithm is **optimal and complete** — it is guaranteed to find a solution
    if one exists, and the solution will be optimal with respect to the Sum-of-Costs
    objective (i.e., the total cost across all agents' paths).

    While CBS is relatively fast in practice, especially in sparse environments,
    performance may degrade on large graphs or with many agents.

    Parameters
    ----------
    graph : _AbsGraph
        The environment in which to search for paths.

    References
    ----------
    - Sharon et al. 2012 Conflict-Based Search For Optimal Multi-Agent Path Finding
    - Li et al. 2019 Disjoint Splitting for Multi-Agent Path Finding with Conflict-Based Search
    """

    cdef cdefs.CBS* _obj

    def __cinit__(self, _AbsGraph graph):
        self.graph = graph
        self._obj = new cdefs.CBS(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj

    @property
    def num_generated_nodes(self) -> int:
        """
        Total number of Constraint Tree (CT) nodes generated during the last search.
        """
        return self._obj.num_generated_nodes

    @property
    def num_closed_nodes(self) -> int:
        """
        Number of Constraint Tree (CT) nodes expanded (processed) during the last search.
        """
        return self._obj.num_closed_nodes

    @_mapf
    def mapf(
        self,
        vector[int] starts,
        vector[int] goals,
        int max_length=100,
        double max_time=1,
        bool disjoint_splitting=True,
        ReservationTable reservation_table=None,
    ):
        """
        Finds non-colliding paths for all agents from their respective
        start nodes to goal nodes. The result is a list of individual paths — one
        per agent.

        Parameters
        ----------
        starts : list[node]
            A list of start nodes, one per agent.

        goals : list[node]
            A list of goal nodes, one per agent. Must be the same length as `starts`.

        max_length : int, default=100
            The maximum allowed length of any individual agent's path.

        max_time : float, default=1.0
            The maximum amount of time (in seconds) allowed for CBS to find a solution.
            If the time limit is exceeded, a `RuntimeError` is raised.

        disjoint_splitting : bool, default=True
            If `True`, enables disjoint splitting to improve CBS efficiency and performance.
            If `False`, standard collision splitting is used.

        reservation_table : ReservationTable, optional
            Dynamic obstacles.

        Returns
        -------
        list[list[node]]
            A list of paths, one per agent.
            If no collision-free paths are found, returns an empty list.

        Raises
        ------
        ValueError
            If the number of `starts` and `goals` is different, or if any node is invalid.
        RuntimeError
            If no solution is found within the time constraint.
        """
        return self._obj.mapf(
            starts,
            goals,
            max_length,
            max_time,
            disjoint_splitting,
            self._to_crt(reservation_table),
        )


cdef class ICTS(_AbsMAPF):
    """
    Increasing Cost Tree Search (ICTS) algorithm for multi-agent pathfinding.

    ICTS is a two-level search algorithm that explores combinations of path costs
    to find collision-free solutions.

    1. High-level search - Performs a top-down search over combinations of
       individual path costs, forming an Increasing Cost Tree (ICT). The root of the tree
       represents the minimal possible path costs for each agent.

    2. Low-level search - For each high-level node, attempts to build a valid
       joint plan using Multi-value Decision Diagrams (MDDs) to ensure that the agents'
       paths do not conflict.

    This algorithm is **complete** — it is guaranteed to find a solution if one exists.
    It is also **optimal** with respect to the Sum-of-Costs objective, but only
    in unweighted graphs (i.e. where all edges have equal cost). In weighted graphs,
    it may still find valid solutions, but they are not guaranteed to be optimal.

    Parameters
    ----------
    graph : _AbsGraph
        The environment in which to search for paths.

    References
    ----------
    Sharon, G., Stern, R., Goldenberg, M., Felner, A.: The increasing cost tree search
    for optimal multi-agent pathfinding. Artificial Intelligence 195, 470-495 (2013)
    """
    cdef cdefs.ICTS* _obj

    def __cinit__(self, _AbsGraph graph):
        self.graph = graph
        self._obj = new cdefs.ICTS(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj

    @property
    def num_generated_nodes(self) -> int:
        """
        Total number of Increasing Cost Tree (ICT) nodes generated during the last search.
        """
        return self._obj.num_generated_nodes

    @property
    def num_closed_nodes(self):
        """
        Number of Constraint Tree (CT) nodes expanded (processed) during the last search.
        """
        return self._obj.num_closed_nodes

    @_mapf
    def mapf(
        self,
        vector[int] starts,
        vector[int] goals,
        int max_length=100,
        double max_time=1,
        bool ict_pruning=True,
        ReservationTable reservation_table=None,
    ):
        """
        Finds non-colliding paths for all agents from their respective
        start nodes to goal nodes. The result is a list of individual paths — one
        per agent.

        Parameters
        ----------
        starts : list[node]
            A list of start nodes, one per agent.

        goals : list[node]
            A list of goal nodes, one per agent. Must be the same length as `starts`.

        max_length : int, default=100
            The maximum allowed length of any individual agent's path.

        max_time : float, default=1.0
            The maximum amount of time (in seconds) allowed for CBS to find a solution.
            If the time limit is exceeded, a `RuntimeError` is raised.

        ict_pruning : bool, default=True
            If `True`, enables enhanced pairwise pruning.

        reservation_table : ReservationTable, optional
            Dynamic obstacles.

        Returns
        -------
        list[list[node]]
            A list of paths, one per agent.
            If no collision-free paths are found, returns an empty list.

        Raises
        ------
        ValueError
            If the number of `starts` and `goals` is different, or if any node is invalid.
        RuntimeError
            If no solution is found within the time constraint.
        """
        return self._obj.mapf(
            starts,
            goals,
            max_length,
            max_time,
            ict_pruning,
            self._to_crt(reservation_table),
        )


cdef class MultiAgentAStar(_AbsMAPF):
    """
    Multi-Agent A* (MAA*) algorithm for multi-agent pathfinding.

    This algorithm performs a joint search in the combined state space of all agents,
    treating the multi-agent configuration as a single search node.

    It is **optimal and complete** — it is guaranteed to find a solution
    if one exists, and the solution will be optimal with respect to the Sum-of-Costs
    objective (i.e., the total cost across all agents' paths).

    This algorithm typically scales poorly due to the size of the joint state space.
    Can be suitable only for a small number of agents or simple environments.

    Parameters
    ----------
    graph : _AbsGraph
        The environment in which to search for paths.

    References
    ----------
    Standley, T.S.: Finding optimal solutions to cooperative pathfinding problems.
    In: AAAI Conference on Artificial Intelligence. pp. 173-178 (2010)
    """
    cdef cdefs.MultiAgentAStar* _obj

    def __cinit__(self, _AbsGraph graph):
        self.graph = graph
        self._obj = new cdefs.MultiAgentAStar(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj

    @_mapf
    def mapf(
        self,
        vector[int] starts,
        vector[int] goals,
        int max_length=100,
        double max_time=1,
        bool operator_decomposition=True,
        ReservationTable reservation_table=None,
    ):
        """
        Finds non-colliding paths for all agents from their respective
        start nodes to goal nodes. The result is a list of individual paths — one
        per agent.

        Parameters
        ----------
        starts : list[node]
            A list of start nodes, one per agent.

        goals : list[node]
            A list of goal nodes, one per agent. Must be the same length as `starts`.

        max_length : int, default=100
            The maximum allowed length of any individual agent's path.

        max_time : float, default=1.0
            The maximum amount of time (in seconds) allowed for CBS to find a solution.
            If the time limit is exceeded, a `RuntimeError` is raised.

        operator_decomposition : bool, default=True
            If `True`, enables Operator Decomposition to reduce branching in joint space.

        reservation_table : ReservationTable, optional
            Dynamic obstacles.

        Returns
        -------
        list[list[node]]
            A list of paths, one per agent.
            If no collision-free paths are found, returns an empty list.

        Raises
        ------
        ValueError
            If the number of `starts` and `goals` is different, or if any node is invalid.
        RuntimeError
            If no solution is found within the time constraint.
        """
        return self._obj.mapf(
            starts,
            goals,
            max_length,
            max_time,
            operator_decomposition,
            self._to_crt(reservation_table),
        )
 