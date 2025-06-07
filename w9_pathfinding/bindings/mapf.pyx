# distutils: language = c++

from functools import wraps
from libcpp cimport bool
from libcpp.vector cimport vector
from cython.operator cimport dereference

from w9_pathfinding.bindings cimport cdefs
from w9_pathfinding.bindings.envs cimport _AbsGraph


cdef class ReservationTable:
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
        cdef int node_id = self.graph._node_mapper.to_id(node)
        return self._obj.is_reserved(time, node_id)

    def is_edge_reserved(self, int time, n1, n2):
        cdef int n1_id = self.graph._node_mapper.to_id(n1)
        cdef int n2_id = self.graph._node_mapper.to_id(n2)
        return self._obj.is_reserved_edge(time, n1_id, n2_id)

    def add_path(
        self,
        path,
        int start_time=0,
        bool reserve_destination=False,
    ):
        cdef vector[int] node_ids = self.graph._node_mapper.to_ids(path)
        self._obj.add_path(start_time, node_ids, reserve_destination, self.graph.edge_collision)

    def add_vertex_constraint(self, int time, node, bool permanent=False):
        # if permanent - the node is permanently reserved from the moment time, inclusive
        cdef int node_id = self.graph._node_mapper.to_id(node)
        if not permanent:
            self._obj.add_vertex_constraint(time, node_id)
        else:
            self._obj.add_semi_static_constraint(time, node_id)

    def add_edge_constraint(self, int time, n1, n2):
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
        assert len(starts) == len(goals)

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
            Provides time-dependent obstacle reservations.

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
            A time-based structure containing obstacle reservations

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
            A time-based structure containing obstacle reservations

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
            A time-based structure containing obstacle reservations

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
    cdef cdefs.AbsMAPF* _baseobj
    cdef readonly _AbsGraph graph

    def __cinit__(self):
        pass

    def __repr__(self):
        return f"{self.__class__.__name__}(graph={self.graph})"

    cdef cdefs.ReservationTable* _to_crt(self, ReservationTable reservation_table):
        cdef cdefs.ReservationTable* crt
        if reservation_table is None:
            crt = NULL
        else:
            assert(reservation_table.graph == self.graph)
            crt = reservation_table._obj
        return crt

    @_mapf
    def mapf(self, vector[int] starts, vector[int] goals):
        return self._baseobj.mapf(starts, goals)


cdef class HCAStar(_AbsMAPF):
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
        return self._obj.mapf(
            starts,
            goals,
            max_length,
            self._to_crt(reservation_table),
        )


cdef class WHCAStar(_AbsMAPF):
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
        return self._obj.mapf(
            starts,
            goals,
            max_length,
            window_size,
            self._to_crt(reservation_table),
        )


cdef class CBS(_AbsMAPF):
    cdef cdefs.CBS* _obj

    def __cinit__(self, _AbsGraph graph):
        self.graph = graph
        self._obj = new cdefs.CBS(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj

    @property
    def num_generated_nodes(self):
        return self._obj.num_generated_nodes

    @property
    def num_closed_nodes(self):
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
        return self._obj.mapf(
            starts,
            goals,
            max_length,
            max_time,
            disjoint_splitting,
            self._to_crt(reservation_table),
        )


cdef class ICTS(_AbsMAPF):
    cdef cdefs.ICTS* _obj

    def __cinit__(self, _AbsGraph graph):
        self.graph = graph
        self._obj = new cdefs.ICTS(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj

    @property
    def num_generated_nodes(self):
        return self._obj.num_generated_nodes

    @property
    def num_closed_nodes(self):
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
        return self._obj.mapf(
            starts,
            goals,
            max_length,
            max_time,
            ict_pruning,
            self._to_crt(reservation_table),
        )


cdef class MultiAgentAStar(_AbsMAPF):
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
        return self._obj.mapf(
            starts,
            goals,
            max_length,
            max_time,
            operator_decomposition,
            self._to_crt(reservation_table),
        )
 