# distutils: language = c++

from w9_pathfinding cimport cdefs
from w9_pathfinding.envs cimport _AbsGrid, _AbsGraph, Graph, ReservationTable


def to_node_id(graph, node):
    if isinstance(graph, Graph):
        graph.assert_in(node)
        return node
    elif isinstance(graph, _AbsGrid):
        return graph.get_node_id(node)
    else:
        raise NotImplementedError


def to_python_node(graph, node_id):
    if isinstance(graph, Graph):
        return node_id
    elif isinstance(graph, _AbsGrid):
        return graph.get_coordinates(node_id)
    else:
        raise NotImplementedError


def _pathfinding(func):

    def wrap(finder, start, goal, **kwargs):
        g = finder.graph

        if isinstance(g, Graph):
            g.assert_in(start)
            g.assert_in(goal)
            path = func(finder, start, goal, **kwargs)

        elif isinstance(g, _AbsGrid):
            start = g.get_node_id(start)
            goal = g.get_node_id(goal)
            path = func(finder, start, goal, **kwargs)
            path = [g.get_coordinates(node) for node in path]

        else:
            raise NotImplementedError

        return path

    return wrap


cdef class _AbsPathFinder():
    cdef cdefs.AbsPathFinder* _baseobj
    cdef public _AbsGraph graph

    def __cinit__(self):
        pass

    def __repr__(self):
        return f"{self.__class__.__name__}(graph={self.graph})"

    @_pathfinding
    def find_path(self, start, goal):
        return self._baseobj.find_path(start, goal)


cdef class DFS(_AbsPathFinder):
    cdef cdefs.DFS* _obj

    def __cinit__(self, _AbsGraph graph):
        self.graph = graph
        self._obj = new cdefs.DFS(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj


cdef class BFS(_AbsPathFinder):
    cdef cdefs.BFS* _obj

    def __cinit__(self, _AbsGraph graph):
        self.graph = graph
        self._obj = new cdefs.BFS(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj


cdef class BiBFS(_AbsPathFinder):
    cdef cdefs.BiBFS* _obj

    def __cinit__(self, _AbsGraph graph):
        self.graph = graph
        self._obj = new cdefs.BiBFS(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj


cdef class Dijkstra(_AbsPathFinder):
    cdef cdefs.Dijkstra* _obj
    
    def __cinit__(self, _AbsGraph graph):
        self.graph = graph
        self._obj = new cdefs.Dijkstra(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj


cdef class BiDijkstra(_AbsPathFinder):
    cdef cdefs.BiDijkstra* _obj

    def __cinit__(self, _AbsGraph graph):
        self.graph = graph
        self._obj = new cdefs.BiDijkstra(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj


cdef class AStar(_AbsPathFinder):
    cdef cdefs.AStar* _obj

    def __cinit__(self, _AbsGraph graph):
        if isinstance(graph, Graph) and not graph.has_coordinates():
            raise ValueError(
                "A* cannot work with a graph without coordinates. "
                "You can add coordinates using graph.set_coordinates(), "
                "or choose some non-heuristic algorithm."
            )
        self.graph = graph
        self._obj = new cdefs.AStar(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj


cdef class BiAStar(_AbsPathFinder):
    cdef cdefs.BiAStar* _obj

    def __cinit__(self, _AbsGraph graph):
        if isinstance(graph, Graph) and not graph.has_coordinates():
            raise ValueError(
                "A* cannot work with a graph without coordinates. "
                "You can add coordinates using graph.set_coordinates(), "
                "or choose some non-heuristic algorithm."
            )
        self.graph = graph
        self._obj = new cdefs.BiAStar(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj


cdef class GBS(_AbsPathFinder):
    # Greedy Best-first Search

    cdef cdefs.GBS* _obj

    def __cinit__(self, _AbsGraph graph):
        if isinstance(graph, Graph) and not graph.has_coordinates():
            raise ValueError(
                "GBS cannot work with a graph without coordinates. "
                "You can add coordinates using graph.set_coordinates(), "
                "or choose some non-heuristic algorithm."
            )
        self.graph = graph
        self._obj = new cdefs.GBS(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj


cdef class IDAStar(_AbsPathFinder):
    # Iterative deepening A*

    cdef cdefs.IDAStar* _obj

    def __cinit__(self, _AbsGraph graph):
        if isinstance(graph, Graph) and not graph.has_coordinates():
            raise ValueError(
                "IDA* cannot work with a graph without coordinates. "
                "You can add coordinates using graph.set_coordinates(), "
                "or choose some non-heuristic algorithm."
            )
        self.graph = graph
        self._obj = new cdefs.IDAStar(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj

    @_pathfinding
    def find_path(self, int start, int goal, double max_distance=10):
        return self._obj.find_path(start, goal, max_distance)


cdef class ResumableBFS:
    cdef cdefs.ResumableBFS* _obj
    cdef public _AbsGraph graph

    def __cinit__(self, _AbsGraph graph, start_node):
        self.graph = graph
        self._obj = new cdefs.ResumableBFS(graph._baseobj, to_node_id(graph, start_node))

    def __repr__(self):
        return f"{self.__class__.__name__}(graph={self.graph}, start_node={self.start_node})"

    def __dealloc__(self):
        del self._obj

    @property
    def start_node(self):
        return to_python_node(self.graph, self._obj.start_node())

    @start_node.setter
    def start_node(self, start_node):
        self._obj.set_start_node(to_node_id(self.graph, start_node))

    def distance(self, node):
        d = self._obj.distance(to_node_id(self.graph, node))
        return d if d >= 0 else float("inf")

    def find_path(self, node):
        g = self.graph
        path = self._obj.find_path(to_node_id(g, node))
        return [to_python_node(g, node_id) for node_id in path]


cdef class ResumableDijkstra:
    cdef cdefs.ResumableDijkstra* _obj
    cdef public _AbsGraph graph

    def __cinit__(self, _AbsGraph graph, start_node):
        self.graph = graph
        self._obj = new cdefs.ResumableDijkstra(graph._baseobj, to_node_id(graph, start_node))

    def __repr__(self):
        return f"{self.__class__.__name__}(graph={self.graph}, start_node={self.start_node})"

    def __dealloc__(self):
        del self._obj

    @property
    def start_node(self):
        return to_python_node(self.graph, self._obj.start_node())

    @start_node.setter
    def start_node(self, start_node):
        self._obj.set_start_node(to_node_id(self.graph, start_node))

    def distance(self, node):
        d = self._obj.distance(to_node_id(self.graph, node))
        return d if d >= 0 else float("inf")

    def find_path(self, node):
        g = self.graph
        path = self._obj.find_path(to_node_id(g, node))
        return [to_python_node(g, node_id) for node_id in path]


cdef class SpaceTimeAStar(_AbsPathFinder):
    cdef cdefs.SpaceTimeAStar* _obj

    def __cinit__(self, _AbsGraph graph):
        self.graph = graph
        self._obj = new cdefs.SpaceTimeAStar(graph._baseobj)
        self._baseobj = self._obj

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

    @_pathfinding
    def find_path(
        self,
        int start,
        int goal,
        int search_depth=100,
        ReservationTable reservation_table=None,
    ):
        return self._obj.find_path_with_depth_limit(
            start,
            goal,
            search_depth,
            self._to_crt(reservation_table),
        )

    @_pathfinding
    def find_path_with_depth_limit(
        self,
        int start,
        int goal,
        int search_depth=100,
        ReservationTable reservation_table=None,
    ):
        return self._obj.find_path_with_depth_limit(
            start,
            goal,
            search_depth,
            self._to_crt(reservation_table),
        )

    @_pathfinding
    def find_path_with_exact_length(
        self,
        int start,
        int goal,
        int length,
        ReservationTable reservation_table=None,
    ):
        return self._obj.find_path_with_exact_length(
            start,
            goal,
            length,
            self._to_crt(reservation_table),
        )

    @_pathfinding
    def find_path_with_length_limit(
        self,
        int start,
        int goal,
        int max_length,
        ReservationTable reservation_table=None,
    ):
        return self._obj.find_path_with_length_limit(
            start,
            goal,
            max_length,
            self._to_crt(reservation_table),
        )
