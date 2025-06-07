# distutils: language = c++

from functools import wraps
from w9_pathfinding.bindings cimport cdefs
from w9_pathfinding.bindings.envs cimport _AbsGraph, Graph


def _pathfinding(func):
    @wraps(func)
    def wrap(finder, start, goal, **kwargs):
        map = finder.graph._node_mapper
        start = map.to_id(start)
        goal = map.to_id(goal)
        path = map.from_ids(func(finder, start, goal, **kwargs))
        return path

    return wrap


cdef class _AbsPathFinder():
    cdef cdefs.AbsPathFinder* _baseobj
    cdef readonly _AbsGraph graph

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
    cdef readonly _AbsGraph graph

    def __cinit__(self, _AbsGraph graph, start_node):
        self.graph = graph
        node_id = graph._node_mapper.to_id(start_node)
        self._obj = new cdefs.ResumableBFS(graph._baseobj, node_id)

    def __repr__(self):
        return f"{self.__class__.__name__}(graph={self.graph}, start_node={self.start_node})"

    def __dealloc__(self):
        del self._obj

    @property
    def start_node(self):
        node_id = self._obj.start_node()
        return self.graph._node_mapper.from_id(node_id)

    @start_node.setter
    def start_node(self, start_node):
        node_id = self.graph._node_mapper.to_id(start_node)
        self._obj.set_start_node(node_id)

    def distance(self, node):
        node_id = self.graph._node_mapper.to_id(node)
        d = self._obj.distance(node_id)
        return d if d >= 0 else float("inf")

    def find_path(self, node):
        map = self.graph._node_mapper
        node_id = map.to_id(node)
        path = self._obj.find_path(node_id)
        return map.from_ids(path)


cdef class ResumableDijkstra:
    cdef cdefs.ResumableDijkstra* _obj
    cdef readonly _AbsGraph graph

    def __cinit__(self, _AbsGraph graph, start_node):
        self.graph = graph
        node_id = graph._node_mapper.to_id(start_node)
        self._obj = new cdefs.ResumableDijkstra(graph._baseobj, node_id)

    def __repr__(self):
        return f"{self.__class__.__name__}(graph={self.graph}, start_node={self.start_node})"

    def __dealloc__(self):
        del self._obj

    @property
    def start_node(self):
        node_id = self._obj.start_node()
        return self.graph._node_mapper.from_id(node_id)

    @start_node.setter
    def start_node(self, start_node):
        node_id = self.graph._node_mapper.to_id(start_node)
        self._obj.set_start_node(node_id)

    def distance(self, node):
        node_id = self.graph._node_mapper.to_id(node)
        d = self._obj.distance(node_id)
        return d if d >= 0 else float("inf")

    def find_path(self, node):
        map = self.graph._node_mapper
        node_id = map.to_id(node)
        path = self._obj.find_path(node_id)
        return map.from_ids(path)
