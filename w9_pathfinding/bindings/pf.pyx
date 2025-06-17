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
    """
    Abstract base class for all pathfinding algorithms
    """

    cdef cdefs.AbsPathFinder* _baseobj
    cdef readonly _AbsGraph graph

    def __cinit__(self):
        pass

    def __repr__(self):
        return f"{self.__class__.__name__}(graph={self.graph})"

    @_pathfinding
    def find_path(self, start, goal):
        """
        Find a path between two nodes.

        Parameters
        ----------
        start : node
            The starting node.

        goal : node
            The goal node.

        Returns
        -------
        list of nodes
            A list of node representing the path from `start` to `goal`.
            If path is found the list will start with `start` and end with `goal`.
            If no path is found, returns an empty list.

        Raises
        ------
        ValueError
            If either `start` or `goal` is not a valid node in the environment.
        """
        return self._baseobj.find_path(start, goal)


cdef class DFS(_AbsPathFinder):
    """
    Depth-First Search (DFS) pathfinding algorithm.

    DFS explores as far as possible along each branch before backtracking.

    It does not guarantee to find the optimal path.

    Parameters
    ----------
    graph : _AbsGraph
        The environment in which to search for paths.
    """

    cdef cdefs.DFS* _obj

    def __cinit__(self, _AbsGraph graph):
        self.graph = graph
        self._obj = new cdefs.DFS(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj


cdef class BFS(_AbsPathFinder):
    """
    Breadth-First Search (BFS) pathfinding algorithm.

    BFS explores all nodes at the current depth before moving to the next level.

    It guarantees to find the optimal path only in unweighted graphs
    (when all edges have equal cost).

    Parameters
    ----------
    graph : _AbsGraph
        The environment in which to search for paths.
    """

    cdef cdefs.BFS* _obj

    def __cinit__(self, _AbsGraph graph):
        self.graph = graph
        self._obj = new cdefs.BFS(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj


cdef class BiBFS(_AbsPathFinder):
    """
    Bidirectional Breadth-First Search (BiBFS) algorithm.

    BiBFS performs two simultaneous BFS searches: one from the start and one
    from the goal. When the two frontiers meet, a valid path is reconstructed.
    This algorithm can significantly reduce search time in large graphs compared to BFS.

    It guarantees to find the optimal path only in unweighted graphs
    (when all edges have equal cost).

    Parameters
    ----------
    graph : _AbsGraph
        The environment in which to search for paths.
    """

    cdef cdefs.BiBFS* _obj

    def __cinit__(self, _AbsGraph graph):
        self.graph = graph
        self._obj = new cdefs.BiBFS(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj


cdef class Dijkstra(_AbsPathFinder):
    """
    Dijkstra's algorithm for optimal pathfinding in weighted graphs.

    Dijkstra's algorithm computes the lowest-cost path between nodes by
    expanding nodes in order of increasing path cost.

    It guarantees to find the optimal path in any graph.

    Parameters
    ----------
    graph : _AbsGraph
        The environment in which to search for paths.
    """

    cdef cdefs.Dijkstra* _obj
    
    def __cinit__(self, _AbsGraph graph):
        self.graph = graph
        self._obj = new cdefs.Dijkstra(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj


cdef class BiDijkstra(_AbsPathFinder):
    """
    Bidirectional Dijkstra's algorithm for optimal pathfinding in weighted graphs.

    This algorithm runs two simultaneous Dijkstra searches — one from the start,
    one from the goal — and stops when the frontiers meet. Often faster than regular Dijkstra.

    It guarantees to find the optimal path in any graph.

    Parameters
    ----------
    graph : _AbsGraph
        The environment in which to search for paths.
    """

    cdef cdefs.BiDijkstra* _obj

    def __cinit__(self, _AbsGraph graph):
        self.graph = graph
        self._obj = new cdefs.BiDijkstra(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj


cdef class AStar(_AbsPathFinder):
    """
    A* search algorithm for optimal pathfinding.

    A* extends Dijkstra by using a heuristic function to estimate
    the remaining cost to the goal, allowing faster search on many graphs.

    It guarantees to find the optimal path in any graph.

    Parameters
    ----------
    graph : _AbsGraph
        The environment in which to search for paths.

    Raises
    ------
    ValueError
        If the graph does not support heuristics.
    """

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
    """
    Bidirectional A* search algorithm for optimal pathfinding.

    This algorithm runs two simultaneous A* searches — one from the start,
    one from the goal — and stops when the frontiers meet. Often faster than regular A*.

    It guarantees to find the optimal path in any graph.

    Parameters
    ----------
    graph : _AbsGraph
        The environment in which to search for paths.

    Raises
    ------
    ValueError
        If the graph does not support heuristics.
    """

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
    """
    Greedy Best-First Search (GBS) algorithm.

    GBS uses only the heuristic function to guide the search, always choosing
    the node that appears closest to the goal.

    It can be fast, but does not guarantee to find the optimal path.

    Parameters
    ----------
    graph : _AbsGraph
        The environment in which to search for paths.

    Raises
    ------
    ValueError
        If the graph does not support heuristics.
    """

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
    """
    Iterative Deepening A* (IDA*) algorithm.

    IDA* combines the space efficiency of DFS with the optimality of A*.
    It performs repeated depth-limited searches, gradually increasing the
    limit based on estimated cost.

    It guarantees to find the optimal path in any graph.

    Parameters
    ----------
    graph : _AbsGraph
        The environment in which to search for paths.

    Raises
    ------
    ValueError
        If the graph does not support heuristics.
    """

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
        """
        Find a path between two nodes.

        Parameters
        ----------
        start : node
            The starting node.

        goal : node
            The goal node.

        max_distance : float, default is 10.
            The maximum allowed path cost (`g + h`) to consider during the search.
            If the optimal path exceeds this cost, the algorithm returns an empty list.

        Returns
        -------
        list of nodes
            A list of node representing the path from `start` to `goal`.
            If path is found the list will start with `start` and end with `goal`.
            If no path is found, returns an empty list.

        Raises
        ------
        ValueError
            If either `start` or `goal` is not a valid node in the environment.
        """
        return self._obj.find_path(start, goal, max_distance)


cdef class ResumableBFS:
    """
    Resumable Breadth-First Search (BFS) for shortest-path queries from a fixed source.

    This class performs a single-source BFS from a given `start_node`,
    and allows querying shortest-path distances and paths to any node.
    Unlike standard BFS, the results are cached and reused efficiently for
    multiple distance/path queries without re-running the full search.

    Useful in scenarios where multiple path or distance lookups are needed
    from the same source node.

    It guarantees to find the optimal path only in unweighted graphs
    (when all edges have equal cost).

    Parameters
    ----------
    graph : _AbsGraph
        The environment in which the BFS traversal is performed.

    start_node : node
        The source node to start BFS from.

    Raises
    ------
    ValueError
        If `start_node` is not a valid node in the environment.
    """

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
        """
        The current start node for the BFS traversal.
        This is the source node from which all distances and paths are computed.

        You can get or set this property. Setting a new start node will
        reset internal state and rerun BFS's algorithm in the next query.
        """
        node_id = self._obj.start_node()
        return self.graph._node_mapper.from_id(node_id)

    @start_node.setter
    def start_node(self, start_node):
        """
        Set a new start node for the BFS traversal.
        """
        node_id = self.graph._node_mapper.to_id(start_node)
        self._obj.set_start_node(node_id)

    def distance(self, node):
        """
        Get the minimum number of steps required to reach `node` from the `start_node`.

        Parameters
        ----------
        node : node
            The target node.

        Returns
        -------
        float
            The number of steps from `start_node` to `node`.
            Returns `float('inf')` if the node is unreachable.

        Raises
        ------
        ValueError
            If `node` is not a valid node in the environment.
        """
        node_id = self.graph._node_mapper.to_id(node)
        d = self._obj.distance(node_id)
        return d if d >= 0 else float("inf")

    def find_path(self, node):
        """
        Find the shortest path from the start node to the given node,
        minimizing the number of steps.

        Parameters
        ----------
        node : node
            The target node.

        Returns
        -------
        list
            A list of nodes representing the shortest (step-wise) path from
            `start_node` to `node`.
            If path is found the list will start with `start_node` and end with `node`.
            Returns an empty list if the target node is unreachable.

        Raises
        ------
        ValueError
            If `node` is not a valid node in the environment.
        """
        map = self.graph._node_mapper
        node_id = map.to_id(node)
        path = self._obj.find_path(node_id)
        return map.from_ids(path)


cdef class ResumableDijkstra:
    """
    Resumable Dijkstra's algorithm for shortest-path queries from a fixed source.

    This class performs a single-source Dijkstra traversal from a given `start_node`,
    and allows querying shortest-path distances and paths to any node.
    Unlike standard Dijkstra's algorithm, the results are cached and reused efficiently for
    multiple distance/path queries without re-running the full search.

    It guarantees to find the optimal path in any graph.

    Parameters
    ----------
    graph : _AbsGraph
        The environment in which the BFS traversal is performed.

    start_node : node
        The source node to start Dijkstra's algorithm from.

    Raises
    ------
    ValueError
        If `start_node` is not a valid node in the environment.
    """

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
        """
        The current start node for the Dijkstra's traversal.
        This is the source node from which all distances and paths are computed.

        You can get or set this property. Setting a new start node will
        reset internal state and rerun Dijkstra's algorithm in the next query..
        """
        node_id = self._obj.start_node()
        return self.graph._node_mapper.from_id(node_id)

    @start_node.setter
    def start_node(self, start_node):
        """
        Set a new start node for the Dijkstra's traversal.
        """
        node_id = self.graph._node_mapper.to_id(start_node)
        self._obj.set_start_node(node_id)

    def distance(self, node):
        """
        Get the shortest-path cost from the `start node` to the given node.

        Parameters
        ----------
        node : node
            The target node.

        Returns
        -------
        float
            The minimum cost from `start_node` to `node`.
            Returns `float('inf')` if the node is unreachable.

        Raises
        ------
        ValueError
            If `node` is not a valid node in the environment.
        """
        node_id = self.graph._node_mapper.to_id(node)
        d = self._obj.distance(node_id)
        return d if d >= 0 else float("inf")

    def find_path(self, node):
        """
        Find the optimal path from the start node to the given node.

        Parameters
        ----------
        node : node
            The target node.

        Returns
        -------
        list
            A list of nodes representing the optimal path from `start_node` to `node`.
            If path is found the list will start with `start_node` and end with `node`.
            Returns an empty list if the target node is unreachable.

        Raises
        ------
        ValueError
            If `node` is not a valid node in the environment.
        """
        map = self.graph._node_mapper
        node_id = map.to_id(node)
        path = self._obj.find_path(node_id)
        return map.from_ids(path)
