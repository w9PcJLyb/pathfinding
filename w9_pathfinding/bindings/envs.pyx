# distutils: language = c++

from libcpp cimport bool
from libcpp.vector cimport vector
from w9_pathfinding.bindings cimport cdefs
from w9_pathfinding.hex_layout import HexLayout
from w9_pathfinding.diagonal_movement import DiagonalMovement


cdef class _NodeMapper:
    """
    This class abstracts away the conversion between user-facing node representations
    (like coordinates) and internal integer node IDs used by C++ pathfinding algorithms.
    """

    def __cinit__(self):
        pass

    def contains(self, node) -> bool:
        """
        Return True if the given node is inside the graph.
        Otherwise, return False.
        """
        raise NotImplementedError()

    def assert_in(self, node):
        """
        Validates that the given user-facing node is inside the graph.
        Raises an error if the node is invalid.
        """
        if not self.contains(node):
            raise ValueError(f"Node {node} is out of bounds")

    def to_id(self, node) -> int:
        """
        Converts a user-facing node to an internal integer node ID.
        This is used for interfacing with C++ algorithms.
        """
        raise NotImplementedError()

    def to_ids(self, nodes):
        """
        Converts a list of user-facing nodes to internal IDs.
        This is used for interfacing with C++ algorithms.
        """
        cdef vector[int] node_ids
        node_ids = [self.to_id(n) for n in nodes]
        return node_ids

    def from_id(self, int node_id):
        """
        Converts an internal node ID back into a user-facing coordinate or identifier.
        Useful for presenting results back to the use
        """
        raise NotImplementedError()

    def from_ids(self, vector[int] node_ids):
        """
        Converts a list of internal node IDs back into user-facing coordinates or identifiers.
        Useful for presenting pathfinding results back to the use
        """
        return [self.from_id(n) for n in node_ids]


cdef class _SimpleMapper(_NodeMapper):
    cdef int _size

    def __cinit__(self, size):
        self._size = size

    def contains(self, node):
        if int(node) != node:
            return False
        return 0 <= node < self._size

    def to_id(self, node):
        self.assert_in(node)
        return int(node)

    def from_id(self, node_id):
        return int(node_id)


cdef class _Grid2DMapper(_NodeMapper):
    cdef int _w, _h

    def __cinit__(self, w, h):
        self._w = w
        self._h = h

    def contains(self, node):
        if len(node) != 2 or any(x != int(x) for x in node):
            return False
        return 0 <= node[0] < self._w and 0 <= node[1] < self._h

    def to_id(self, node):
        self.assert_in(node)
        return node[0] + node[1] * self._w

    def from_id(self, node_id: int):
        return node_id % self._w, node_id // self._w


cdef class _Grid3DMapper(_NodeMapper):
    cdef int _w, _h, _d, _wh

    def __cinit__(self, w, h, d):
        self._w = w
        self._h = h
        self._d = d
        self._wh = self._w * self._h

    def contains(self, node):
        if len(node) != 3 or any(x != int(x) for x in node):
            return False
        return (
            0 <= node[0] < self._w and
            0 <= node[1] < self._h and
            0 <= node[2] < self._d
        )

    def to_id(self, node):
        self.assert_in(node)
        return node[0] + node[1] * self._w + node[2] * self._wh

    def from_id(self, node_id: int):
        xy = node_id % self._wh
        return (
            xy % self._w,
            xy // self._w,
            node_id // self._wh,
        )


cdef class _AbsGraph:
    """
    Abstract base class for all graph and grid structures.

    This class defines the core interface and behaviors expected of any
    graph representation used in the pathfinding engine, including neighbor
    access, cost calculations, and edge collision handling.
    """

    def __cinit__(self):
        pass

    def _base_init(self, edge_collision=False):
        if edge_collision:
            self.edge_collision = edge_collision

    @property
    def size(self) -> int:
        """
        Total number of nodes in the graph.

        Returns
        -------
        int
            Number of nodes.
        """
        return self._baseobj.size()

    def contains(self, node) -> bool:
        """
        Check if the graph contains a given node.

        Parameters
        ----------
        node
            The node to check.

        Returns
        -------
        bool
            True if the node is in the graph, False otherwise.
        """
        return self._node_mapper.contains(node)

    def calculate_cost(self, path) -> double:
        """
        Calculate the total cost of a given path through the graph.

        If the path is invalid (e.g. contains non-adjacent nodes), this will return `-1`.

        Parameters
        ----------
        path : iterable of nodes
            A sequence of nodes representing the path to evaluate.

        Returns
        -------
        float
            Total cost of traversing the path, or -1 if the path is invalid.

        Raises
        ------
        ValueError
            If one of the nodes in the path is not present in the graph.
        """
        cdef vector[int] nodes = self._node_mapper.to_ids(path)
        if not self._baseobj.is_valid_path(nodes):
            return -1
        return self._baseobj.calculate_cost(nodes)

    def is_valid_path(self, path) -> bool:
        """
        Check if a given path is valid within the graph.

        Parameters
        ----------
        path : iterable of nodes
            A sequence of nodes representing the path

        Returns
        -------
        bool
            True if the path is valid, False otherwise.

        Raises
        ------
        ValueError
            If one of the nodes in the path is not present in the graph.
        """
        cdef vector[int] nodes = self._node_mapper.to_ids(path)
        return self._baseobj.is_valid_path(nodes)

    def get_neighbors(self, node, include_self=False) -> list:
        """
        Get neighboring nodes of a given node, along with the cost of moving to each.

        Each neighbor is returned as a tuple of `(neighbor_node, cost)`.
        For example: `[(n1, 1.0), (n2, 2.5)]`

        Parameters
        ----------
        node
            The target node.

        include_self : bool, default False
            Whether to include the node itself in the results.

        Returns
        -------
        List[Tuple[node, float]]
            List of neighbors with movement costs.

        Raises
        ------
        ValueError
            If the node is not present in the graph.
        """
        map = self._node_mapper
        node_id = map.to_id(node)
        neighbors = self._baseobj.get_neighbors(node_id, False, include_self)
        return [(map.from_id(node_id), weight) for node_id, weight in neighbors]

    def adjacent(self, v1, v2) -> bool:
        """
        Check whether two nodes are directly connected.

        Parameters
        ----------
        v1 : node
            First node.
        v2 : node
            Second node.

        Returns
        -------
        bool
            True if there is a direct edge from `v1` to `v2`.

        Raises
        ------
        ValueError
            If either `v1` or `v2` is not present in the graph.
        """
        v1 = self._node_mapper.to_id(v1)
        v2 = self._node_mapper.to_id(v2)
        return self._baseobj.adjacent(v1, v2)

    @property
    def edge_collision(self) -> bool:
        """
        Whether edge collision checks are enabled.

        Returns
        -------
        bool
            True if edge collisions are checked.
        """
        return self._baseobj.edge_collision()

    @edge_collision.setter
    def edge_collision(self, bool b):
        """
        Enable or disable edge collision checking.

        Parameters
        ----------
        b : bool
            True to enable, False to disable.
        """
        self._baseobj.set_edge_collision(b)

    def to_dict(self) -> dict:
        """
        Convert graph settings to a serializable dictionary.

        Returns
        -------
        dict
            Dictionary of graph configuration options.
        """
        return {"edge_collision": self.edge_collision}

    def __copy__(self):
        return self.__class__(**self.to_dict())

    def __reduce__(self):
        return _construct, (self.__class__, self.to_dict())


def _construct(cls, kw):
    return cls(**kw)


cdef class Graph(_AbsGraph):
    """
    A basic graph implementation with optional coordinates and edge support.

    This class supports both directed and undirected graphs. Nodes can be
    optionally associated with coordinates.

    Parameters
    ----------
    num_vertices : int
        The number of vertices (nodes) in the graph.

    directed : bool, default True
        Whether the graph is directed (`True`) or undirected (`False`).

    edges : iterable of tuple, optional
        Initial edges to populate the graph. Each edge can be either:

        - (from_node, to_node)
        - (from_node, to_node, weight)

        If the weight is omitted, it defaults to 1.0.

    coordinates : list of list of float, optional
        A list of coordinates for each node. Coordinates can be in 2D, 3D, or higher dimensions.
        Used to enable distance-based heuristics (e.g. Euclidean distance) in A*-like pathfinding algorithms.
        If not provided, heuristic-based algorithms that depend on node positions will not function.

    edge_collision : bool, default False
        Whether to enable edge collision checks.
        If set to `True`, prevents two agents from using the same edge simultaneously,
        even if they are moving in opposite directions.
        This helps avoid edge conflicts (e.g. node swapping) in multi-agent pathfinding scenarios.


    Example
    -------
    Create a directed graph with 3 nodes and 2 edges (0 → 1 → 2):

    >>>  graph = Graph(3, edges=[(0, 1), (1, 2)])

    """

    def __cinit__(
        self,
        unsigned int num_vertices,
        *,
        bool directed=True,
        coordinates=None,
        edges=None,
        **kwargs,
    ):
        self._obj = new cdefs.Graph(num_vertices, directed)
        self._baseobj = self._obj
        self._num_vertices = num_vertices
        self._directed = directed
        self._node_mapper = _SimpleMapper(self._num_vertices)
        if coordinates is not None:
            self.set_coordinates(coordinates)
        if edges is not None:
            self.add_edges(edges)
        self._base_init(**kwargs)

    def __dealloc__(self):
        del self._obj

    def __repr__(self):
        return f"Graph(num_vertices={self.num_vertices}, num_edges={self.num_edges})"

    @property
    def num_vertices(self) -> int:
        """
        Total number of vertices in the graph. Equivalent to calling the `size` property.

        Returns
        -------
        int
            Number of vertices (nodes).
        """
        return self._num_vertices

    @property
    def directed(self) -> bool:
        """
        Whether the graph is directed.

        Returns
        -------
        bool
            True if the graph is directed.
        """
        return self._directed

    def set_coordinates(self, vector[vector[double]] coordinates):
        """
        Set spatial coordinates for each node in the graph.

        Each coordinate defines the position of a node in space and enables
        distance-based heuristics (e.g. Euclidean distance) in A*-like algorithms.

        Parameters
        ----------
        coordinates : list of list of float
            A list of coordinates, one per node. All coordinates must have the
            same number of dimensions (e.g. 2D, 3D, etc.).

        Raises
        ------
        ValueError
            If the number of coordinates does not match the number of vertices.

        ValueError
            If coordinates have inconsistent dimensionality (e.g. some are 2D, others 3D).
        """
        if (len(coordinates) != self.num_vertices):
            raise ValueError(
                "Number of elements must be equal to the number of vertices"
            )

        if self.num_vertices > 0:
            num_dimensions = len(coordinates[0])
            if any(len(x) != num_dimensions for x in coordinates):
                raise ValueError(
                    "The number of dimensions should not change from vertex to vertex"
                )

            if num_dimensions == 0:
                raise ValueError(
                    "The number of dimensions must be greater than zero"
                )

        self._obj.set_coordinates(coordinates)

    def has_coordinates(self) -> bool:
        """
        Check whether node coordinates are defined.

        Returns
        -------
        bool
            True if coordinates have been set.
        """
        return self._obj.has_coordinates()

    def estimate_distance(self, int v1, int v2) -> double:
        """
        Estimate the geometric (Euclidean) distance between two nodes.

        Requires that node coordinates are set using `set_coordinates()` beforehand.
        If coordinates are not available, a `RuntimeError` is raised.

        Parameters
        ----------
        v1 : int
            ID of the first node.
        v2 : int
            ID of the second node.

        Returns
        -------
        float
            Euclidean distance between the two nodes.

        Raises
        ------
        RuntimeError
            If coordinates have not been set.
        ValueError
            If one of the nodes is not present in the graph.
        """
        if not self.has_coordinates():
            raise RuntimeError("estimate_distance requires node coordinates to be set")
        v1 = self._node_mapper.to_id(v1)
        v2 = self._node_mapper.to_id(v2)
        return self._obj.estimate_distance(v1, v2)

    @property
    def num_edges(self) -> int:
        """
        Number of edges in the graph.

        Returns
        -------
        int
            Total edge count.
        """
        return self._obj.num_edges()

    @property
    def edges(self):
        """
        Get all edges in the graph.

        Returns
        -------
        List[Tuple[int, int, float]]
            List of edges as (from, to, weight).
        """
        map = self._node_mapper
        data = []
        for (start, end, cost) in self._obj.get_edges():
            data.append([map.from_id(start), map.from_id(end), cost])
        return data

    @property
    def coordinates(self):
        """
        Get the coordinates of all nodes in the graph.

        Returns
        -------
        List[List[float]] or None
            A list of coordinate vectors, one per node,
            or `None` if coordinates have not been set.
        """
        if not self.has_coordinates():
            return None
        return self._obj.get_coordinates()

    def add_edges(self, edges):
        """
        Add multiple edges to the graph.

        Each edge can be a 2-tuple `(from_node, to_node)` or a 3-tuple
        `(from_node, to_node, weight)`. If weight is omitted, a default of `1.0` is used.

        Parameters
        ----------
        edges : Iterable[Tuple[int, int] or Tuple[int, int, float]]
            Iterable of edges to add.

        Raises
        ------
        ValueError
            If an edge contains a node not present in the graph.
        """
        map = self._node_mapper
        starts, ends, costs = [], [], []
        for edge in edges:
            if len(edge) == 2:
                start, end = edge
                cost = 1
            else:
                start, end, cost = edge
            start = map.to_id(start)
            end = map.to_id(end)
            if cost < 0:
                raise ValueError("Weight cannot be negative!")
            starts.append(start)
            ends.append(end)
            costs.append(cost)
        self._obj.add_edges(starts, ends, costs)

    def reverse(self, *, bool inplace=False):
        """
        Reverse the direction of all edges.

        Parameters
        ----------
        inplace : bool, default False
            If True, modify the graph in-place; otherwise return a new reversed graph.

        Returns
        -------
        Graph or None
            Reversed graph if not in-place; otherwise None.
        """

        if inplace:
            self._obj.reverse_inplace()
            return

        reversed_graph = Graph(self.num_vertices)
        del reversed_graph._obj
        reversed_graph._obj = self._obj.create_reversed_graph()
        reversed_graph._baseobj = reversed_graph._obj
        return reversed_graph

    def find_components(self):
        """
        Find connected components (only for undirected graphs).

        Returns
        -------
        List[List[int]]
            List of components, each as a list of nodes.

        Raises
        ------
        ValueError
            If called on a directed graph.
        """
        if self.directed:
            raise ValueError("find_components only works for an undirected graph")
        return self._obj.find_components()

    def find_scc(self):
        """
        Find strongly connected components (only for directed graphs).

        Returns
        -------
        List[List[int]]
            List of strongly connected components.

        Raises
        ------
        ValueError
            If called on an undirected graph.
        """
        if not self.directed:
            raise ValueError("find_scc only works for a directed graph")

        return self._obj.find_scc()

    @property
    def edge_collision(self) -> bool:
        """
        Whether edge collision checks are enabled.

        Returns
        -------
        bool
            True if edge collisions are checked.
        """
        return self._baseobj.edge_collision()

    @edge_collision.setter
    def edge_collision(self, bool b):
        if b and not self.directed:
            raise ValueError("An undirected graph does not support edge collisions")
        self._baseobj.set_edge_collision(b)

    def to_dict(self):
        return {
            "num_vertices": self.num_vertices,
            "directed": self.directed,
            "coordinates": self.coordinates,
            "edges": self.edges,
            **super().to_dict(),
        }


cdef class _AbsGrid(_AbsGraph):

    def _base_init(self, pause_weights=None, **kwargs):
        if pause_weights is not None:
            self.pause_weights = pause_weights
        return super()._base_init(**kwargs)

    @property
    def shape(self):
        raise NotImplementedError()

    def __repr__(self):
        shape_str = "x".join([str(x) for x in self.shape])
        return f"{self.__class__.__name__}({shape_str})"

    def has_obstacle(self, point):
        node_id = self._node_mapper.to_id(point)
        return self._basegridobj.has_obstacle(node_id)

    def add_obstacle(self, point):
        node_id = self._node_mapper.to_id(point)
        self._basegridobj.add_obstacle(node_id)

    def remove_obstacle(self, point):
        node_id = self._node_mapper.to_id(point)
        self._basegridobj.remove_obstacle(node_id)

    def update_weight(self, point, new_value):
        node_id = self._node_mapper.to_id(point)
        self._basegridobj.update_weight(node_id, new_value)

    def get_weight(self, point):
        node_id = self._node_mapper.to_id(point)
        return self._basegridobj.get_weight(node_id)

    def find_components(self):
        return [
            self._node_mapper.from_ids(component)
            for component in self._baseobj.find_components()
        ]

    @property
    def weights(self):
        return self._convert(self._basegridobj.get_weights(), self.shape)

    @weights.setter
    def weights(self, data):
        if self._get_shape(data) != self.shape:
            shape_str = "x".join([str(x) for x in self.shape])
            raise ValueError(f"Weights must have shape {shape_str}")

        vector = self._convert(data)
        if len(vector) != self.size:
            raise ValueError("Weights have an inhomogeneous shape")

        self._basegridobj.set_weights(vector)

    def get_pause_weight(self, point):
        node_id = self._node_mapper.to_id(point)
        return self._basegridobj.get_pause_weight(node_id)

    @property
    def pause_weights(self):
        return self._convert(self._basegridobj.get_pause_weights(), self.shape)

    @pause_weights.setter
    def pause_weights(self, data):
        if not hasattr(data, '__iter__'):
            self._basegridobj.set_pause_weight(data)
            return

        if self._get_shape(data) != self.shape:
            shape_str = "x".join([str(x) for x in self.shape])
            raise ValueError(f"Weights must have shape {shape_str}")

        vector = self._convert(data)
        if len(vector) != self.size:
            raise ValueError("Weights have an inhomogeneous shape")

        self._basegridobj.set_pause_weights(vector)

    @property
    def obstacle_map(self):
        weights = self._basegridobj.get_weights()
        weights = [int(w == -1) for w in weights]
        return self._convert(weights, self.shape)

    @staticmethod
    def _get_shape(data):
        shape = []
        d = data
        while hasattr(d, "__len__"):
            shape.append(len(d))
            if len(d) == 0:
                break
            d = d[0]

        def check_shape(arr, shape):
            if len(arr) != shape[0]:
                return False

            if len(shape) == 1:
                return True

            shape_ = shape[1:]
            for x in arr:
                if not check_shape(x, shape_):
                    return False

            return True

        if not check_shape(data, shape):
            raise ValueError("Data has an inhomogeneous shape")

        return tuple(shape)

    @staticmethod
    def _convert(data, shape=None):
        if shape is None:
            # matrix -> vector
            vector = []

            def flatten(arr):
                if len(arr) == 0:
                    return
                if hasattr(arr[0], "__len__"):
                    for x in arr:
                        flatten(x)
                else:
                    vector.extend(arr)

            flatten(data)
            return vector

        # vector -> matrix
        num_elements = 1
        for s in shape:
            num_elements *= s

        if len(data) != num_elements:
            raise ValueError("The total number of elements must match the shape")

        def create_matrix(arr, shape):
            if len(shape) == 1:
                return arr
            else:
                size = int(len(arr) / shape[0])
                shape_ = shape[1:]
                return [
                    create_matrix(arr[i * size:(i + 1) * size], shape_)
                    for i in range(shape[0])
                ]

        return create_matrix(data, shape)

    def to_dict(self):
        d = super().to_dict()
        d["weights"] = self.weights
        d["pause_weights"] = self.pause_weights
        return d


cdef class Grid(_AbsGrid):

    def __cinit__(
        self,
        weights=None,
        *,
        width=None,
        height=None,
        int diagonal_movement=0,
        bool passable_left_right_border=False,
        bool passable_up_down_border=False,
        double diagonal_movement_cost_multiplier=1,
        **kwargs,
    ):

        if weights is None:
            if not isinstance(width, int) or not isinstance(height, int):
                raise ValueError("Either weights or height and width must be provided.")
        else:
            height, width = self._get_shape(weights)

        if width <= 0:
            raise ValueError("Width must be greater than zero.")

        if height <= 0:
            raise ValueError("Height must be greater than zero.")

        self.width = width
        self.height = height
        self._node_mapper = _Grid2DMapper(self.width, self.height)

        if weights is None:
            self._obj = new cdefs.Grid(width, height)
        else:
            self._obj = new cdefs.Grid(width, height, self._convert(weights))

        self._baseobj = self._obj
        self._basegridobj = self._obj

        if diagonal_movement:
            self.diagonal_movement = diagonal_movement

        if passable_left_right_border:
            self.passable_left_right_border = passable_left_right_border

        if passable_up_down_border:
            self.passable_up_down_border = passable_up_down_border

        if diagonal_movement_cost_multiplier != 1:
            self.diagonal_movement_cost_multiplier = diagonal_movement_cost_multiplier

        self._base_init(**kwargs)

    def __dealloc__(self):
        del self._obj
        self._obj = NULL
        self._baseobj = NULL
        self._basegridobj = NULL

    @property
    def shape(self):
        return self.height, self.width

    @property
    def diagonal_movement(self):
        return DiagonalMovement(self._obj.get_diagonal_movement())

    @diagonal_movement.setter
    def diagonal_movement(self, int _x):
        diagonal_movement = DiagonalMovement(_x)
        self._obj.set_diagonal_movement(diagonal_movement)

    @property
    def passable_left_right_border(self):
        return self._obj.passable_left_right_border

    @passable_left_right_border.setter
    def passable_left_right_border(self, bool _b):
        self._obj.passable_left_right_border = _b

    @property
    def passable_up_down_border(self):
        return self._obj.passable_up_down_border

    @passable_up_down_border.setter
    def passable_up_down_border(self, bool _b):
        self._obj.passable_up_down_border = _b

    @property
    def diagonal_movement_cost_multiplier(self):
        return self._obj.diagonal_movement_cost_multiplier

    @diagonal_movement_cost_multiplier.setter
    def diagonal_movement_cost_multiplier(self, double m):
        if not 1 <= m <= 2:
            raise ValueError("diagonal_movement_cost_multiplier must be in range [1, 2].")
        self._obj.diagonal_movement_cost_multiplier = m

    def show_path(self, path):
        if path:
            start = path[0]
            end = path[-1]
        else:
            path = []
            start = None
            end = None

        line = "+ " + " ".join("-" * self.width) + " +"
        str_grid = line + "\n"
        for y in range(self.height):
           row = []
           for x in range(self.width):
               k = (x, y)
               if self.has_obstacle(k):
                   c = "#"
               elif k == start:
                   c = "s"
               elif k == end:
                   c = "e"
               elif k in path:
                   c = "x"
               else:
                   c = " "
               row.append(c)
           str_grid += "| " + " ".join(row) + " |\n"
        str_grid += line
        print(str_grid)

    def show(self):
        self.show_path(None)

    def to_dict(self):
        return {
            "width": self.width,
            "height": self.height,
            "diagonal_movement": self.diagonal_movement.value,
            "passable_left_right_border": self.passable_left_right_border,
            "passable_up_down_border": self.passable_up_down_border,
            "diagonal_movement_cost_multiplier": self.diagonal_movement_cost_multiplier,
            **super().to_dict(),
        }


cdef class Grid3D(_AbsGrid):

    def __cinit__(
        self,
        weights=None,
        *,
        width=None,
        height=None,
        depth=None,
        bool passable_borders=False,
        **kwargs,
    ):

        if weights is None:
            if not isinstance(width, int) or not isinstance(height, int) or not isinstance(depth, int):
                raise ValueError("Either weights or height and width and depth must be provided.")
        else:
            depth, height, width = self._get_shape(weights)

        if width <= 0:
            raise ValueError("Width must be greater than zero.")

        if height <= 0:
            raise ValueError("Height must be greater than zero.")

        if depth <= 0:
            raise ValueError("Depth must be greater than zero.")

        self.width = width
        self.height = height
        self.depth = depth
        self._node_mapper = _Grid3DMapper(self.width, self.height, self.depth)

        if weights is None:
            self._obj = new cdefs.Grid3D(width, height, depth)
        else:
            self._obj = new cdefs.Grid3D(width, height, depth, self._convert(weights))

        self._baseobj = self._obj
        self._basegridobj = self._obj

        if passable_borders:
            self.passable_borders = passable_borders

        self._base_init(**kwargs)

    def __dealloc__(self):
        del self._obj

    @property
    def diagonal_movement(self):
        return 0

    @property
    def passable_borders(self):
        return self._obj.passable_borders

    @passable_borders.setter
    def passable_borders(self, bool _b):
        self._obj.passable_borders = _b

    @property
    def shape(self):
        return self.depth, self.height, self.width

    def to_dict(self):
        return {
            "width": self.width,
            "height": self.height,
            "depth": self.depth,
            "passable_borders": self.passable_borders,
            **super().to_dict(),
        }


cdef class HexGrid(_AbsGrid):
    # Hexagonal Grid

    def __cinit__(
        self,
        weights=None,
        *,
        width=None,
        height=None,
        int layout=0,
        bool passable_left_right_border=False,
        bool passable_up_down_border=False,
        **kwargs,
    ):

        if weights is None:
            if not isinstance(width, int) or not isinstance(height, int):
                raise ValueError("Either weights or height and width must be provided.")
        else:
            height, width = self._get_shape(weights)

        if width <= 0:
            raise ValueError("Width must be greater than zero.")

        if height <= 0:
            raise ValueError("Height must be greater than zero.")

        self.width = width
        self.height = height
        layout = HexLayout(layout)
        self._node_mapper = _Grid2DMapper(self.width, self.height)

        if weights is None:
            self._obj = new cdefs.HexGrid(width, height, layout)
        else:
            self._obj = new cdefs.HexGrid(width, height, layout, self._convert(weights))

        self._baseobj = self._obj
        self._basegridobj = self._obj

        if passable_left_right_border:
            self.passable_left_right_border = passable_left_right_border

        if passable_up_down_border:
            self.passable_up_down_border = passable_up_down_border

        self._base_init(**kwargs)

    def __dealloc__(self):
        del self._obj

    @property
    def shape(self):
        return self.height, self.width

    @property
    def layout(self):
        return HexLayout(self._obj.layout)

    def is_flat_top_layout(self):
        return self.layout.is_flat_top()

    def is_pointy_top_layout(self):
        return self.layout.is_pointy_top()

    @property
    def passable_left_right_border(self):
        return self._obj.passable_left_right_border

    @passable_left_right_border.setter
    def passable_left_right_border(self, bool _b):
        if _b and self.is_flat_top_layout() and self.width % 2 == 1:
            raise ValueError(
                "With flat_top layout the left-right border can only be passable, "
                "if the width is even"
            )
        self._obj.passable_left_right_border = _b

    @property
    def passable_up_down_border(self):
        return self._obj.passable_up_down_border

    @passable_up_down_border.setter
    def passable_up_down_border(self, bool _b):
        if _b and self.is_pointy_top_layout() and self.height % 2 == 1:
            raise ValueError(
                "With pointy_top layout the up-down border can only be passable, "
                "if the height is even"
            )
        self._obj.passable_up_down_border = _b

    def to_dict(self):
        return {
            "width": self.width,
            "height": self.height,
            "layout": self.layout.value,
            "passable_left_right_border": self.passable_left_right_border,
            "passable_up_down_border": self.passable_up_down_border,
            **super().to_dict(),
        }
