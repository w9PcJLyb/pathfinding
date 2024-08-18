# distutils: language = c++

from libcpp cimport bool
from libcpp.vector cimport vector
from w9_pathfinding.cdefs cimport (
    AbsGraph as CAbsGraph,
    AbsPathFinder as CAbsPathFinder,
    Graph as CGraph,
    Grid as CGrid,
    Grid3D as CGrid3D,
    HexGrid as CHexGrid,
    DFS as CDFS,
    BFS as CBFS,
    BiBFS as CBiBFS,
    Dijkstra as CDijkstra,
    BiDijkstra as CBiDijkstra,
    AStar as CAStar,
    BiAStar as CBiAStar,
    GBS as CGBS,
    IDAStar as CIDAStar,
    SpaceTimeAStar as CSpaceTimeAStar,
    ReservationTable as CReservationTable,
    AbsMAPF as CAbsMAPF,
    HCAStar as CHCAStar,
    WHCAStar as CWHCAStar,
    CBS as CCBS,
)
from w9_pathfinding.hex_layout import HexLayout
from w9_pathfinding.diagonal_movement import DiagonalMovement


cdef class _AbsGraph:
    cdef CAbsGraph* _baseobj

    def __cinit__(self):
        pass

    def _base_init(self, pause_action_cost=1, edge_collision=False):
        if pause_action_cost != 1:
            self.pause_action_cost = pause_action_cost

        if edge_collision:
            self.edge_collision = edge_collision

    def size(self):
        return self._baseobj.size()

    def calculate_cost(self, vector[int] path):
        return self._baseobj.calculate_cost(path)

    def get_neighbors(self, int node_id):
        # return [[neighbour_id, cost], ...]
        return self._baseobj.get_neighbors(node_id)

    def adjacent(self, int v1, int v2):
        return self._baseobj.adjacent(v1, v2)

    @property
    def pause_action_cost(self):
        return self._baseobj.get_pause_action_cost()

    @pause_action_cost.setter
    def pause_action_cost(self, double cost):
        if cost < 0 and cost != -1:
            raise ValueError("pause_action_cost must be either non-negative or equal to -1")
        self._baseobj.set_pause_action_cost(cost)

    def is_pause_action_allowed(self):
        return self._baseobj.is_pause_action_allowed()

    @property
    def edge_collision(self):
        return self._baseobj.edge_collision()

    @edge_collision.setter
    def edge_collision(self, bool b):
        self._baseobj.set_edge_collision(b)

    def to_dict(self):
        return {"edge_collision": self.edge_collision, "pause_action_cost": self.pause_action_cost}


cdef class Graph(_AbsGraph):
    cdef CGraph* _obj
    cdef readonly int num_vertices
    cdef readonly bool directed

    def __cinit__(
        self,
        unsigned int num_vertices,
        *,
        bool directed=True,
        coordinates=None,
        edges=None,
        **kwargs,
    ):
        self._obj = new CGraph(num_vertices, directed)
        self._baseobj = self._obj
        self.num_vertices = num_vertices
        self.directed = directed
        if coordinates:
            self.set_coordinates(coordinates)
        if edges:
            self.add_edges(edges)
        self._base_init(**kwargs)

    def __dealloc__(self):
        del self._obj

    def __repr__(self):
        return f"Graph(num_vertices={self.num_vertices}, num_edges={self.num_edges})"

    def set_coordinates(self, vector[vector[double]] coordinates):
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

    def has_coordinates(self):
        return self._obj.has_coordinates()

    def estimate_distance(self, int v1, int v2):
        if not self.has_coordinates():
            return
        self.assert_in(v1)
        self.assert_in(v2)
        return self._obj.estimate_distance(v1, v2)

    @property
    def num_edges(self):
        return self._obj.num_edges()

    @property
    def edges(self):
        data = []
        for (start, end, cost) in self._obj.get_edges():
            data.append([int(start), int(end), cost])
        return data

    @property
    def coordinates(self):
        return self._obj.get_coordinates()

    def assert_in(self, int node_id):
        if not 0 <= node_id < self.num_vertices:
            raise ValueError(f"Node with id {node_id} does not exist")

    def add_edges(self, edges):
        starts, ends, costs = [], [], []
        for edge in edges:
            if len(edge) == 2:
                start, end = edge
                cost = 1
            else:
                start, end, cost = edge
            self.assert_in(start)
            self.assert_in(end)
            if cost < 0:
                raise ValueError("Weight cannot be negative!")
            starts.append(start)
            ends.append(end)
            costs.append(cost)
        self._obj.add_edges(starts, ends, costs)

    def reverse(self, *, bool inplace=False):
        if inplace:
            self._obj.reverse_inplace()
            return

        reversed_graph = Graph(self.num_vertices)
        del reversed_graph._obj
        reversed_graph._obj = self._obj.create_reversed_graph()
        reversed_graph._baseobj = reversed_graph._obj
        return reversed_graph

    def find_components(self):
        if self.directed:
            raise ValueError("find_components only works for an undirected graph")
        return self._obj.find_components()

    def find_scc(self):
        # find the strongly connected components of a directed graph
        if not self.directed:
            raise ValueError("find_scc only works for a directed graph")

        return self._obj.find_scc()

    @property
    def edge_collision(self):
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
    def assert_in(self, point):
        raise NotImplementedError()

    def get_node_id(self, point):
        raise NotImplementedError()

    def get_coordinates(self, int node_id):
        raise NotImplementedError()

    def get_neighbors(self, point):
        node_id = self.get_node_id(point)
        neighbours = []
        for n, cost in self._baseobj.get_neighbors(node_id):
            neighbours.append((self.get_coordinates(n), cost))
        return neighbours

    def calculate_cost(self, path):
        cdef vector[int] nodes
        nodes = [self.get_node_id(x) for x in path]
        return self._baseobj.calculate_cost(nodes)

    def find_components(self):
        return [
            [self.get_coordinates(node_id) for node_id in component]
            for component in self._baseobj.find_components()
        ]

    def adjacent(self, p1, p2):
        return self._baseobj.adjacent(self.get_node_id(p1), self.get_node_id(p2))


cdef class Grid(_AbsGrid):
    cdef CGrid* _obj
    cdef readonly int width, height

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
            height, width = len(weights), len(weights[0])

        if width <= 0:
            raise ValueError("Width must be greater than zero.")

        if height <= 0:
            raise ValueError("Height must be greater than zero.")

        self.width = width
        self.height = height

        if weights is None:
            self._obj = new CGrid(width, height)
        else:
            self._check_weights(weights)
            self._obj = new CGrid(weights)

        self._baseobj = self._obj

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

    def __repr__(self):
        return f"Grid({self.width}x{self.height})"

    def assert_in(self, point):
        if not 0 <= point[0] < self.width or not 0 <= point[1] < self.height:
            raise ValueError(f"Point {point} is out of the {self}")

    def get_node_id(self, point):
        self.assert_in(point)
        return point[0] + point[1] * self.width

    def get_coordinates(self, int node_id):
        return node_id % self.width, node_id // self.width

    def has_obstacle(self, point):
        self.assert_in(point)
        return self._obj.has_obstacle(self.get_node_id(point))

    def add_obstacle(self, point):
        self.assert_in(point)
        self._obj.add_obstacle(self.get_node_id(point))

    def remove_obstacle(self, point):
        self.assert_in(point)
        self._obj.remove_obstacle(self.get_node_id(point))

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

    @property
    def obstacle_map(self):
        weights = self._obj.get_weights()
        map = []
        for y in range(self.height):
            row = [int(weights[self.get_node_id((x, y))] == -1) for x in range(self.width)]
            map.append(row)
        return map

    def _check_weights(self, weights):
        height, width = len(weights), len(weights[0])
        if height != self.height or width != self.width:
            raise ValueError(f"weights.shape must be {self.width}x{self.height}")
        for row in weights:
            for w in row:
                if w < 0 and w != -1:
                    raise ValueError("Weight must be either non-negative or equal to -1")

    @property
    def weights(self):
        weights = self._obj.get_weights()
        matrix = []
        for y in range(self.height):
            row = [weights[self.get_node_id((x, y))] for x in range(self.width)]
            matrix.append(row)
        return matrix

    @weights.setter
    def weights(self, matrix):
        self._check_weights(matrix)
        self._obj.set_weights(sum(matrix, []))

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
            "weights": self.weights,
            "diagonal_movement": self.diagonal_movement.value,
            "passable_left_right_border": self.passable_left_right_border,
            "passable_up_down_border": self.passable_up_down_border,
            "diagonal_movement_cost_multiplier": self.diagonal_movement_cost_multiplier,
            **super().to_dict(),
        }


cdef class Grid3D(_AbsGrid):
    cdef CGrid3D* _obj
    cdef readonly int width, height, depth

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
            cut = weights[0]
            depth, height, width = len(weights), len(cut), len(cut[0])

        if width <= 0:
            raise ValueError("Width must be greater than zero.")

        if height <= 0:
            raise ValueError("Height must be greater than zero.")

        if depth <= 0:
            raise ValueError("Depth must be greater than zero.")

        self.width = width
        self.height = height
        self.depth = depth

        if weights is None:
            self._obj = new CGrid3D(width, height, depth)
        else:
            self._check_weights(weights)
            self._obj = new CGrid3D(weights)

        self._baseobj = self._obj

        if passable_borders:
            self.passable_borders = passable_borders

        self._base_init(**kwargs)

    def __dealloc__(self):
        del self._obj

    def __repr__(self):
        return f"Grid3D({self.width}x{self.height}x{self.depth})"

    @property
    def diagonal_movement(self):
        return 0

    @property
    def passable_borders(self):
        return self._obj.passable_borders

    @passable_borders.setter
    def passable_borders(self, bool _b):
        self._obj.passable_borders = _b

    def assert_in(self, point):
        if not 0 <= point[0] < self.width or not 0 <= point[1] < self.height or not 0 <= point[2] < self.depth:
            raise ValueError(f"Point {point} is out of the {self}")

    def get_node_id(self, point):
        self.assert_in(point)
        return point[0] + point[1] * self.width + point[2] * self.width * self.height

    def get_coordinates(self, int node_id):
        xy = node_id % (self.width * self.height)
        return xy % self.width, xy // self.width, node_id // (self.width * self.height)

    def has_obstacle(self, point):
        return self._obj.has_obstacle(self.get_node_id(point))

    def add_obstacle(self, point):
        self._obj.add_obstacle(self.get_node_id(point))

    def remove_obstacle(self, point):
        self._obj.remove_obstacle(self.get_node_id(point))

    @property
    def obstacle_map(self):
        weights = self.weights
        for i in range(self.depth):
            for j in range(self.height):
                weights[i][j] = [int(x < 0) for x in weights[i][j]]
        return map

    def _check_weights(self, weights):
        cut = weights[0]
        depth, height, width = len(weights), len(cut), len(cut[0])
        if depth != self.depth or height != self.height or width != self.width:
            raise ValueError(f"weights.shape must be {self.width}x{self.height}x{self.depth}")
        for cut in weights:
            for row in cut:
                for w in row:
                    if w < 0 and w != -1:
                        raise ValueError("Weight must be positive or equal to -1")

    @property
    def weights(self):
        weights = self._obj.get_weights()
        matrix = []
        for z in range(self.depth):
            matrix.append([])
            for y in range(self.height):
                row = [weights[self.get_node_id((x, y, z))] for x in range(self.width)]
                matrix[-1].append(row)
        return matrix

    @weights.setter
    def weights(self, matrix):
        self._check_weights(matrix)
        self._obj.set_weights(sum(sum(matrix, []), []))

    def to_dict(self):
        return {
            "width": self.width,
            "height": self.height,
            "depth": self.depth,
            "weights": self.weights,
            "passable_borders": self.passable_borders,
            **super().to_dict(),
        }


cdef class HexGrid(_AbsGrid):
    # Hexagonal Grid

    cdef CHexGrid* _obj
    cdef readonly int width, height

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
            height, width = len(weights), len(weights[0])

        if width <= 0:
            raise ValueError("Width must be greater than zero.")

        if height <= 0:
            raise ValueError("Height must be greater than zero.")

        self.width = width
        self.height = height
        layout = HexLayout(layout)

        if weights is None:
            self._obj = new CHexGrid(width, height, layout)
        else:
            self._check_weights(weights)
            self._obj = new CHexGrid(layout, weights)

        self._baseobj = self._obj

        if passable_left_right_border:
            self.passable_left_right_border = passable_left_right_border

        if passable_up_down_border:
            self.passable_up_down_border = passable_up_down_border

        self._base_init(**kwargs)

    def __dealloc__(self):
        del self._obj

    def __repr__(self):
        return f"HexGrid({self.width}x{self.height})"

    def assert_in(self, point):
        if not 0 <= point[0] < self.width or not 0 <= point[1] < self.height:
            raise ValueError(f"Point {point} is out of the {self}")

    def get_node_id(self, point):
        self.assert_in(point)
        return point[0] + point[1] * self.width

    def get_coordinates(self, int node_id):
        return node_id % self.width, node_id // self.width

    def has_obstacle(self, point):
        self.assert_in(point)
        return self._obj.has_obstacle(self.get_node_id(point))

    def add_obstacle(self, point):
        self.assert_in(point)
        self._obj.add_obstacle(self.get_node_id(point))

    def remove_obstacle(self, point):
        self.assert_in(point)
        self._obj.remove_obstacle(self.get_node_id(point))

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

    @property
    def obstacle_map(self):
        weights = self._obj.get_weights()
        map = []
        for y in range(self.height):
            row = [int(weights[self.get_node_id((x, y))] == -1) for x in range(self.width)]
            map.append(row)
        return map

    def _check_weights(self, weights):
        height, width = len(weights), len(weights[0])
        if height != self.height or width != self.width:
            raise ValueError(f"weights.shape must be {self.width}x{self.height}")
        for row in weights:
            for w in row:
                if w < 0 and w != -1:
                    raise ValueError("Weight must be either non-negative or equal to -1")

    @property
    def weights(self):
        weights = self._obj.get_weights()
        matrix = []
        for y in range(self.height):
            row = [weights[self.get_node_id((x, y))] for x in range(self.width)]
            matrix.append(row)
        return matrix

    @weights.setter
    def weights(self, matrix):
        self._check_weights(matrix)
        self._obj.set_weights(sum(matrix, []))

    def to_dict(self):
        return {
            "width": self.width,
            "height": self.height,
            "weights": self.weights,
            "layout": self.layout.value,
            "passable_left_right_border": self.passable_left_right_border,
            "passable_up_down_border": self.passable_up_down_border,
            **super().to_dict(),
        }


def _pathfinding(func):

    def wrap(finder, start, goal, **kwargs):
        g = finder.graph

        if isinstance(g, Graph):
            g.assert_in(start)
            g.assert_in(goal)
            path = func(finder, start, goal, **kwargs)

        elif isinstance(g, (Grid, Grid3D, HexGrid)):
            start = g.get_node_id(start)
            goal = g.get_node_id(goal)
            path = func(finder, start, goal, **kwargs)
            path = [g.get_coordinates(node) for node in path]

        else:
            raise NotImplementedError

        return path

    return wrap


cdef class _AbsPathFinder():
    cdef CAbsPathFinder* _baseobj
    cdef public _AbsGraph graph

    def __cinit__(self):
        pass

    def __repr__(self):
        return f"{self.__class__.__name__}(graph={self.graph})"

    @_pathfinding
    def find_path(self, start, goal):
        return self._baseobj.find_path(start, goal)


cdef class DFS(_AbsPathFinder):
    cdef CDFS* _obj

    def __cinit__(self, _AbsGraph graph):
        self.graph = graph
        self._obj = new CDFS(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj


cdef class BFS(_AbsPathFinder):
    cdef CBFS* _obj

    def __cinit__(self, _AbsGraph graph):
        self.graph = graph
        self._obj = new CBFS(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj


cdef class BiBFS(_AbsPathFinder):
    cdef CBiBFS* _obj

    def __cinit__(self, _AbsGraph graph):
        self.graph = graph
        self._obj = new CBiBFS(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj


cdef class Dijkstra(_AbsPathFinder):
    cdef CDijkstra* _obj
    
    def __cinit__(self, _AbsGraph graph):
        self.graph = graph
        self._obj = new CDijkstra(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj


cdef class BiDijkstra(_AbsPathFinder):
    cdef CBiDijkstra* _obj

    def __cinit__(self, _AbsGraph graph):
        self.graph = graph
        self._obj = new CBiDijkstra(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj


cdef class AStar(_AbsPathFinder):
    cdef CAStar* _obj

    def __cinit__(self, _AbsGraph graph):
        if isinstance(graph, Graph) and not graph.has_coordinates():
            raise ValueError(
                "A* cannot work with a graph without coordinates. "
                "You can add coordinates using graph.set_coordinates(), "
                "or choose some non-heuristic algorithm."
            )
        self.graph = graph
        self._obj = new CAStar(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj


cdef class BiAStar(_AbsPathFinder):
    cdef CBiAStar* _obj

    def __cinit__(self, _AbsGraph graph):
        if isinstance(graph, Graph) and not graph.has_coordinates():
            raise ValueError(
                "A* cannot work with a graph without coordinates. "
                "You can add coordinates using graph.set_coordinates(), "
                "or choose some non-heuristic algorithm."
            )
        self.graph = graph
        self._obj = new CBiAStar(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj


cdef class GBS(_AbsPathFinder):
    # Greedy Best-first Search

    cdef CGBS* _obj

    def __cinit__(self, _AbsGraph graph):
        if isinstance(graph, Graph) and not graph.has_coordinates():
            raise ValueError(
                "GBS cannot work with a graph without coordinates. "
                "You can add coordinates using graph.set_coordinates(), "
                "or choose some non-heuristic algorithm."
            )
        self.graph = graph
        self._obj = new CGBS(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj


cdef class IDAStar(_AbsPathFinder):
    # Iterative deepening A*

    cdef CIDAStar* _obj

    def __cinit__(self, _AbsGraph graph):
        if isinstance(graph, Graph) and not graph.has_coordinates():
            raise ValueError(
                "IDA* cannot work with a graph without coordinates. "
                "You can add coordinates using graph.set_coordinates(), "
                "or choose some non-heuristic algorithm."
            )
        self.graph = graph
        self._obj = new CIDAStar(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj

    @_pathfinding
    def find_path(self, int start, int goal, double max_distance=10):
        return self._obj.find_path(start, goal, max_distance)


cdef class SpaceTimeAStar(_AbsPathFinder):
    cdef CSpaceTimeAStar* _obj

    def __cinit__(self, _AbsGraph graph):
        self.graph = graph
        self._obj = new CSpaceTimeAStar(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj

    @_pathfinding
    def find_path(
        self,
        int start,
        int goal,
        int search_depth=100,
        ReservationTable reservation_table=None,
    ):
        cdef CReservationTable* crt
        if reservation_table is None:
            crt = NULL
        else:
            assert(reservation_table.graph == self.graph)
            crt = reservation_table._obj

        return self._obj.find_path(start, goal, search_depth, crt)


cdef class ReservationTable:
    cdef CReservationTable* _obj
    cdef public _AbsGraph graph

    def __cinit__(self, _AbsGraph graph):
        self.graph = graph
        self._obj = new CReservationTable(graph.size())

    def __dealloc__(self):
        del self._obj

    def __repr__(self):
        return f"ReservationTable(graph={self.graph})"

    def _to_node_id(self, node):
        if isinstance(self.graph, Graph):
            return node
        elif isinstance(self.graph, (Grid, Grid3D)):
            return self.graph.get_node_id(node)
        else:
            raise NotImplementedError()

    def _convert_path(self, path):
        return [self._to_node_id(node) for node in path]

    def is_reserved(self, int time, node):
        cdef int node_id = self._to_node_id(node)
        return self._obj.is_reserved(time, node_id)

    def add_path(
        self,
        path,
        int start_time=0,
        bool reserve_destination=False,
    ):
        cdef vector[int] node_ids = self._convert_path(path)
        self._obj.add_path(start_time, node_ids, reserve_destination, self.graph.edge_collision)

    def add_vertex_constraint(self, node, int time=0):
        cdef int node_id = self._to_node_id(node)
        self._obj.add_vertex_constraint(time, node_id)

    def add_edge_constraint(self, n1, n2, int time=0):
        cdef int n1_id, n2_id
        n1_id = self._to_node_id(n1)
        n2_id = self._to_node_id(n2)
        self._obj.add_edge_constraint(time, n1_id, n2_id)


def _mapf(func):

    def wrap(finder, starts, goals, **kwargs):
        assert len(starts) == len(goals)

        g = finder.graph

        if isinstance(g, Graph):
            for i in range(len(starts)):
                g.assert_in(starts[i])
                g.assert_in(goals[i])
            paths = func(finder, starts, goals, **kwargs)

        elif isinstance(g, (Grid, Grid3D, HexGrid)):
            starts = [g.get_node_id(x) for x in starts]
            goals = [g.get_node_id(x) for x in goals]
            paths = []
            for path in func(finder, starts, goals, **kwargs):
                paths.append([g.get_coordinates(node) for node in path])

        else:
            raise NotImplementedError

        return paths

    return wrap


cdef class _AbsMAPF():
    cdef CAbsMAPF* _baseobj
    cdef public _AbsGraph graph

    def __cinit__(self):
        pass

    def __repr__(self):
        return f"{self.__class__.__name__}(graph={self.graph})"

    cdef CReservationTable* _to_crt(self, ReservationTable reservation_table):
        cdef CReservationTable* crt
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
    cdef CHCAStar* _obj

    def __cinit__(self, _AbsGraph graph):
        self.graph = graph
        self._obj = new CHCAStar(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj

    @_mapf
    def mapf(
        self,
        vector[int] starts,
        vector[int] goals,
        int search_depth=100,
        bool despawn_at_destination=False,
        ReservationTable reservation_table=None,
    ):
        return self._obj.mapf(
            starts,
            goals,
            search_depth,
            despawn_at_destination,
            self._to_crt(reservation_table),
        )


cdef class WHCAStar(_AbsMAPF):
    cdef CWHCAStar* _obj

    def __cinit__(self, _AbsGraph graph):
        self.graph = graph
        self._obj = new CWHCAStar(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj

    @_mapf
    def mapf(
        self,
        vector[int] starts,
        vector[int] goals,
        int search_depth=100,
        int window_size=16,
        bool despawn_at_destination=False,
        ReservationTable reservation_table=None,
    ):
        return self._obj.mapf(
            starts,
            goals,
            search_depth,
            window_size,
            despawn_at_destination,
            self._to_crt(reservation_table),
        )


cdef class CBS(_AbsMAPF):
    cdef CCBS* _obj

    def __cinit__(self, _AbsGraph graph):
        self.graph = graph
        self._obj = new CCBS(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj

    @_mapf
    def mapf(
        self,
        vector[int] starts,
        vector[int] goals,
        int search_depth=100,
        double max_time=1,
        bool despawn_at_destination=False,
        ReservationTable reservation_table=None,
    ):
        return self._obj.mapf(
            starts,
            goals,
            search_depth,
            max_time,
            despawn_at_destination,
            self._to_crt(reservation_table),
        )
