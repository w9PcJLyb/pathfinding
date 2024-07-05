# distutils: language = c++

from libcpp cimport bool
from libcpp.vector cimport vector
from w9_pathfinding.cdefs cimport (
    AbsGraph as CAbsGraph,
    AbsPathFinder as CAbsPathFinder,
    Graph as CGraph,
    Grid as CGrid,
    DFS as CDFS,
    BFS as CBFS,
    BiBFS as CBiBFS,
    Dijkstra as CDijkstra,
    BiDijkstra as CBiDijkstra,
    AStar as CAStar,
    BiAStar as CBiAStar,
)


cdef class _AbsGraph:
    cdef CAbsGraph* _baseobj

    def __cinit__(self):
        pass

    def calculate_cost(self, vector[int] path):
        return self._baseobj.calculate_cost(path)

    def get_neighbours(self, int node_id):
        # return [[neighbour_id, cost], ...]
        return self._baseobj.get_neighbours(node_id)


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
        edges=None
    ):
        self._obj = new CGraph(num_vertices, directed)
        self._baseobj = self._obj
        self.num_vertices = num_vertices
        self.directed = directed
        if coordinates:
            self.set_coordinates(coordinates)
        if edges:
            self.add_edges(edges)

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
        for start, end, cost in edges:
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


cdef class Grid(_AbsGraph):
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
            self._obj = new CGrid(width, height, sum(weights, []))

        self._baseobj = self._obj

        if diagonal_movement:
            self.diagonal_movement = diagonal_movement

        if passable_left_right_border:
            self.passable_left_right_border = passable_left_right_border

        if passable_up_down_border:
            self.passable_up_down_border = passable_up_down_border

        if diagonal_movement_cost_multiplier != 1:
            self.diagonal_movement_cost_multiplier = diagonal_movement_cost_multiplier

    def __dealloc__(self):
        del self._obj

    def __repr__(self):
        return f"Grid({self.width}x{self.height})"

    @property
    def diagonal_movement(self):
        return self._obj.get_diagonal_movement()

    @diagonal_movement.setter
    def diagonal_movement(self, unsigned int _x):
        self._obj.set_diagonal_movement(_x)

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

    def assert_in(self, int x, int y):
        if not 0 <= x < self.width or not 0 <= y < self.height:
            raise ValueError(f"Point({x}, {y}) is out of the {self}")

    def has_obstacle(self, int x, int y):
        self.assert_in(x, y)
        return self._obj.has_obstacle(x + y * self.width)

    def add_obstacle(self, int x, int y):
        self.assert_in(x, y)
        self._obj.add_obstacle(x + y * self.width)

    def remove_obstacle(self, int x, int y):
        self.assert_in(x, y)
        self._obj.remove_obstacle(x + y * self.width)

    def get_node_id(self, int x, int y):
        self.assert_in(x, y)
        return x + y * self.width

    def get_coordinates(self, int node_id):
        return node_id % self.width, node_id // self.width

    def get_neighbours(self, int x, int y):
        node_id = self.get_node_id(x, y)
        neighbours = []
        for n, cost in self._obj.get_neighbours(node_id):
            neighbours.append((self.get_coordinates(n), cost))
        return neighbours

    @property
    def obstacle_map(self):
        weights = self._obj.get_weights()
        map = []
        for y in range(self.height):
            row = [int(weights[self.get_node_id(x, y)] == -1) for x in range(self.width)]
            map.append(row)
        return map

    def _check_weights(self, weights):
        height, width = len(weights), len(weights[0])
        if height != self.height or width != self.width:
            raise ValueError(f"weights.shape must be {self.width}x{self.height}")
        for row in weights:
            for w in row:
                if w < 0 and w != -1:
                    raise ValueError("Weight must be positive or equal to -1")

    @property
    def weights(self):
        weights = self._obj.get_weights()
        matrix = []
        for y in range(self.height):
            row = [weights[self.get_node_id(x, y)] for x in range(self.width)]
            matrix.append(row)
        return matrix

    @weights.setter
    def weights(self, matrix):
        self._check_weights(matrix)
        self._obj.set_weights(sum(matrix, []))

    def calculate_cost(self, path):
        cdef vector[int] nodes
        nodes = [self.get_node_id(*x) for x in path]
        return self._obj.calculate_cost(nodes)

    def find_components(self):
        return [
            [self.get_coordinates(node_id) for node_id in component]
            for component in self._obj.find_components()
        ]

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
               if self.has_obstacle(*k):
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


cdef class _AbsPathFinder():
    cdef CAbsPathFinder* _baseobj
    cdef public _AbsGraph graph

    def __cinit__(self, _AbsGraph graph):
        self.graph = graph

    def __repr__(self):
        return f"{self.__class__.__name__}(graph={self.graph})"

    def find_path(self, start, end):
        g = self.graph
        
        if type(g) is Graph:
            g.assert_in(start)
            g.assert_in(end)
            path = self._baseobj.find_path(start, end)

        elif type(g) is Grid:
            start = g.get_node_id(*start)
            end = g.get_node_id(*end)
            path = self._baseobj.find_path(start, end)
            path = [g.get_coordinates(node) for node in path]
        
        else:
            raise NotImplementedError
        
        return path


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
                "You can add coordinates using graph.add_coordinates(), "
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
                "You can add coordinates using graph.add_coordinates(), "
                "or choose some non-heuristic algorithm."
            )
        self.graph = graph
        self._obj = new CBiAStar(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj
