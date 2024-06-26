# distutils: language = c++

from libcpp cimport bool
from w9_pathfinding.cdefs cimport (
    AbsGraph as CAbsGraph,
    AbsPathFinder as CAbsPathFinder,
    Grid as CGrid,
    Graph as CGraph,
    DFS as CDFS,
    BFS as CBFS,
    Dijkstra as CDijkstra,
    BiDijkstra as CBiDijkstra,
    AStar as CAstar,
)


cdef class _AbsGraph:
    cdef CAbsGraph* _baseobj

    def __cinit__(self):
        pass


cdef class Graph(_AbsGraph):
    cdef CGraph* _obj
    cdef readonly int num_vertices

    def __cinit__(self, unsigned int num_vertices):
        self._obj = new CGraph(num_vertices)
        self._baseobj = self._obj
        self.num_vertices = num_vertices

    def __dealloc__(self):
        del self._obj

    def __repr__(self):
        return f"Graph(num_vertices={self.num_vertices}, num_edges={self.num_edges})"

    @property
    def num_edges(self):
        return self._obj.num_edges()

    @property
    def edges(self):
        data = []
        for (start, end, cost) in self._obj.get_edges():
            data.append([int(start), int(end), cost])
        return data

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

    def get_neighbours(self, int node_id):
        return self._obj.get_neighbours(node_id)


cdef class Grid(_AbsGraph):
    cdef CGrid* _obj
    cdef readonly int width, height

    def __cinit__(
        self,
        obstacle_map=None,
        *,
        width=None,
        height=None,
        diagonal_movement=0,
        passable_left_right_border=False,
        passable_up_down_border=False,
    ):

        if obstacle_map is None:
            if not isinstance(width, int) or not isinstance(height, int):
                raise ValueError("Either obstacle_map or height and width must be provided.")
        else:
            height, width = len(obstacle_map), len(obstacle_map[0])

        if width <= 0:
            raise ValueError("Width must be greater than zero.")

        if height <= 0:
            raise ValueError("Height must be greater than zero.")

        self.width = width
        self.height = height

        self._obj = new CGrid(width, height)
        self._baseobj = self._obj

        if obstacle_map is not None:
            self.obstacle_map = obstacle_map

        if diagonal_movement:
            self.diagonal_movement = diagonal_movement

        if passable_left_right_border:
            self.passable_left_right_border = passable_left_right_border

        if passable_up_down_border:
            self.passable_up_down_border = passable_up_down_border

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

    def clear_obstacles(self):
        self._obj.clear_obstacles()

    def get_node_id(self, int x, int y):
        self.assert_in(x, y)
        return x + y * self.width

    def get_coordinates(self, int node_id):
        return node_id % self.width, node_id // self.width

    def get_neighbours(self, int x, int y):
        node_id = self.get_node_id(x, y)
        nodes = self._obj.get_neighbours(node_id)
        return [self.get_coordinates(x) for x, _ in nodes]

    @property
    def obstacle_map(self):
        flat_map = self._obj.get_obstacle_map()
        map = []
        for y in range(self.height):
            row = [int(flat_map[self.get_node_id(x, y)]) for x in range(self.width)]
            map.append(row)
        return map

    @obstacle_map.setter
    def obstacle_map(self, _map):
        height, width = len(_map), len(_map[0])
        if height != self.height or width != self.width:
            raise ValueError(f"obstacle_map.shape must be {self.width}x{self.height}")
 
        obstacle_map = []
        for row in _map:
            for x in row:
               obstacle_map.append(x != 0) 
 
        if len(obstacle_map) != self.width * self.height:
            raise ValueError(f"obstacle_map.shape must be {self.width}x{self.height}")
 
        self._obj.set_obstacle_map(obstacle_map)

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
    cdef CAstar* _obj

    def __cinit__(self, Grid grid):
        self.graph = grid
        if grid.diagonal_movement:
            heuristic = 1  # chebyshev
        else:
            heuristic = 0  # manhattan
        self._obj = new CAstar(grid._obj, heuristic)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj
