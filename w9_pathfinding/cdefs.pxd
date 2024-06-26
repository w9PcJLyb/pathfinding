from libcpp cimport bool
from libcpp.pair cimport pair
from libcpp.vector cimport vector


cdef extern from "src/core.cpp":
    pass


cdef extern from "src/include/core.h":

    cdef cppclass AbsGraph:
        AbsGraph() except +
        size_t size()

    cdef cppclass AbsPathFinder:
        AbsPathFinder() except +
        vector[int] find_path(int, int)


cdef extern from "src/graph.cpp":
    pass


cdef extern from "src/include/graph.h":

    cdef cppclass Graph(AbsGraph):
        Graph(int) except +
        void add_edges(vector[int], vector[int], vector[double])
        size_t size()
        size_t num_edges()
        vector[vector[double]] get_edges()
        vector[pair[int, double]] get_neighbours(int)


cdef extern from "src/grid.cpp":
    pass


cdef extern from "src/include/grid.h":

    cdef cppclass Grid(AbsGraph):
        int width, height
        bool passable_left_right_border, passable_up_down_border

        Grid(int, int) except +
        Grid(int, int, vector[int]) except +
        unsigned int get_diagonal_movement()
        void set_diagonal_movement(int)
        bool has_obstacle(int)
        void add_obstacle(int)
        void remove_obstacle(int)
        void clear_obstacles()
        void set_obstacle_map(vector[int]&)
        vector[int] get_obstacle_map()
        vector[pair[int, double]] get_neighbours(int)


cdef extern from "src/dfs.cpp":
    pass


cdef extern from "src/include/dfs.h":

    cdef cppclass DFS(AbsPathFinder):
        DFS(AbsGraph*) except + 
        vector[int] find_path(int, int)


cdef extern from "src/bfs.cpp":
    pass


cdef extern from "src/include/bfs.h":

    cdef cppclass BFS(AbsPathFinder):
        BFS(AbsGraph*) except +
        vector[int] find_path(int, int)


cdef extern from "src/dijkstra.cpp":
    pass


cdef extern from "src/include/dijkstra.h":

    cdef cppclass Dijkstra(AbsPathFinder):
        Dijkstra(AbsGraph*) except +
        vector[int] find_path(int, int)


cdef extern from "src/bidijkstra.cpp":
    pass


cdef extern from "src/include/bidijkstra.h":

    cdef cppclass BiDijkstra(AbsPathFinder):
        BiDijkstra(AbsGraph*) except +
        vector[int] find_path(int, int)


cdef extern from "src/a_star.cpp":
    pass


cdef extern from "src/include/a_star.h":

    cdef cppclass AStar(AbsPathFinder):
        AStar(Grid*, int) except +
        vector[int] find_path(int, int)

