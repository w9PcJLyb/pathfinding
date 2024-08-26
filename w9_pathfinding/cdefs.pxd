from libcpp cimport bool, string
from libcpp.pair cimport pair
from libcpp.vector cimport vector


cdef extern from "src/core.cpp":
    pass


cdef extern from "src/include/core.h":

    cdef cppclass AbsGraph:
        AbsGraph() except +
        size_t size()
        double calculate_cost(vector[int])
        void reverse_inplace()
        vector[pair[int, double]] get_neighbors(int)
        vector[vector[int]] find_components()
        vector[vector[int]] find_scc()
        bool adjacent(int, int)
        void set_pause_action_cost(double)
        double get_pause_action_cost()
        void set_edge_collision(bool)
        bool edge_collision()

    cdef cppclass AbsGrid(AbsGraph):
        bool has_obstacle(int)
        void add_obstacle(int)
        void remove_obstacle(int)
        void clear_weights()
        void set_weights(vector[double]&)
        vector[double] get_weights()

    cdef cppclass AbsPathFinder:
        AbsPathFinder() except +
        vector[int] find_path(int, int)

    cdef cppclass AbsMAPF:
        AbsMAPF() except +
        vector[vector[int]] mapf(vector[int], vector[int])


cdef extern from "src/resumable_search.cpp":
    pass


cdef extern from "src/include/resumable_search.h":
    pass


cdef extern from "src/reservation_table.cpp":
    pass


cdef extern from "src/include/reservation_table.h":
    cdef cppclass ReservationTable:
        ReservationTable(int) except +
        bool is_reserved(int, int)
        void add_path(int, vector[int], bool, bool)
        void add_vertex_constraint(int, int)
        void add_edge_constraint(int, int, int)
        int last_time_reserved(int)


cdef extern from "src/graph.cpp":
    pass


cdef extern from "src/include/graph.h":

    cdef cppclass Graph(AbsGraph):
        Graph(int, bool) except +
        Graph(int, bool, vector[vector[double]]) except +
        void add_edges(vector[int], vector[int], vector[double])
        size_t num_edges()
        vector[vector[double]] get_edges()
        vector[vector[double]] get_coordinates()
        void set_coordinates(vector[vector[double]])
        bool has_coordinates()
        double estimate_distance(int v1, int v2)
        Graph* create_reversed_graph()


cdef extern from "src/grid.cpp":
    pass


cdef extern from "src/include/grid.h":

    cdef cppclass Grid(AbsGrid):
        bool passable_left_right_border, passable_up_down_border
        double diagonal_movement_cost_multiplier

        Grid(int, int) except +
        Grid(vector[vector[double]]) except +
        unsigned int get_diagonal_movement()
        void set_diagonal_movement(int)


cdef extern from "src/grid_3d.cpp":
    pass


cdef extern from "src/include/grid_3d.h":

    cdef cppclass Grid3D(AbsGrid):
        bool passable_borders

        Grid3D(int, int, int) except +
        Grid3D(vector[vector[vector[double]]]) except +


cdef extern from "src/hex_grid.cpp":
    pass


cdef extern from "src/include/hex_grid.h":

    cdef cppclass HexGrid(AbsGrid):
        bool passable_left_right_border, passable_up_down_border
        int layout

        HexGrid(int, int, int) except +
        HexGrid(int, vector[vector[double]]) except +


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


cdef extern from "src/bi_bfs.cpp":
    pass


cdef extern from "src/include/bi_bfs.h":

    cdef cppclass BiBFS(AbsPathFinder):
        BiBFS(AbsGraph*) except +
        vector[int] find_path(int, int)


cdef extern from "src/dijkstra.cpp":
    pass


cdef extern from "src/include/dijkstra.h":

    cdef cppclass Dijkstra(AbsPathFinder):
        Dijkstra(AbsGraph*) except +
        vector[int] find_path(int, int)


cdef extern from "src/bi_dijkstra.cpp":
    pass


cdef extern from "src/include/bi_dijkstra.h":

    cdef cppclass BiDijkstra(AbsPathFinder):
        BiDijkstra(AbsGraph*) except +
        vector[int] find_path(int, int)


cdef extern from "src/a_star.cpp":
    pass


cdef extern from "src/include/a_star.h":

    cdef cppclass AStar(AbsPathFinder):
        AStar(AbsGraph*) except +
        vector[int] find_path(int, int)


cdef extern from "src/bi_a_star.cpp":
    pass


cdef extern from "src/include/bi_a_star.h":

    cdef cppclass BiAStar(AbsPathFinder):
        BiAStar(AbsGraph*) except +
        vector[int] find_path(int, int)


cdef extern from "src/gbs.cpp":
    pass


cdef extern from "src/include/gbs.h":

    cdef cppclass GBS(AbsPathFinder):
        GBS(AbsGraph*) except +
        vector[int] find_path(int, int)



cdef extern from "src/ida_star.cpp":
    pass


cdef extern from "src/include/ida_star.h":

    cdef cppclass IDAStar(AbsPathFinder):
        IDAStar(AbsGraph*) except +
        vector[int] find_path(int, int, double)


cdef extern from "src/resumable_search.cpp":
    pass


cdef extern from "src/include/resumable_search.h":

    cdef cppclass ResumableBFS:
        ResumableBFS(AbsGraph*, int) except +
        double distance(int)
        vector[int] find_path(int)
        int start_node()
        void set_start_node(int)

    cdef cppclass ResumableDijkstra:
        ResumableDijkstra(AbsGraph*, int) except +
        double distance(int)
        vector[int] find_path(int)
        int start_node()
        void set_start_node(int)


cdef extern from "src/space_time_a_star.cpp":
    pass


cdef extern from "src/include/space_time_a_star.h":

    cdef cppclass SpaceTimeAStar(AbsPathFinder):
        SpaceTimeAStar(AbsGraph*) except +
        vector[int] find_path(int, int, int, ReservationTable*)


cdef extern from "src/hc_a_star.cpp":
    pass


cdef extern from "src/include/hc_a_star.h":

    cdef cppclass HCAStar(AbsMAPF):
        HCAStar(AbsGraph*) except +
        vector[vector[int]] mapf(vector[int], vector[int], int, bool, ReservationTable*)


cdef extern from "src/whc_a_star.cpp":
    pass


cdef extern from "src/include/whc_a_star.h":

    cdef cppclass WHCAStar(AbsMAPF):
        WHCAStar(AbsGraph*) except +
        vector[vector[int]] mapf(vector[int], vector[int], int, int, bool, ReservationTable*)


cdef extern from "src/cbs.cpp":
    pass


cdef extern from "src/include/cbs.h":

    cdef cppclass CBS(AbsMAPF):
        CBS(AbsGraph*) except +
        vector[vector[int]] mapf(vector[int], vector[int], int, double, bool, ReservationTable*) except +


cdef extern from "src/limited_search.cpp":
    pass


cdef extern from "src/include/limited_search.h":

    cdef cppclass LimitedSearch(AbsMAPF):
        LimitedSearch(AbsGraph*) except +
        vector[int] find_path(int, int, int)
        vector[int] find_path_to_moving_goal(int, vector[int], int)
