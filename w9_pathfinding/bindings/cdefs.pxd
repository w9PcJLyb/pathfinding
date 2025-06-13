from libcpp cimport bool, string
from libcpp.pair cimport pair
from libcpp.vector cimport vector


cdef extern from "core.h":

    cdef cppclass AbsGraph:
        AbsGraph() except +
        size_t size()
        double calculate_cost(vector[int])
        bool is_valid_path(vector[int])
        void reverse_inplace()
        vector[pair[int, double]] get_neighbors(int, bool, bool)
        vector[vector[int]] find_components()
        vector[vector[int]] find_scc()
        bool adjacent(int, int)
        void set_edge_collision(bool)
        bool edge_collision()

    cdef cppclass AbsGrid(AbsGraph):
        bool has_obstacle(int)
        void add_obstacle(int)
        void remove_obstacle(int)
        void clear_weights()
        void set_weights(vector[double]&) except +
        void update_weight(int, double) except +
        double get_weight(int) except +
        vector[double] get_weights()
        void set_pause_weight(double w) except +
        void set_pause_weights(vector[double]&) except +
        double get_pause_weight(int) except +
        vector[double] get_pause_weights()

    cdef cppclass AbsPathFinder:
        AbsPathFinder() except +
        vector[int] find_path(int, int)

    cdef cppclass AbsMAPF:
        AbsMAPF() except +
        vector[vector[int]] mapf(vector[int], vector[int])


cdef extern from "reservation_table.h":
    cdef cppclass ReservationTable:
        ReservationTable(int)
        ReservationTable(const ReservationTable&)
        bool is_reserved(int, int)
        bool is_reserved_edge(int, int, int)
        void add_path(int, vector[int], bool, bool)
        void add_vertex_constraint(int, int)
        void add_edge_constraint(int, int, int)
        void add_semi_static_constraint(int, int)
        int last_time_reserved(int)


cdef extern from "graph.h":

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


cdef extern from "grid.h":

    cdef cppclass Grid(AbsGrid):
        bool passable_left_right_border, passable_up_down_border
        double diagonal_movement_cost_multiplier

        Grid(int, int) except +
        Grid(int, int, vector[double]) except +
        unsigned int get_diagonal_movement()
        void set_diagonal_movement(int)


cdef extern from "grid_3d.h":

    cdef cppclass Grid3D(AbsGrid):
        bool passable_borders

        Grid3D(int, int, int) except +
        Grid3D(int, int, int, vector[double]) except +


cdef extern from "hex_grid.h":

    cdef cppclass HexGrid(AbsGrid):
        bool passable_left_right_border, passable_up_down_border
        int layout

        HexGrid(int, int, int) except +
        HexGrid(int, int, int, vector[double]) except +


cdef extern from "dfs.h":

    cdef cppclass DFS(AbsPathFinder):
        DFS(AbsGraph*) except + 
        vector[int] find_path(int, int)


cdef extern from "bfs.h":

    cdef cppclass BFS(AbsPathFinder):
        BFS(AbsGraph*) except +
        vector[int] find_path(int, int)


cdef extern from "bi_bfs.h":

    cdef cppclass BiBFS(AbsPathFinder):
        BiBFS(AbsGraph*) except +
        vector[int] find_path(int, int)


cdef extern from "dijkstra.h":

    cdef cppclass Dijkstra(AbsPathFinder):
        Dijkstra(AbsGraph*) except +
        vector[int] find_path(int, int)


cdef extern from "bi_dijkstra.h":

    cdef cppclass BiDijkstra(AbsPathFinder):
        BiDijkstra(AbsGraph*) except +
        vector[int] find_path(int, int)


cdef extern from "a_star.h":

    cdef cppclass AStar(AbsPathFinder):
        AStar(AbsGraph*) except +
        vector[int] find_path(int, int)


cdef extern from "bi_a_star.h":

    cdef cppclass BiAStar(AbsPathFinder):
        BiAStar(AbsGraph*) except +
        vector[int] find_path(int, int)


cdef extern from "gbs.h":

    cdef cppclass GBS(AbsPathFinder):
        GBS(AbsGraph*) except +
        vector[int] find_path(int, int)


cdef extern from "ida_star.h":

    cdef cppclass IDAStar(AbsPathFinder):
        IDAStar(AbsGraph*) except +
        vector[int] find_path(int, int, double)


cdef extern from "resumable_search.h":

    cdef cppclass ResumableSearch:
        double distance(int)
        vector[int] find_path(int)
        int start_node()
        void set_start_node(int)

    cdef cppclass ResumableBFS(ResumableSearch):
        ResumableBFS(AbsGraph*, int) except +

    cdef cppclass ResumableDijkstra(ResumableSearch):
        ResumableDijkstra(AbsGraph*, int) except +


cdef extern from "space_time_a_star.h":

    cdef cppclass SpaceTimeAStar:
        SpaceTimeAStar(AbsGraph*) except +
        vector[int] find_path_with_depth_limit(int, int, int, ReservationTable*, ResumableSearch*, int)
        vector[int] find_path_with_exact_length(int, int, int, ReservationTable*)
        vector[int] find_path_with_length_limit(int, int, int, ReservationTable*, ResumableSearch*, int)


cdef extern from "hc_a_star.h":

    cdef cppclass HCAStar(AbsMAPF):
        HCAStar(AbsGraph*) except +
        vector[vector[int]] mapf(vector[int], vector[int], int, ReservationTable*)


cdef extern from "whc_a_star.h":

    cdef cppclass WHCAStar(AbsMAPF):
        WHCAStar(AbsGraph*) except +
        vector[vector[int]] mapf(vector[int], vector[int], int, int, ReservationTable*)


cdef extern from "cbs.h":

    cdef cppclass CBS(AbsMAPF):
        int num_generated_nodes, num_closed_nodes
        CBS(AbsGraph*) except +
        vector[vector[int]] mapf(vector[int], vector[int], int, double, bool, ReservationTable*) except +


cdef extern from "icts.h" namespace "icts":

    cdef cppclass ICTS(AbsMAPF):
        int num_generated_nodes, num_closed_nodes
        ICTS(AbsGraph*) except +
        vector[vector[int]] mapf(vector[int], vector[int], int, double, bool, ReservationTable*) except +


cdef extern from "multi_agent_a_star.h" namespace "maas":

    cdef cppclass MultiAgentAStar(AbsMAPF):
        MultiAgentAStar(AbsGraph*) except +
        vector[vector[int]] mapf(vector[int], vector[int], int, double, bool, ReservationTable*) except +
