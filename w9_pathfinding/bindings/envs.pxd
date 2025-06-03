from libcpp cimport bool

from w9_pathfinding.bindings cimport cdefs


cdef class _NodeMapper:
    pass


cdef class _AbsGraph:
    cdef cdefs.AbsGraph* _baseobj
    cdef readonly _NodeMapper _node_mapper
    cdef object __weakref__


cdef class Graph(_AbsGraph):
    cdef cdefs.Graph* _obj
    cdef readonly int num_vertices
    cdef readonly bool directed


cdef class _AbsGrid(_AbsGraph):
    cdef cdefs.AbsGrid* _basegridobj


cdef class Grid(_AbsGrid):
    cdef cdefs.Grid* _obj
    cdef readonly int width, height


cdef class Grid3D(_AbsGrid):
    cdef cdefs.Grid3D* _obj
    cdef readonly int width, height, depth


cdef class HexGrid(_AbsGrid):
    cdef cdefs.HexGrid* _obj
    cdef readonly int width, height
