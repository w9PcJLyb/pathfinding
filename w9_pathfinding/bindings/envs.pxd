from libcpp cimport bool

from w9_pathfinding.bindings cimport cdefs


cdef class _NodeMapper:
    pass


cdef class _Env:
    cdef cdefs.Env* _baseobj
    cdef readonly _NodeMapper _node_mapper
    cdef object __weakref__


cdef class Graph(_Env):
    cdef cdefs.Graph* _obj


cdef class _GridEnv(_Env):
    cdef cdefs.GridEnv* _basegridobj


cdef class Grid(_GridEnv):
    cdef cdefs.Grid* _obj


cdef class Grid3D(_GridEnv):
    cdef cdefs.Grid3D* _obj


cdef class HexGrid(_GridEnv):
    cdef cdefs.HexGrid* _obj
