# distutils: language = c++

from libcpp cimport bool
from libcpp.vector cimport vector

from w9_pathfinding cimport cdefs
from w9_pathfinding.envs cimport _AbsGraph, ReservationTable


def _mapf(func):

    def wrap(finder, starts, goals, **kwargs):
        assert len(starts) == len(goals)

        map = finder.graph._node_mapper
        starts = map.to_ids(starts)
        goals = map.to_ids(goals)
        paths = [map.from_ids(p) for p in func(finder, starts, goals, **kwargs)]
        return paths

    return wrap


cdef class _AbsMAPF():
    cdef cdefs.AbsMAPF* _baseobj
    cdef public _AbsGraph graph

    def __cinit__(self):
        pass

    def __repr__(self):
        return f"{self.__class__.__name__}(graph={self.graph})"

    cdef cdefs.ReservationTable* _to_crt(self, ReservationTable reservation_table):
        cdef cdefs.ReservationTable* crt
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
    cdef cdefs.HCAStar* _obj

    def __cinit__(self, _AbsGraph graph):
        self.graph = graph
        self._obj = new cdefs.HCAStar(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj

    @_mapf
    def mapf(
        self,
        vector[int] starts,
        vector[int] goals,
        int max_length=100,
        ReservationTable reservation_table=None,
    ):
        return self._obj.mapf(
            starts,
            goals,
            max_length,
            self._to_crt(reservation_table),
        )


cdef class WHCAStar(_AbsMAPF):
    cdef cdefs.WHCAStar* _obj

    def __cinit__(self, _AbsGraph graph):
        self.graph = graph
        self._obj = new cdefs.WHCAStar(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj

    @_mapf
    def mapf(
        self,
        vector[int] starts,
        vector[int] goals,
        int max_length=100,
        int window_size=16,
        ReservationTable reservation_table=None,
    ):
        return self._obj.mapf(
            starts,
            goals,
            max_length,
            window_size,
            self._to_crt(reservation_table),
        )


cdef class CBS(_AbsMAPF):
    cdef cdefs.CBS* _obj

    def __cinit__(self, _AbsGraph graph):
        self.graph = graph
        self._obj = new cdefs.CBS(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj

    @property
    def num_generated_nodes(self):
        return self._obj.num_generated_nodes

    @property
    def num_closed_nodes(self):
        return self._obj.num_closed_nodes

    @_mapf
    def mapf(
        self,
        vector[int] starts,
        vector[int] goals,
        int max_length=100,
        double max_time=1,
        bool disjoint_splitting=True,
        ReservationTable reservation_table=None,
    ):
        return self._obj.mapf(
            starts,
            goals,
            max_length,
            max_time,
            disjoint_splitting,
            self._to_crt(reservation_table),
        )


cdef class ICTS(_AbsMAPF):
    cdef cdefs.ICTS* _obj

    def __cinit__(self, _AbsGraph graph):
        self.graph = graph
        self._obj = new cdefs.ICTS(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj

    @property
    def num_generated_nodes(self):
        return self._obj.num_generated_nodes

    @property
    def num_closed_nodes(self):
        return self._obj.num_closed_nodes

    @_mapf
    def mapf(
        self,
        vector[int] starts,
        vector[int] goals,
        int max_length=100,
        double max_time=1,
        bool ict_pruning=True,
        ReservationTable reservation_table=None,
    ):
        return self._obj.mapf(
            starts,
            goals,
            max_length,
            max_time,
            ict_pruning,
            self._to_crt(reservation_table),
        )


cdef class MultiAgentAStar(_AbsMAPF):
    cdef cdefs.MultiAgentAStar* _obj

    def __cinit__(self, _AbsGraph graph):
        self.graph = graph
        self._obj = new cdefs.MultiAgentAStar(graph._baseobj)
        self._baseobj = self._obj

    def __dealloc__(self):
        del self._obj

    @_mapf
    def mapf(
        self,
        vector[int] starts,
        vector[int] goals,
        int max_length=100,
        double max_time=1,
        bool operator_decomposition=True,
        ReservationTable reservation_table=None,
    ):
        return self._obj.mapf(
            starts,
            goals,
            max_length,
            max_time,
            operator_decomposition,
            self._to_crt(reservation_table),
        )
 