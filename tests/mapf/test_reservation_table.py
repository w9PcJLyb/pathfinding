import unittest
from copy import copy
from w9_pathfinding.envs import Grid, ReservationTable


class TestReservationTable(unittest.TestCase):
    """
    pytest tests/test_reservation_table.py::TestReservationTable
    """

    def test_with_grid(self):
        grid = Grid(width=3, height=3)
        rt = ReservationTable(grid)

        self.assertFalse(rt.is_reserved(2, (0, 1)))
        rt.add_vertex_constraint(2, (0, 1))

        self.assertTrue(rt.is_reserved(2, (0, 1)))

        self.assertFalse(rt.is_edge_reserved(2, (0, 1), (1, 1)))
        rt.add_edge_constraint(2, (0, 1), (1, 1))
        self.assertTrue(rt.is_edge_reserved(2, (0, 1), (1, 1)))

    def test_reserve_permanently(self):
        grid = Grid(width=3, height=3)
        rt = ReservationTable(grid)

        rt.add_vertex_constraint(time=2, node=(0, 1), permanent=True)

        for time, reserved in (
            (0, False),
            (1, False),
            (2, True),
            (3, True),
            (100, True),
        ):
            self.assertEqual(rt.is_reserved(time, (0, 1)), reserved)

    def test_copy(self):
        grid = Grid(width=3, height=3)
        rt = ReservationTable(grid)

        rt.add_vertex_constraint(time=2, node=(0, 1))

        rt_copy = copy(rt)
        rt_copy.add_vertex_constraint(3, node=(0, 1))

        self.assertEqual(id(rt.graph), id(rt_copy.graph))

        self.assertTrue(rt.is_reserved(2, (0, 1)))
        self.assertFalse(rt.is_reserved(3, (0, 1)))

        self.assertTrue(rt_copy.is_reserved(2, (0, 1)))
        self.assertTrue(rt_copy.is_reserved(3, (0, 1)))
