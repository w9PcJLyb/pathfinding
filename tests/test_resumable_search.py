import unittest
from w9_pathfinding.graph import Graph, Grid
from w9_pathfinding.pf import ResumableBFS, ResumableDijkstra


class TestRS(unittest.TestCase):
    """
    pytest tests/test_resumable_search.py::TestRS
    """

    def test_with_graph(self):
        graph = Graph(4, edges=[(0, 1), (1, 2), (2, 1)])
        inf = float("inf")

        for a in (ResumableBFS, ResumableDijkstra):
            with self.subTest(a.__name__):
                rs = a(graph, 0)
                for n, ans in [(0, 0), (1, 1), (2, 2), (3, inf)]:
                    self.assertEqual(rs.distance(n), ans)

                rs.start_node = 2
                for n, ans in [(0, inf), (1, 1), (2, 0), (3, inf)]:
                    self.assertEqual(rs.distance(n), ans)

    def test_with_grid(self):
        grid = Grid([[1, 1, 1], [-1, -1, 1], [1, 1, 1]])
        inf = float("inf")

        for a in (ResumableBFS, ResumableDijkstra):
            with self.subTest(a.__name__):
                rs = a(grid, (0, 0))
                for n, ans in [((0, 0), 0), ((0, 1), inf), ((0, 2), 6)]:
                    self.assertEqual(rs.distance(n), ans)

                rs.start_node = (2, 1)
                for n, ans in [((0, 0), 3), ((0, 1), inf), ((0, 2), 3)]:
                    self.assertEqual(rs.distance(n), ans)
