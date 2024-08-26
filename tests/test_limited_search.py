import unittest
from w9_pathfinding import Graph, Grid, LimitedSearch


class Test(unittest.TestCase):
    """
    pytest tests/test_limited_search.py::Test
    """

    def test_simple_grid(self):
        grid = Grid([[1]])
        path = LimitedSearch(grid).find_path((0, 0), (0, 0), max_steps=1)
        self.assertEqual(path, [(0, 0)])

    def test_with_graph(self):
        graph = Graph(
            5, edges=[(0, 1, 5), (0, 2, 2), (2, 1, 2), (0, 3, 1), (3, 4, 1), (4, 1, 1)]
        )
        for max_steps in range(5, 0, -1):
            path = LimitedSearch(graph).find_path(0, 1, max_steps=max_steps)
            if max_steps >= 3:
                self.assertEqual(path, [0, 3, 4, 1])
            elif max_steps >= 2:
                self.assertEqual(path, [0, 2, 1])
            elif max_steps >= 1:
                self.assertEqual(path, [0, 1])
            else:
                self.assertEqual(path, [])

    def test_with_grid(self):
        grid = Grid(
            [
                [1, 1, 1, 1],
                [9, 2, 1, 1],
                [8, 3, 1, 1],
                [1, 1, 1, 1],
            ]
        )

        for max_steps in range(8, 0, -1):
            path = LimitedSearch(grid).find_path((0, 0), (0, 3), max_steps=max_steps)
            if max_steps >= 7:
                self.assertEqual(
                    path,
                    [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2), (2, 3), (1, 3), (0, 3)],
                )
            elif max_steps >= 5:
                self.assertEqual(path, [(0, 0), (1, 0), (1, 1), (1, 2), (1, 3), (0, 3)])
            elif max_steps >= 3:
                self.assertEqual(path, [(0, 0), (0, 1), (0, 2), (0, 3)])
            else:
                self.assertEqual(path, [])
