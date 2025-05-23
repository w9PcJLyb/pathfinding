import unittest
from w9_pathfinding.envs import HexGrid
from tests.pf.test_grid_pf import ALL_ALGORITHMS


class TestHexGrid(unittest.TestCase):
    """
    pytest tests/pf/test_hex_grid_pf.py::TestHexGrid
    """

    def test_0d(self):
        weights = [[1]]
        start, end = (0, 0), (0, 0)
        answer = [(0, 0)]

        grid = HexGrid(weights)

        for a in ALL_ALGORITHMS:
            with self.subTest(a.__name__):
                path = a(grid).find_path(start, end)
                self.assertListEqual(path, answer)

    def test_out_of_the_grid(self):
        weights = [[1]]
        start, end = (0, 0), (10, 10)

        grid = HexGrid(weights)

        for a in ALL_ALGORITHMS:
            with self.subTest(a.__name__):
                with self.assertRaises(ValueError):
                    a(grid).find_path(start, end)

    def test_1d(self):
        weights = [[1, 1, 1, 1, 1]]
        start, end = (0, 0), (4, 0)
        answer = [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)]

        grid = HexGrid(weights)

        for a in ALL_ALGORITHMS:
            with self.subTest(a.__name__):
                path = a(grid).find_path(start, end)
                self.assertListEqual(path, answer)

    def test_1d_no_path(self):
        weights = [[1, 1, 1, -1, 1]]
        start, end = (0, 0), (4, 0)
        answer = []

        grid = HexGrid(weights)

        for a in ALL_ALGORITHMS:
            with self.subTest(a.__name__):
                path = a(grid).find_path(start, end)
                self.assertListEqual(path, answer)

    def test_3x3(self):
        """
        _ # #
         _ _ #
        # # _
        """
        weights = [[1, -1, -1], [1, 1, -1], [-1, -1, 1]]
        start, end = (0, 0), (2, 2)
        answer = [(0, 0), (0, 1), (1, 1), (2, 2)]

        grid = HexGrid(weights)

        for a in ALL_ALGORITHMS:
            with self.subTest(a.__name__):
                path = a(grid).find_path(start, end)
                self.assertListEqual(path, answer)
