import unittest
from w9_pathfinding import (
    Grid3D,
    DFS,
    BFS,
    BiBFS,
    Dijkstra,
    BiDijkstra,
    AStar,
    BiAStar,
    SpaceTimeAStar,
)

SHORTEST_PATH_ALGORITHMS = [Dijkstra, BiDijkstra, AStar, BiAStar, SpaceTimeAStar]
ALL_ALGORITHMS = [DFS, BFS, BiBFS] + SHORTEST_PATH_ALGORITHMS


class TestSimpleGrid3D(unittest.TestCase):
    """
    pytest tests/test_grid_3d_pathfinding.py::TestSimpleGrid3D
    """

    def test_0d(self):
        """
        + - +
        | s |
        + - +
        """
        weights = [[[1]]]
        start, end = (0, 0, 0), (0, 0, 0)
        answer = [(0, 0, 0)]

        grid = Grid3D(weights)

        for a in ALL_ALGORITHMS:
            with self.subTest(a.__name__):
                path = a(grid).find_path(start, end)
                self.assertListEqual(path, answer)

    def test_out_of_the_grid(self):
        """
        + - +
        | s |
        + - +
        """
        weights = [[[1]]]
        start, end = (0, 0, 0), (10, 10, 10)

        grid = Grid3D(weights)

        for a in ALL_ALGORITHMS:
            with self.subTest(a.__name__):
                with self.assertRaises(ValueError):
                    a(grid).find_path(start, end)

    def test_1d(self):
        """
        + -  -  -  -  - +
        | s  *  *  *  e |
        + -  -  -  -  - +
        """
        weights = [[[1, 1, 1, 1, 1]]]
        start, end = (0, 0, 0), (4, 0, 0)
        answer = [(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0), (4, 0, 0)]

        grid = Grid3D(weights)

        for a in ALL_ALGORITHMS:
            with self.subTest(a.__name__):
                path = a(grid).find_path(start, end)
                self.assertListEqual(path, answer)

    def test_passable_borders(self):
        """
        + -  -  -  -  - +
        | s        #  e |
        + -  -  -  -  - +
        """
        weights = [[[1, 1, 1, -1, 1]]]
        start, end = (0, 0, 0), (4, 0, 0)
        answer = [(0, 0, 0), (4, 0, 0)]

        grid = Grid3D(weights, passable_borders=True)

        for a in ALL_ALGORITHMS:
            with self.subTest(a.__name__):
                path = a(grid).find_path(start, end)
                self.assertListEqual(path, answer)

    def test_2d(self):
        """
        + -  -  -  - +
        | s  #  #  e |
        | *  *  *  * |
        + -  -  -  - +
        """

        weights = [
            [
                [1, -1, -1, 1],
                [1, 1, 1, 1],
            ]
        ]
        start, end = (0, 0, 0), (3, 0, 0)
        answer = [(0, 0, 0), (0, 1, 0), (1, 1, 0), (2, 1, 0), (3, 1, 0), (3, 0, 0)]

        grid = Grid3D(weights)

        for a in ALL_ALGORITHMS:
            with self.subTest(a.__name__):
                path = a(grid).find_path(start, end)
                self.assertListEqual(path, answer)

    def test_3d(self):
        """
        z = 0
        + -  -  -  +
        | s  #  e  |
        | *  #  *  |
        | *  #  *  |
        + -  -  -  +

        z = 1
        + -  -  -  +
        |    #     |
        | #  #  #  |
        | *  *  *  |
        + -  -  -  +
        """

        weights = [
            [[1, -1, 1], [1, -1, 1], [1, -1, 1]],
            [[1, -1, 1], [-1, -1, -1], [1, 1, 1]],
        ]
        start, end = (0, 0, 0), (2, 0, 0)
        answer = [
            (0, 0, 0),
            (0, 1, 0),
            (0, 2, 0),
            (0, 2, 1),
            (1, 2, 1),
            (2, 2, 1),
            (2, 2, 0),
            (2, 1, 0),
            (2, 0, 0),
        ]

        grid = Grid3D(weights)

        for a in ALL_ALGORITHMS:
            with self.subTest(a.__name__):
                path = a(grid).find_path(start, end)
                self.assertListEqual(path, answer)
