import unittest
from w9_pathfinding import (
    Grid,
    Graph,
    DiagonalMovement,
    SpaceTimeAStar,
    ReservationTable,
)


class TestSpaceTimeAStar(unittest.TestCase):
    """
    pytest tests/test_space_time_a_star.py::TestSpaceTimeAStar
    """

    def test_with_directed_graph(self):
        graph = Graph(3, edges=[[0, 1], [1, 2]])
        rt = ReservationTable(graph)
        rt.add_path([1, 1, 1])

        a = SpaceTimeAStar(graph)
        path = a.find_path(0, 2, reservation_table=rt)
        self.assertListEqual(path, [0, 0, 0, 1, 2])

    def test_with_directed_weighted_graph(self):
        graph = Graph(5, edges=[[0, 1, 100], [0, 4, 1], [1, 2, 100], [2, 3, 100]])

        a = SpaceTimeAStar(graph)
        path = a.find_path(0, 3, search_depth=4)
        self.assertListEqual(path, [0, 1, 2, 3])

    def test_with_grid(self):
        weights = [[1, 1, 1], [1, 1, 1]]
        grid = Grid(weights=weights)
        rt = ReservationTable(grid)
        rt.add_path([(1, 0), (1, 0), (1, 0), (1, 0)])

        a = SpaceTimeAStar(grid)
        path = a.find_path((0, 0), (2, 0), reservation_table=rt)
        self.assertListEqual(path, [(0, 0), (0, 1), (1, 1), (2, 1), (2, 0)])

    def test_3x4_with_two_agents(self):
        """
        + -  -  -  - +
        | s        # |
        |    #     # |
        |       e2 e1|
        + -  -  -  - +
        """
        weights = [[1, 2, 0.9, -1], [1, -1, 0.9, -1], [1, 1, 1, 1]]
        grid = Grid(weights=weights, diagonal_movement=DiagonalMovement.never)

        a = SpaceTimeAStar(grid)

        path1 = a.find_path((0, 0), (3, 2))
        self.assertListEqual(path1, [(0, 0), (0, 1), (0, 2), (1, 2), (2, 2), (3, 2)])

        rt = ReservationTable(grid)
        rt.add_path(path1)

        grid.pause_action_cost = 5
        path2 = a.find_path((0, 0), (2, 2), reservation_table=rt)
        self.assertListEqual(
            path2, [(0, 0), (1, 0), (2, 0), (2, 1), (2, 0), (2, 1), (2, 2)]
        )

        grid.pause_action_cost = 0.1
        path2 = a.find_path((0, 0), (2, 2), reservation_table=rt)
        self.assertListEqual(path2, [(0, 0), (0, 0), (0, 1), (0, 2), (1, 2), (2, 2)])

    def test_max_steps(self):
        """
        + - - - - - - +
        | s x         |
        | # x     #   |
        | x x   #     |
        | x # #     # |
        | x x x   # # |
        |     #     e |
        + - - - - - - +
        """
        weights = [
            [1, 1, 1, 1, 1, 1],
            [-1, 1, 1, 1, -1, 1],
            [1, 1, 1, -1, 1, 1],
            [1, -1, -1, 1, 1, -1],
            [1, 1, 1, 1, -1, -1],
            [1, 1, -1, 1, 1, 1],
        ]
        start, end = (0, 0), (5, 5)

        grid = Grid(weights=weights, diagonal_movement=DiagonalMovement.never)

        a = SpaceTimeAStar(grid)
        path = a.find_path(start, end, search_depth=8)
        self.assertEqual(len(path), 9)
        self.assertListEqual(
            path,
            [(0, 0), (1, 0), (1, 1), (1, 2), (0, 2), (0, 3), (0, 4), (1, 4), (2, 4)],
        )
