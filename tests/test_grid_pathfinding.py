import math
import unittest
from w9_pathfinding import Grid, DFS, BFS, BiBFS, Dijkstra, BiDijkstra, AStar, BiAStar, DiagonalMovement

SHORTEST_PATH_ALGORITHMS = [Dijkstra, BiDijkstra, AStar, BiAStar]
ALL_ALGORITHMS = [DFS, BFS, BiBFS] + SHORTEST_PATH_ALGORITHMS


class TestSimpleGrid(unittest.TestCase):
    """
    pytest tests/test_grid_pathfinding.py::TestSimpleGrid
    """

    def test_0d(self):
        """
            + - +
            | s |
            + - +
        """
        obstacle_matrix = [[0]]
        start, end = (0, 0), (0, 0)
        answer = [(0, 0)]

        grid = Grid(obstacle_matrix)

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
        obstacle_matrix = [[0]]
        start, end = (0, 0), (10, 10)

        grid = Grid(obstacle_matrix)

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
        obstacle_matrix = [[0, 0, 0, 0, 0]]
        start, end = (0, 0), (4, 0)
        answer = [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)]

        grid = Grid(obstacle_matrix)

        for a in ALL_ALGORITHMS:
            with self.subTest(a.__name__):
                path = a(grid).find_path(start, end)
                self.assertListEqual(path, answer)

    def test_1d_no_path(self):
        """
            + -  -  -  -  - +
            | s        #  e |
            + -  -  -  -  - +
        """
        obstacle_matrix = [[0, 0, 0, 1, 0]]
        start, end = (0, 0), (4, 0)
        answer = []

        grid = Grid(obstacle_matrix)

        for a in ALL_ALGORITHMS:
            with self.subTest(a.__name__):
                path = a(grid).find_path(start, end)
                self.assertListEqual(path, answer)

    def test_passable_left_right_border(self):
        """
            + -  -  -  -  - +
            | s        #  e |
            + -  -  -  -  - +
        """
        obstacle_matrix = [[0, 0, 0, 1, 0]]
        start, end = (0, 0), (4, 0)
        answer = [(0, 0), (4, 0)]

        grid = Grid(obstacle_matrix, passable_left_right_border=True)

        for a in ALL_ALGORITHMS:
            with self.subTest(a.__name__):
                path = a(grid).find_path(start, end)
                self.assertListEqual(path, answer)

    def test_passable_up_down_border(self):
        """
            + -  -  +
            | s  #  |
            | #     |
            |       |
            | e  #  |
            + -  -  +
        """
        obstacle_matrix = [[0, 1], [1, 0], [0, 0], [0, 1]]
        start, end = (0, 0), (0, 3)
        answer = [(0, 0), (0, 3)]

        grid = Grid(obstacle_matrix, passable_up_down_border=True)

        for a in ALL_ALGORITHMS:
            with self.subTest(a.__name__):
                path = a(grid).find_path(start, end)
                self.assertListEqual(path, answer)

    def test_diagonal_movement(self):
        """
            + -  -  - +
            | s     # |
            |    #    |
            | #     e |
            + -  -  - +
        """
        obstacle_matrix = [[0, 0, 1], [0, 1, 0], [1, 0, 0]]
        start, end = (0, 0), (2, 2)
        path_len = {
            DiagonalMovement.never: 0,
            DiagonalMovement.only_when_no_obstacle: 0,
            DiagonalMovement.if_at_most_one_obstacle: 0,
            DiagonalMovement.always: 4,
        }

        for diagonal_movement, answer_len in path_len.items():
            grid = Grid(obstacle_matrix, diagonal_movement=diagonal_movement)
            for a in ALL_ALGORITHMS:
                with self.subTest(f"{a.__name__}, diagonal_movement={diagonal_movement}"):
                    path = a(grid).find_path(start, end)
                    self.assertEqual(len(path), answer_len)

    def test_diagonal_movement_2(self):
        """
            + -  -  - +
            | s  #  e |
            |    #    |
            |         |
            + -  -  - +
        """
        obstacle_matrix = [[0, 1, 0], [0, 1, 0], [0, 0, 0]]
        start, end = (0, 0), (2, 0)
        path_len = {
            DiagonalMovement.never: 7,
            DiagonalMovement.only_when_no_obstacle: 7,
            DiagonalMovement.if_at_most_one_obstacle: 5,
            DiagonalMovement.always: 5,
        }

        for diagonal_movement, answer_len in path_len.items():
            grid = Grid(obstacle_matrix, diagonal_movement=diagonal_movement)
            for a in ALL_ALGORITHMS:
                with self.subTest(f"{a.__name__}, diagonal_movement={diagonal_movement}"):
                    path = a(grid).find_path(start, end)
                    self.assertEqual(len(path), answer_len)

    def test_diagonal_movement_3(self):
        """
            + -  -  - +
            | s     # |
            |         |
            | #     e |
            + -  -  - +
        """
        obstacle_matrix = [[0, 0, 1], [0, 0, 0], [1, 0, 0]]
        start, end = (0, 0), (2, 2)
        path_len = {
            DiagonalMovement.never: 5,
            DiagonalMovement.only_when_no_obstacle: 3,
            DiagonalMovement.if_at_most_one_obstacle: 3,
            DiagonalMovement.always: 3,
        }

        for diagonal_movement, answer_len in path_len.items():
            grid = Grid(obstacle_matrix, diagonal_movement=diagonal_movement)
            for a in SHORTEST_PATH_ALGORITHMS:
                with self.subTest(f"{a.__name__}, diagonal_movement={diagonal_movement}"):
                    path = a(grid).find_path(start, end)
                    self.assertEqual(len(path), answer_len)

class TestShortestPath(unittest.TestCase):
    """
    pytest tests/test_grid_pathfinding.py::TestShortestPath
    """

    def test_case_1(self):
        """
            + - - - - - - +
            |   #     # e |
            |     #   x # |
            |   x # x     |
            | s   x # # # |
            + - - - - - - +
        """
        obstacle_matrix = [[0, 1, 0, 0, 1, 0], [0, 0, 1, 0, 0, 1], [0, 0, 1, 0, 0, 0], [0, 0, 0, 1, 1, 1]]
        start, end = (0, 3), (5, 0)
        grid = Grid(obstacle_matrix, diagonal_movement=DiagonalMovement.always)
        answer_len = 6

        for a in SHORTEST_PATH_ALGORITHMS:
            with self.subTest(a.__name__):
                path = a(grid).find_path(start, end)
                self.assertEqual(len(path), answer_len)

    def test_case_2(self):
        """
            + - - - - - - +
            |     # #   # |
            |   #   x s   |
            | #   x #   # |
            |   x       # |
            | e         # |
            + - - - - - - +
        """
        obstacle_matrix = [[0, 0, 1, 1, 0, 1], [0, 1, 0, 0, 0, 0], [1, 0, 0, 1, 0, 1], [0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 1]]
        start, end = (4, 1), (0, 4)
        grid = Grid(
            obstacle_matrix,
            diagonal_movement=DiagonalMovement.if_at_most_one_obstacle,
            passable_left_right_border=True
        )
        answer_len = 5

        for a in SHORTEST_PATH_ALGORITHMS:
            with self.subTest(a.__name__):
                path = a(grid).find_path(start, end)
                self.assertEqual(len(path), answer_len)

    def test_case_3(self):
        """
            + - - - - - - +
            | s x x x     |
            |         x   |
            |   # # # # x |
            |   #       x |
            |           x |
            |           e |
            + - - - - - - +
        """
        obstacle_matrix = [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 1, 1, 1, 1, 0], [0, 1, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]]
        start, end = (0, 0), (5, 5)
        grid = Grid(
            obstacle_matrix,
            diagonal_movement=DiagonalMovement.always,
            diaganal_movement_cost_multiplier=math.sqrt(2),
        )
        answer_cost = 6 + 2 * math.sqrt(2)

        for a in SHORTEST_PATH_ALGORITHMS:
            with self.subTest(a.__name__):
                path = a(grid).find_path(start, end)
                self.assertAlmostEqual(grid.calculate_cost(path), answer_cost, places=3)
