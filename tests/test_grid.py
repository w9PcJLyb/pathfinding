import copy
import unittest
import numpy as np
from w9_pathfinding import Grid, DiagonalMovement


class TestGrid(unittest.TestCase):
    """
    pytest tests/test_grid.py::TestGrid
    """

    def test_find_components(self):
        """
        + -  -  - +
        |    #  # |
        | #  #    |
        |         |
        + -  -  - +
        """
        weights = [[1, -1, -1], [-1, -1, 1], [1, 1, 1]]
        grid = Grid(weights)

        def sorted_components(graph):
            components = graph.find_components()
            for i, x in enumerate(components):
                components[i] = sorted(x)
            components = sorted(components)
            return components

        self.assertListEqual(
            sorted_components(grid), [[(0, 0)], [(0, 2), (1, 2), (2, 1), (2, 2)]]
        )

        grid.passable_up_down_border = True

        self.assertListEqual(
            sorted_components(grid), [[(0, 0), (0, 2), (1, 2), (2, 1), (2, 2)]]
        )

    def test_passable_borders(self):
        grid = Grid(width=3, height=3)

        def neighbors(x, y):
            return {p[0] for p in grid.get_neighbors((x, y))}

        grid.passable_left_right_border = False
        grid.passable_up_down_border = False
        self.assertEqual(neighbors(0, 0), {(0, 1), (1, 0)})

        grid.passable_left_right_border = True
        grid.passable_up_down_border = False
        self.assertEqual(neighbors(0, 0), {(0, 1), (1, 0), (2, 0)})

        grid.passable_left_right_border = False
        grid.passable_up_down_border = True
        self.assertEqual(neighbors(0, 0), {(0, 1), (1, 0), (0, 2)})

        grid.passable_left_right_border = True
        grid.passable_up_down_border = True
        self.assertEqual(neighbors(0, 0), {(0, 1), (1, 0), (0, 2), (2, 0)})

    def test_diagonal_movement(self):
        """
        + -  -  - +
        |         |
        |       # |
        |    #    |
        + -  -  - +
        """

        weights = [[1, 1, 1], [1, 1, -1], [1, -1, 1]]
        grid = Grid(weights)

        def diagonal_neighbors(x, y):
            neighbors = set()
            for p, _ in grid.get_neighbors((x, y)):
                if p[0] == x or p[1] == y:
                    continue
                neighbors.add(p)
            return neighbors

        grid.diagonal_movement = DiagonalMovement.never
        self.assertEqual(diagonal_neighbors(1, 1), set())

        grid.diagonal_movement = DiagonalMovement.only_when_no_obstacle
        self.assertEqual(diagonal_neighbors(1, 1), {(0, 0)})

        grid.diagonal_movement = DiagonalMovement.if_at_most_one_obstacle
        self.assertEqual(diagonal_neighbors(1, 1), {(0, 0), (2, 0), (0, 2)})

        grid.diagonal_movement = DiagonalMovement.always
        self.assertEqual(diagonal_neighbors(1, 1), {(0, 0), (2, 0), (0, 2), (2, 2)})

    def test_with_unknown_diagonal_movement(self):
        with self.assertRaises(ValueError):
            Grid(width=4, height=4, diagonal_movement=10)

        grid = Grid(width=4, height=4, diagonal_movement=1)
        with self.assertRaises(ValueError):
            grid.diagonal_movement = 10

    def test_init_with_numpy_weights(self):
        weights = np.random.random((4, 3))
        grid = Grid(weights)

        self.assertEqual(grid.width, weights.shape[1])
        self.assertEqual(grid.height, weights.shape[0])
        self.assertTrue(np.all(weights == grid.weights))

    def test_to_dict(self):
        weights = [[1, 1, 1], [1, 1, -1], [1, -1, 1]]
        grid = Grid(weights, passable_left_right_border=True)

        new_grid = Grid(**grid.to_dict())
        self.assertEqual(new_grid.weights, weights)
        self.assertEqual(new_grid.passable_left_right_border, True)
        self.assertEqual(new_grid.passable_up_down_border, False)

    def test_is_valid_path(self):
        grid = Grid(width=3, height=3)
        grid.add_obstacle((1, 1))

        self.assertTrue(grid.is_valid_path([(0, 0), (1, 0), (2, 0)]))
        self.assertFalse(grid.is_valid_path([(0, 0), (2, 0)]))
        self.assertFalse(grid.is_valid_path([(0, 1), (1, 1), (2, 1)]))

    def test_update_weight(self):
        grid = Grid(width=3, height=3)
        self.assertEqual(grid.get_weight((1, 1)), 1)

        grid.update_weight((1, 1), 10.9)
        self.assertEqual(grid.get_weight((1, 1)), 10.9)

        with self.assertRaises(ValueError):
            grid.update_weight((1, 1), -2)
        self.assertEqual(grid.get_weight((1, 1)), 10.9)

        with self.assertRaises(ValueError):
            grid.update_weight((9, 9), 10.9)

    def test_copy(self):
        weights = [[1, 2], [3, 4]]
        grid = Grid(
            weights, passable_left_right_border=True, passable_up_down_border=True
        )

        grid_copy = copy.copy(grid)
        grid_copy.passable_left_right_border = False
        grid_copy.update_weight((0, 0), 10)

        self.assertEqual(grid_copy.passable_left_right_border, False)
        self.assertEqual(grid_copy.passable_up_down_border, True)
        self.assertEqual(grid_copy.weights, [[10, 2], [3, 4]])

        self.assertEqual(grid.passable_left_right_border, True)
        self.assertEqual(grid.passable_up_down_border, True)
        self.assertEqual(grid.weights, weights)
