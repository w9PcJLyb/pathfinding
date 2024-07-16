import unittest
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
