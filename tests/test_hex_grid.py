import unittest
import numpy as np
from w9_pathfinding import HexGrid


class TestHexGrid(unittest.TestCase):
    """
    pytest tests/test_hex_grid.py::TestHexGrid
    """

    def test_find_components(self):
        """
        _ # _
         # # _
        _ # #
        """
        weights = [[1, -1, 1], [-1, -1, 1], [1, -1, -1]]
        grid = HexGrid(weights)

        def sorted_components(graph):
            components = graph.find_components()
            for i, x in enumerate(components):
                components[i] = sorted(x)
            components = sorted(components)
            return components

        self.assertListEqual(
            sorted_components(grid), [[(0, 0)], [(0, 2)], [(2, 0), (2, 1)]]
        )

        grid.passable_left_right_border = True

        self.assertListEqual(
            sorted_components(grid), [[(0, 0), (0, 2), (2, 0), (2, 1)]]
        )

    def test_passable_up_down_border_with_odd_height(self):
        with self.assertRaises(ValueError):
            HexGrid(width=4, height=3, passable_up_down_border=True)

        grid = HexGrid(width=4, height=3, passable_up_down_border=False)
        with self.assertRaises(ValueError):
            grid.passable_up_down_border = True

    def test_passable_borders(self):
        grid = HexGrid(width=4, height=4)

        def neighbors(x, y):
            return {p[0] for p in grid.get_neighbors((x, y))}

        grid.passable_left_right_border = False
        grid.passable_up_down_border = False
        self.assertEqual(neighbors(0, 0), {(0, 1), (1, 0)})

        grid.passable_left_right_border = True
        grid.passable_up_down_border = False
        self.assertEqual(neighbors(0, 0), {(0, 1), (1, 0), (3, 0), (3, 1)})

        grid.passable_left_right_border = False
        grid.passable_up_down_border = True
        self.assertEqual(neighbors(0, 0), {(0, 1), (1, 0), (0, 3)})

        grid.passable_left_right_border = True
        grid.passable_up_down_border = True
        self.assertEqual(
            neighbors(0, 0), {(0, 1), (1, 0), (3, 0), (3, 1), (0, 3), (3, 3)}
        )

    def test_init_with_numpy_weights(self):
        weights = np.random.random((4, 3))
        grid = HexGrid(weights)

        self.assertEqual(grid.width, weights.shape[1])
        self.assertEqual(grid.height, weights.shape[0])
        self.assertTrue(np.all(weights == grid.weights))
