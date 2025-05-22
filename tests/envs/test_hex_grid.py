import unittest
import numpy as np
from w9_pathfinding.graph import HexGrid, HexLayout


class TestHexGrid(unittest.TestCase):
    """
    pytest tests/test_hex_grid.py::TestHexGrid
    """

    def test_layout(self):

        def neighbors(grid, x, y):
            return {p[0] for p in grid.get_neighbors((x, y))}

        grid = HexGrid(width=4, height=4, layout=HexLayout.odd_r)
        self.assertEqual(neighbors(grid, 0, 0), {(1, 0), (0, 1)})
        self.assertEqual(neighbors(grid, 1, 0), {(0, 0), (2, 0), (0, 1), (1, 1)})

        grid = HexGrid(width=4, height=4, layout=HexLayout.even_r)
        self.assertEqual(neighbors(grid, 0, 0), {(1, 0), (0, 1), (1, 1)})
        self.assertEqual(neighbors(grid, 1, 0), {(0, 0), (2, 0), (1, 1), (2, 1)})

        grid = HexGrid(width=4, height=4, layout=HexLayout.odd_q)
        self.assertEqual(neighbors(grid, 0, 0), {(1, 0), (0, 1)})
        self.assertEqual(
            neighbors(grid, 1, 0), {(0, 0), (2, 0), (0, 1), (2, 1), (1, 1)}
        )

        grid = HexGrid(width=4, height=4, layout=HexLayout.even_q)
        self.assertEqual(neighbors(grid, 0, 0), {(1, 0), (0, 1), (1, 1)})
        self.assertEqual(neighbors(grid, 1, 0), {(0, 0), (2, 0), (1, 1)})

    def test_init_with_unknown_layout(self):
        with self.assertRaises(ValueError):
            HexGrid(width=4, height=4, layout=10)

    def test_update_layout(self):
        grid = HexGrid(width=4, height=4, layout=HexLayout.even_q)
        with self.assertRaises(AttributeError):
            grid.layout = HexLayout.odd_q

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

    def test_pointy_top_layout_with_odd_height(self):
        with self.assertRaises(ValueError):
            HexGrid(
                width=4, height=3, layout=HexLayout.odd_r, passable_up_down_border=True
            )

        grid = HexGrid(
            width=4, height=3, layout=HexLayout.odd_r, passable_up_down_border=False
        )
        with self.assertRaises(ValueError):
            grid.passable_up_down_border = True

    def test_flat_top_layout_with_odd_width(self):
        with self.assertRaises(ValueError):
            HexGrid(
                width=3,
                height=4,
                layout=HexLayout.odd_q,
                passable_left_right_border=True,
            )

        grid = HexGrid(
            width=3, height=4, layout=HexLayout.odd_q, passable_left_right_border=False
        )
        with self.assertRaises(ValueError):
            grid.passable_left_right_border = True

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

    def test_to_dict(self):
        weights = [[1, 1, 1], [1, 1, -1], [1, -1, 1]]
        grid = HexGrid(weights, passable_left_right_border=True)

        new_grid = HexGrid(**grid.to_dict())
        self.assertEqual(new_grid.weights, weights)
        self.assertEqual(new_grid.passable_left_right_border, True)
        self.assertEqual(new_grid.passable_up_down_border, False)
