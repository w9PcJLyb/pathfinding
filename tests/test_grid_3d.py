import unittest
import numpy as np
from w9_pathfinding.graph import Grid3D


class TestGrid3D(unittest.TestCase):
    """
    pytest tests/test_grid_3d.py::TestGrid3D
    """

    def test_find_components(self):
        """
        z = 0
        + -  -  - +
        |    #  # |
        | #  #  # |
        | #       |
        + -  -  - +

        z = 1
        + -  -  - +
        |         |
        | #  #  # |
        |    #    |
        + -  -  - +

        """
        weights = [
            [[1, -1, -1], [-1, -1, -1], [-1, 1, 1]],
            [[1, 1, 1], [-1, -1, -1], [1, -1, 1]],
        ]
        grid = Grid3D(weights)

        def sorted_components(graph):
            components = graph.find_components()
            for i, x in enumerate(components):
                components[i] = sorted(x)
            components = sorted(components)
            return components

        components = sorted_components(grid)
        self.assertEqual(len(components), 3)
        self.assertListEqual(
            components,
            [
                [(0, 0, 0), (0, 0, 1), (1, 0, 1), (2, 0, 1)],
                [(0, 2, 1)],
                [(1, 2, 0), (2, 2, 0), (2, 2, 1)],
            ],
        )

        grid.passable_borders = True
        components = sorted_components(grid)
        self.assertEqual(len(components), 1)
        self.assertEqual(len(components[0]), 8)

    def test_passable_borders(self):
        grid = Grid3D(width=3, height=3, depth=3)

        def neighbors(x, y, z):
            return {p[0] for p in grid.get_neighbors((x, y, z))}

        grid.passable_borders = False
        self.assertEqual(neighbors(0, 0, 0), {(0, 1, 0), (1, 0, 0), (0, 0, 1)})

        grid.passable_borders = True
        self.assertEqual(
            neighbors(0, 0, 0),
            {(1, 0, 0), (0, 1, 0), (0, 0, 1), (2, 0, 0), (0, 2, 0), (0, 0, 2)},
        )

    def test_init_with_numpy_weights(self):
        weights = np.random.random((4, 3, 2))
        grid = Grid3D(weights)

        self.assertEqual(grid.width, weights.shape[2])
        self.assertEqual(grid.height, weights.shape[1])
        self.assertEqual(grid.depth, weights.shape[0])
        self.assertTrue(np.all(weights == grid.weights))

    def test_to_dict(self):
        weights = [[[1, 1, 1], [1, 1, -1], [1, -1, 1]]]
        grid = Grid3D(weights, passable_borders=True)

        new_grid = Grid3D(**grid.to_dict())
        self.assertEqual(new_grid.weights, weights)
        self.assertEqual(new_grid.passable_borders, True)
