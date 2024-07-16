import unittest
from w9_pathfinding import Grid3D


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
