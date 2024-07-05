import unittest
from w9_pathfinding import Grid


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
