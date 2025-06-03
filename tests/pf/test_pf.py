import unittest
from w9_pathfinding import pf
from tests.factory import GridFactory


class TestPf(unittest.TestCase):
    """
    pytest tests/pf/test_pf.py::TestPf
    """

    def test_switch_graph(self):
        graph_factory = GridFactory(width=4, height=4)

        g1 = graph_factory()
        g2 = graph_factory()

        finder = pf.AStar(graph=g1)
        with self.assertRaises(AttributeError):
            finder.graph = g2  # error: the attribute 'graph' is read-only
