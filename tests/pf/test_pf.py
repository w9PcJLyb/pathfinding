import weakref
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

    def test_graph_lifetime(self):
        graph_factory = GridFactory(width=4, height=4, weighted=True)

        # Create a random graph object, it will create both Python and C++ instances
        graph = graph_factory()

        # Create a weakref to track when the Python object is garbage collected
        graph_ref = weakref.ref(graph)

        finder = pf.AStar(graph=graph)

        # Delete the graph variable
        # The object should still exist because finder is holding a reference to it
        del graph
        self.assertIsNotNone(graph_ref())

        # Confirm that the C++ instance still valid and the pathfinding still works
        path = finder.find_path((0, 0), (3, 3))
        self.assertGreaterEqual(len(path), 7)

        # Delete the pathfinder. This should release the last reference to the graph
        # Now the graph object (both Python and C++ instances) should be gone
        del finder
        self.assertIsNone(graph_ref())
