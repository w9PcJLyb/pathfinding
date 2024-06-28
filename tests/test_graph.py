import unittest
from w9_pathfinding import Graph


class TestGraph(unittest.TestCase):
    """
    pytest tests/test_graph.py::TestGraph
    """

    def test_reverse_inplace(self):
        graph = Graph(5)
        graph.reverse(inplace=True)
        self.assertEqual(graph.edges, [])

        graph.add_edges([[0, 1, 2], [1, 2, 4]])
        graph.reverse(inplace=True)
        self.assertEqual(graph.num_vertices, 5)
        self.assertEqual(graph.edges, [[1, 0, 2], [2, 1, 4]])

    def test_reverse(self):
        graph = Graph(5)
        reversed_graph = graph.reverse(inplace=False)
        self.assertEqual(reversed_graph.num_vertices, 5)
        self.assertEqual(reversed_graph.edges, [])

        graph.add_edges([[0, 1, 2], [1, 2, 4]])
        reversed_graph = graph.reverse(inplace=False)
        self.assertEqual(graph.num_vertices, 5)
        self.assertEqual(graph.edges, [[0, 1, 2], [1, 2, 4]])
        self.assertEqual(reversed_graph.num_vertices, 5)
        self.assertEqual(reversed_graph.edges, [[1, 0, 2], [2, 1, 4]])

    def test_calc_cost(self):
        graph = Graph(5)
        self.assertEqual(graph.calculate_cost([]), 0)
        self.assertEqual(graph.calculate_cost([0]), 0)
        self.assertEqual(graph.calculate_cost([0, 1]), -1)

        graph.add_edges([[0, 1, 2], [1, 2, 4]])
        self.assertEqual(graph.calculate_cost([0, 1]), 2)
        self.assertEqual(graph.calculate_cost([1, 2]), 4)
        self.assertEqual(graph.calculate_cost([0, 1, 2]), 2 + 4)

        graph.add_edges([[1, 2, 3]])
        self.assertEqual(graph.calculate_cost([0, 1]), 2)
        self.assertEqual(graph.calculate_cost([1, 2]), 3)
        self.assertEqual(graph.calculate_cost([0, 1, 2]), 2 + 3)
