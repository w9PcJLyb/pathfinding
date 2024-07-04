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

    def test_find_components(self):
        graph = Graph(5, directed=False)

        def sorted_components(graph):
            components = graph.find_components()
            for i, x in enumerate(components):
                components[i] = sorted(x)
            components = sorted(components)
            return components

        self.assertListEqual(sorted_components(graph), [[0], [1], [2], [3], [4]])

        graph.add_edges([[1, 2, 100], [4, 3, 0]])
        self.assertListEqual(sorted_components(graph), [[0], [1, 2], [3, 4]])

        graph.add_edges([[2, 3, 100]])
        self.assertListEqual(sorted_components(graph), [[0], [1, 2, 3, 4]])

    def test_find_components_with_directed_graph(self):
        graph = Graph(5, directed=True)
        with self.assertRaises(ValueError):
            graph.find_components()

    def test_find_scc(self):
        with self.subTest("case 1"):
            graph = Graph(5, directed=True)

            def sorted_scc(graph):
                components = graph.find_scc()
                for i, x in enumerate(components):
                    components[i] = sorted(x)
                components = sorted(components)
                return components

            self.assertListEqual(sorted_scc(graph), [[0], [1], [2], [3], [4]])

            graph.add_edges([[2, 1, 1], [3, 2, 1]])
            self.assertListEqual(sorted_scc(graph), [[0], [1], [2], [3], [4]])

            graph.add_edges([[1, 3, 1]])
            self.assertListEqual(sorted_scc(graph), [[0], [1, 2, 3], [4]])

        with self.subTest("case 2"):
            graph = Graph(4, directed=True)
            graph.add_edges([[0, 1, 1], [3, 0, 1], [1, 2, 1], [2, 0, 1]])
            self.assertListEqual(sorted_scc(graph), [[0, 1, 2], [3]])

        with self.subTest("case 3"):
            graph = Graph(5, directed=True)
            edges = [
                [1, 0, 1],
                [2, 1, 1],
                [2, 0, 1],
                [3, 2, 1],
                [3, 0, 1],
                [4, 1, 1],
                [4, 2, 1],
            ]
            graph.add_edges(edges)
            self.assertListEqual(sorted_scc(graph), [[0], [1], [2], [3], [4]])
