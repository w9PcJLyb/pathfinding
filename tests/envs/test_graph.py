import copy
import unittest
import numpy as np
from w9_pathfinding.envs import Graph


class TestGraph(unittest.TestCase):
    """
    pytest tests/envs/test_graph.py::TestGraph
    """

    def test_get_neighbors_with_directed_graph(self):
        graph = Graph(5, directed=True)
        self.assertEqual(graph.get_neighbors(0), [])

        graph.add_edges([(0, 1, 5), (0, 2, 6)])
        self.assertEqual(graph.get_neighbors(0), [(1, 5), (2, 6)])
        self.assertEqual(graph.get_neighbors(1), [])
        self.assertEqual(graph.get_neighbors(2), [])

        graph.add_edges([(2, 0, 4)])
        self.assertEqual(graph.get_neighbors(0), [(1, 5), (2, 6)])
        self.assertEqual(graph.get_neighbors(1), [])
        self.assertEqual(graph.get_neighbors(2), [(0, 4)])

    def test_get_neighbors_with_undirected_graph(self):
        graph = Graph(5, directed=False)
        self.assertEqual(graph.get_neighbors(0), [])

        graph.add_edges([(0, 1, 5), (0, 2, 6)])
        self.assertEqual(graph.get_neighbors(0), [(1, 5), (2, 6)])
        self.assertEqual(graph.get_neighbors(1), [(0, 5)])
        self.assertEqual(graph.get_neighbors(2), [(0, 6)])

        graph.add_edges([(2, 0, 4)])
        self.assertEqual(graph.get_neighbors(0), [(1, 5), (2, 6), (2, 4)])
        self.assertEqual(graph.get_neighbors(1), [(0, 5)])
        self.assertEqual(graph.get_neighbors(2), [(0, 4), (0, 6)])

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

    def test_is_valid_path(self):
        graph = Graph(5, edges=[[0, 1, 2], [1, 2, 4], [1, 1, 3]])

        self.assertTrue(graph.is_valid_path([0, 1, 2]))
        self.assertFalse(graph.is_valid_path([2, 1, 0]))
        self.assertFalse(graph.is_valid_path([0, 2]))
        self.assertFalse(graph.is_valid_path([0, 0]))
        self.assertTrue(graph.is_valid_path([1, 1]))

    def test_is_valid_path_with_undirected_graph(self):
        graph = Graph(5, directed=False, edges=[[0, 1, 2], [1, 2, 4], [1, 1, 3]])

        self.assertTrue(graph.is_valid_path([0, 1, 2]))
        self.assertTrue(graph.is_valid_path([2, 1, 0]))
        self.assertFalse(graph.is_valid_path([0, 2]))
        self.assertFalse(graph.is_valid_path([0, 0]))
        self.assertTrue(graph.is_valid_path([1, 1]))

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

    def test_to_dict(self):
        graph = Graph(5, edges=[[0, 1, 10], [1, 2, 20]])

        new_graph = Graph(**graph.to_dict())
        self.assertEqual(new_graph.num_vertices, 5)
        self.assertEqual(new_graph.num_edges, 2)

    def test_copy(self):
        graph = Graph(5, directed=False)
        graph.add_edges([(0, 1, 10), (1, 2, 20)])

        graph_copy = copy.copy(graph)
        graph_copy.add_edges([(2, 3, 30)])

        self.assertEqual(graph_copy.directed, False)
        self.assertEqual(graph_copy.num_edges, 3)
        self.assertEqual(graph_copy.edges, [[0, 1, 10], [1, 2, 20], [2, 3, 30]])

        self.assertEqual(graph_copy.directed, False)
        self.assertEqual(graph.num_edges, 2)
        self.assertEqual(graph.edges, [[0, 1, 10], [1, 2, 20]])

    def test_init_with_numpy_edges(self):
        num_vertices = 3
        edges = np.random.randint(0, num_vertices, size=(10, 2))
        graph = Graph(num_vertices, edges=edges)
        self.assertEqual(graph.num_vertices, 3)
        self.assertEqual(graph.num_edges, 10)
