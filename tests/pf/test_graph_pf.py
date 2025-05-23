import unittest
from w9_pathfinding.envs import Graph
from w9_pathfinding.pf import DFS, BFS, BiBFS, Dijkstra, BiDijkstra #, SpaceTimeAStar

SHORTEST_PATH_ALGORITHMS = [Dijkstra, BiDijkstra] #, SpaceTimeAStar]
ALL_ALGORITHMS = [DFS, BFS, BiBFS, Dijkstra, BiDijkstra]


class TestSimpleGraph(unittest.TestCase):
    """
    pytest tests/pf/test_graph_pf.py::TestSimpleGraph
    """

    def test_graph_without_nodes(self):
        graph = Graph(0)

        for a in ALL_ALGORITHMS:
            with self.subTest(a.__name__):
                with self.assertRaises(ValueError):
                    a(graph).find_path(0, 0)

    def test_with_single_node(self):
        graph = Graph(1)

        for a in ALL_ALGORITHMS:
            with self.subTest(a.__name__):
                path = a(graph).find_path(0, 0)
                self.assertListEqual(path, [0])

    def test_with_two_nodes_without_edge(self):
        graph = Graph(2)

        for a in ALL_ALGORITHMS:
            with self.subTest(a.__name__):
                path = a(graph).find_path(0, 1)
                self.assertListEqual(path, [])

    def test_with_two_nodes_with_edge(self):
        graph = Graph(2)
        graph.add_edges([[0, 1, 1]])

        for a in ALL_ALGORITHMS:
            with self.subTest(a.__name__):
                path = a(graph).find_path(0, 1)
                self.assertListEqual(path, [0, 1])


class TestShortestPath(unittest.TestCase):
    """
    pytest tests/pf/test_graph_pf.py::TestShortestPath
    """

    def test_case_1(self):
        graph = Graph(3)
        graph.add_edges(
            [
                (0, 1, 49.9),
                (1, 2, 50),
                (0, 2, 100),
            ]
        )
        answer = [0, 1, 2]

        for a in SHORTEST_PATH_ALGORITHMS:
            with self.subTest(a.__name__):
                path = a(graph).find_path(0, 2)
                self.assertListEqual(path, answer, a.__name__)

    def test_case_2(self):
        graph = Graph(6)
        graph.add_edges(
            [
                (0, 1, 1),
                (0, 2, 3),
                (0, 3, 1),
                (1, 2, 1),
                (2, 3, 4),
                (2, 5, 1),
                (3, 2, 4),
                (3, 4, 2),
                (4, 5, 1),
            ]
        )
        answer = [0, 1, 2, 5]

        for a in SHORTEST_PATH_ALGORITHMS:
            with self.subTest(a.__name__):
                path = a(graph).find_path(0, 5)
                self.assertListEqual(path, answer)
