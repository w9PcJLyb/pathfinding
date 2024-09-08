import unittest
from collections import defaultdict
import w9_pathfinding as pf
from w9_pathfinding import CBS, Graph, Grid, ReservationTable

MAPF_ALGORITHMS = [pf.HCAStar, pf.WHCAStar, pf.CBS, pf.MultiAgentAStar]


def check_paths(graph, paths):
    if not paths:
        return True

    time = 0
    while True:
        positions = defaultdict(list)
        for agent_id, path in enumerate(paths):
            if time < len(path):
                p = path[time]
                positions[p].append(agent_id)

        if not positions:
            break

        for p, agent_ids in positions.items():
            if len(agent_ids) > 1:
                print(f"Collision at {time}: node = {p}, agents = {agent_ids}")
                return False

        if graph.edge_collision and time > 0:
            edges = defaultdict(list)
            for agent_id, path in enumerate(paths):
                if time < len(path):
                    p1, p2 = path[time - 1], path[time]
                    if p1 == p2:
                        continue
                    edges[(p1, p2)].append(agent_id)
                    edges[(p2, p1)].append(agent_id)

            for edge, agent_ids in edges.items():
                if len(agent_ids) > 1:
                    print(f"Collision at {time}: edge = {edge}, agents = {agent_ids}")
                    return False

        time += 1

    return True


class TestMAPF(unittest.TestCase):
    """
    pytest tests/test_mapf.py::TestMAPF
    """

    def test_with_directed_graph(self):
        graph = Graph(5, edges=[[0, 2], [1, 2], [2, 3], [2, 4]])
        starts = [0, 1]
        goals = [3, 4]

        for a in MAPF_ALGORITHMS:
            with self.subTest(a.__name__):
                paths = a(graph).mapf(starts, goals)

                self.assertTrue(check_paths(graph, paths))
                for path, goal in zip(paths, goals):
                    self.assertLessEqual(len(path), 4)
                    self.assertEqual(path[-1], goal)

    def test_with_grid(self):
        """
        + -  -  - +
        | #  s1 # |
        | s2    g1|
        | #  g2 # |
        + -  -  - +
        """
        grid = Grid([[-1, 1, -1], [1, 1, 1], [-1, 1, -1]])
        starts = [(1, 0), (0, 1)]
        goals = [(2, 1), (1, 2)]

        for a in MAPF_ALGORITHMS:
            with self.subTest(a.__name__):
                paths = a(grid).mapf(starts, goals)

                self.assertTrue(check_paths(grid, paths))
                for path, goal in zip(paths, goals):
                    self.assertLessEqual(len(path), 4)
                    self.assertEqual(path[-1], goal)

    def test_edge_collision(self):
        """
        + -  -  -  - +
        | s1 g2 g1 s2|
        | #        # |
        + -  -  -  - +
        """
        grid = Grid([[1, 1, 1, 1], [-1, 1, 1, -1]], edge_collision=True)
        starts = [(0, 0), (3, 0)]
        goals = [(2, 0), (1, 0)]

        for a in MAPF_ALGORITHMS:
            with self.subTest(a.__name__):
                paths = a(grid).mapf(starts, goals)

                self.assertTrue(check_paths(grid, paths))
                for path, goal in zip(paths, goals):
                    self.assertLessEqual(len(path), 5)
                    self.assertEqual(path[-1], goal)

    def test_grid_with_reservations(self):
        """
        + -  -  -  - +
        | s1       g1|
        | s2       g2|
        + -  -  -  - +
        """
        grid = Grid(width=4, height=2, edge_collision=True)
        starts = [(0, 0), (0, 1)]
        goals = [(3, 0), (3, 1)]
        reserved_path = [(3, 0), (2, 0), (1, 0), (0, 0)]

        rt = ReservationTable(grid)
        rt.add_path(reserved_path)

        for a in MAPF_ALGORITHMS:
            with self.subTest(a.__name__):
                paths = a(grid).mapf(starts, goals, reservation_table=rt)
                self.assertTrue(check_paths(grid, paths + [reserved_path]))
                for path, goal in zip(paths, goals):
                    self.assertLessEqual(len(path), 6)
                    self.assertEqual(path[-1], goal)


class TestCBS(unittest.TestCase):
    """
    pytest tests/test_mapf.py::TestCBS
    """

    def test_edge_collision(self):
        """
        + -  -  -  -  -  -  - +
        | s1 g2          g1 s2|
        | #  #  #     #  #  # |
        + -  -  -  -  -  -  - +
        """
        grid = Grid(
            [[1, 1, 1, 1, 1, 1, 1], [-1, -1, -1, 1, -1, -1, -1]],
            edge_collision=True,
        )
        starts = [(0, 0), (6, 0)]
        goals = [(5, 0), (1, 0)]

        paths = CBS(grid).mapf(starts, goals)
        self.assertEqual(len(paths), 2)
        self.assertTrue(check_paths(grid, paths))
        for path, goal in zip(paths, goals):
            self.assertLessEqual(len(path), 8)
            self.assertEqual(path[-1], goal)

    def test_edge_collision_2(self):
        """
        + -  -  -  -  -  - +
        | s1 s2 g2 g1      |
        | #  #  #  #     # |
        + -  -  -  -  -  - +
        """
        grid = Grid(
            [[1, 1, 1, 1, 1, 1], [-1, -1, -1, -1, 1, -1]],
            edge_collision=True,
        )
        starts = [(0, 0), (1, 0)]
        goals = [(3, 0), (2, 0)]

        paths = CBS(grid).mapf(starts, goals, max_time=10)
        self.assertEqual(len(paths), 2)
        self.assertTrue(check_paths(grid, paths))
        for path, goal in zip(paths, goals):
            self.assertLessEqual(len(path), 8)
            self.assertEqual(path[-1], goal)

    def test_edge_collision_3(self):
        """
        + -  -  -  -  -  - +
        | s1 g2 g1 s2      |
        | #  #  #  #     # |
        + -  -  -  -  -  - +
        """
        grid = Grid(
            [[1, 1, 1, 1, 1, 1], [-1, -1, -1, -1, 1, -1]],
            edge_collision=True,
        )
        starts = [(0, 0), (3, 0)]
        goals = [(2, 0), (1, 0)]

        paths = CBS(grid).mapf(starts, goals, max_time=10)
        self.assertEqual(len(paths), 2)
        self.assertTrue(check_paths(grid, paths))
        for path, goal in zip(paths, goals):
            self.assertLessEqual(len(path), 9)
            self.assertEqual(path[-1], goal)

    def test_with_three_agents(self):
        """
        + -  -  -  +
        |          |
        | #        |
        + -  -  -  +
        """
        grid = Grid([[1, 1, 1], [-1, 1, 1]], edge_collision=True)
        starts, goals = ((2, 0), (0, 0), (1, 0)), ((0, 0), (2, 1), (1, 1))

        paths = CBS(grid).mapf(starts, goals, max_time=10)
        self.assertEqual(len(paths), 3)
        self.assertTrue(check_paths(grid, paths))
        for path, goal in zip(paths, goals):
            self.assertLessEqual(len(path), 5)
            self.assertEqual(path[-1], goal)
