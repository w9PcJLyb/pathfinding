import unittest
from collections import defaultdict
from w9_pathfinding import mapf
from w9_pathfinding.envs import Graph, Grid, ReservationTable

COMPLETE_ALGORITHMS = [
    {
        "name": "CBS (disjoint_splitting=False)",
        "class": mapf.CBS,
        "params": {"disjoint_splitting": False},
    },
    {
        "name": "CBS (disjoint_splitting=True)",
        "class": mapf.CBS,
        "params": {"disjoint_splitting": True},
    },
    {
        "name": "ICTS(ict_pruning=False)",
        "class": mapf.ICTS,
        "params": {"ict_pruning": False},
    },
    {
        "name": "ICTS(ict_pruning=True)",
        "class": mapf.ICTS,
        "params": {"ict_pruning": True},
    },
    {
        "name": "A(od=False)",
        "class": mapf.MultiAgentAStar,
        "params": {"operator_decomposition": False},
    },
    {
        "name": "A(od=True)",
        "class": mapf.MultiAgentAStar,
        "params": {"operator_decomposition": True},
    },
]

MAPF_ALGORITHMS = [
    {"name": "HCA*", "class": mapf.HCAStar},
    {"name": "WHCA*", "class": mapf.WHCAStar},
] + COMPLETE_ALGORITHMS


def check_paths(graph, paths):
    if not paths:
        return True

    longest_path = max(len(path) for path in paths)

    time = 0
    while time < longest_path:
        positions = defaultdict(list)
        for agent_id, path in enumerate(paths):
            p = path[time] if time < len(path) else path[-1]
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

    def test_without_agents(self):
        grid = Grid(width=5, height=5)

        for a in MAPF_ALGORITHMS:
            with self.subTest(a["name"]):
                paths = a["class"](grid).mapf([], [], **a.get("params", {}))
                self.assertEqual(paths, [])

    def test_with_directed_graph(self):
        graph = Graph(5, edges=[[0, 2], [1, 2], [2, 3], [2, 4]])
        starts = [0, 1]
        goals = [3, 4]

        for a in MAPF_ALGORITHMS:
            with self.subTest(a["name"]):
                paths = a["class"](graph).mapf(starts, goals, **a.get("params", {}))

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
            with self.subTest(a["name"]):
                paths = a["class"](grid).mapf(starts, goals, **a.get("params", {}))

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
            with self.subTest(a["name"]):
                paths = a["class"](grid).mapf(starts, goals, **a.get("params", {}))

                self.assertTrue(check_paths(grid, paths))
                for path, goal in zip(paths, goals):
                    self.assertLessEqual(len(path), 5)
                    self.assertEqual(path[-1], goal)

    def test_grid_with_dynamic_obstacles(self):
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
            with self.subTest(a["name"]):
                paths = a["class"](grid).mapf(
                    starts, goals, reservation_table=rt, **a.get("params", {})
                )
                self.assertTrue(check_paths(grid, paths + [reserved_path]))
                for path, goal in zip(paths, goals):
                    self.assertLessEqual(len(path), 6)
                    self.assertEqual(path[-1], goal)

    def test_grid_with_dynamic_obstacles_that_block_the_path(self):
        grid = Grid(width=4, height=2, edge_collision=True)
        starts = [(0, 0), (3, 1)]
        goals = [(3, 0), (0, 1)]
        reserved_paths = [[(1, 0), (2, 0), (3, 0)], [(1, 1), (2, 1), (3, 1)]]

        rt = ReservationTable(grid)
        for path in reserved_paths:
            rt.add_path(path)

        for a in MAPF_ALGORITHMS:
            with self.subTest(a["name"]):
                paths = a["class"](grid).mapf(
                    starts, goals, reservation_table=rt, **a.get("params", {})
                )
                self.assertEqual(paths, [])

    def test_grid_with_dynamic_obstacles_that_moves_through(self):
        grid = Grid(width=4, height=4, edge_collision=True)
        start = (2, 1)
        goal = start
        reserved_path = [(0, 1), (1, 1), (2, 1), (3, 1)]

        rt = ReservationTable(grid)
        rt.add_path(reserved_path)

        for a in MAPF_ALGORITHMS:
            with self.subTest(a["name"]):
                paths = a["class"](grid).mapf(
                    [start], [goal], reservation_table=rt, **a.get("params", {})
                )
                self.assertGreater(len(paths[0]), 2)
                self.assertTrue(check_paths(grid, paths + [reserved_path]))

    def test_max_length(self):
        grid = Grid(width=5, height=5, edge_collision=True)
        starts = [(0, 0)]
        goals = [(4, 0)]

        for a in MAPF_ALGORITHMS:
            for d in range(8):
                with self.subTest(f"{a['name']}(max_length={d})"):
                    paths = a["class"](grid).mapf(starts, goals, max_length=d)
                    if d < 4:
                        self.assertEqual(len(paths), 0)
                    else:
                        self.assertEqual(len(paths), 1)
                        self.assertEqual(len(paths[0]), 5)
                        self.assertEqual(paths[0][-1], goals[0])

    def test_two_agents_with_the_same_goal(self):
        grid = Grid([[1, -1, 1], [1, 1, 1], [-1, 1, -1]])
        starts = [(0, 0), (2, 0)]
        goals = [(1, 1), (1, 1)]

        for a in MAPF_ALGORITHMS:
            with self.subTest(a["name"]):
                paths = a["class"](grid).mapf(starts, goals, **a.get("params", {}))
                self.assertEqual(paths, [])


class TestComplete(unittest.TestCase):
    """
    pytest tests/test_mapf.py::TestComplete
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

        for a in COMPLETE_ALGORITHMS:
            with self.subTest(a["name"]):
                paths = a["class"](grid).mapf(starts, goals, **a.get("params", {}))

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

        for a in COMPLETE_ALGORITHMS:
            with self.subTest(a["name"]):
                paths = a["class"](grid).mapf(
                    starts, goals, max_time=10, **a.get("params", {})
                )

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

        for a in COMPLETE_ALGORITHMS:
            with self.subTest(a["name"]):
                paths = a["class"](grid).mapf(
                    starts, goals, max_time=10, **a.get("params", {})
                )

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

        for a in COMPLETE_ALGORITHMS:
            with self.subTest(a["name"]):
                paths = a["class"](grid).mapf(
                    starts, goals, max_time=10, **a.get("params", {})
                )

                self.assertEqual(len(paths), 3)
                self.assertTrue(check_paths(grid, paths))
                for path, goal in zip(paths, goals):
                    self.assertLessEqual(len(path), 5)
                    self.assertEqual(path[-1], goal)

    def test_optimality(self):
        """
        + - - - - +
        |         |
        |     #   |
        |         |
        | # #     |
        + - - - - +
        """
        grid = Grid(
            [
                [1.0, 1.0, 1.0, 1.0],
                [1.0, 1.0, -1.0, 1.0],
                [1.0, 1.0, 1.0, 1.0],
                [-1.0, -1.0, 1.0, 1.0],
            ],
            edge_collision=True,
        )
        starts, goals = ((3, 3), (1, 0), (1, 1)), ((3, 2), (1, 0), (3, 1))

        for a in COMPLETE_ALGORITHMS:
            with self.subTest(a["name"]):
                paths = a["class"](grid).mapf(
                    starts, goals, max_time=10, **a.get("params", {})
                )
                self.assertEqual(len(paths), 3)
                self.assertTrue(check_paths(grid, paths))

                self.assertListEqual(
                    paths,
                    [
                        [(3, 3), (3, 2)],
                        [(1, 0), (0, 0), (1, 0)],
                        [(1, 1), (1, 0), (2, 0), (3, 0), (3, 1)],
                    ],
                )

    def test_optimality_2(self):
        """
        + - - - - +
        |         |
        | #       |
        |       # |
        | #   # # |
        + - - - - +
        """

        grid = Grid(
            [
                [1.0, 1.0, 1.0, 1.0],
                [-1.0, 1.0, 1.0, 1.0],
                [1.0, 1.0, 1.0, -1.0],
                [-1.0, 1.0, -1.0, -1.0],
            ],
            edge_collision=True,
        )
        starts = ((0, 0), (2, 0), (2, 2), (3, 0))
        goals = ((1, 0), (0, 2), (2, 1), (0, 0))

        for a in COMPLETE_ALGORITHMS:
            with self.subTest(a["name"]):
                paths = a["class"](grid).mapf(
                    starts, goals, max_time=10, **a.get("params", {})
                )
                self.assertEqual(len(paths), 4)
                self.assertTrue(check_paths(grid, paths))

                self.assertListEqual(paths[0], [(0, 0), (0, 0), (1, 0), (1, 1), (1, 0)])
                self.assertListEqual(paths[1], [(2, 0), (1, 0), (1, 1), (1, 2), (0, 2)])
                self.assertListEqual(paths[2], [(2, 2), (2, 1)])
                self.assertIn(
                    paths[3],
                    [
                        [(3, 0), (2, 0), (2, 0), (1, 0), (0, 0)],
                        [(3, 0), (3, 0), (2, 0), (1, 0), (0, 0)],
                    ],
                )
