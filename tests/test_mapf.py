import unittest
from collections import defaultdict
from w9_pathfinding import Graph, Grid, HCAStar, WHCAStar

MAPF_ALGORITHMS = [HCAStar, WHCAStar]


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
                paths = a(graph).mapf(starts, goals, despawn_at_destination=False)

                self.assertTrue(check_paths(graph, paths))
                for path, goal in zip(paths, goals):
                    self.assertEqual(len(paths[0]), len(path))
                    self.assertEqual(path[-1], goal)

    def test_with_grid(self):
        """
        + -  -  - +
        | #  s1 # |
        | s2    e1|
        | #  e2 # |
        + -  -  - +
        """
        grid = Grid([[-1, 1, -1], [1, 1, 1], [-1, 1, -1]])
        starts = [(1, 0), (0, 1)]
        goals = [(2, 1), (1, 2)]

        for a in MAPF_ALGORITHMS:
            with self.subTest(a.__name__):
                paths = a(grid).mapf(starts, goals, despawn_at_destination=False)

                self.assertTrue(check_paths(grid, paths))
                for path, goal in zip(paths, goals):
                    self.assertEqual(len(paths[0]), len(path))
                    self.assertEqual(path[-1], goal)

    def test_edge_collision(self):
        """
        + -  -  - +
        | #  s1 # |
        | #  e2 # |
        | #     # |
        |       # |
        | #     # |
        | #  e1 # |
        | #  s2 # |
        + -  -  - +
        """
        grid = Grid(
            [
                [-1, 1, -1],
                [-1, 1, -1],
                [-1, 1, -1],
                [1, 1, -1],
                [-1, 1, -1],
                [-1, 1, -1],
                [-1, 1, -1],
            ],
            edge_collision=True,
        )
        starts = [(1, 0), (1, 6)]
        goals = [(1, 5), (1, 1)]

        for a in [WHCAStar]:
            with self.subTest(a.__name__):
                paths = a(grid).mapf(starts, goals, despawn_at_destination=False)

                self.assertTrue(check_paths(grid, paths))
                for path, goal in zip(paths, goals):
                    self.assertEqual(len(paths[0]), len(path))
                    self.assertEqual(path[-1], goal)
