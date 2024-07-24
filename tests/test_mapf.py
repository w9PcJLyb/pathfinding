import unittest
from collections import defaultdict
from w9_pathfinding import Grid, DiagonalMovement, HCAStar, WHCAStar, ReservationTable

MAPF_ALGORITHMS = [HCAStar, WHCAStar]


class TestFindPath(unittest.TestCase):
    """
    pytest tests/test_mapf.py::TestFindPath
    """

    def test_3x4_with_two_agents(self):
        """
        + -  -  -  - +
        | s        # |
        |    #     # |
        |       e2 e1|
        + -  -  -  - +
        """
        weights = [[1, 2, 0.9, -1], [1, -1, 0.9, -1], [1, 1, 1, 1]]
        grid = Grid(weights=weights, diagonal_movement=DiagonalMovement.never)

        a = HCAStar(grid)

        path1 = a.find_path((0, 0), (3, 2))
        self.assertListEqual(path1, [(0, 0), (0, 1), (0, 2), (1, 2), (2, 2), (3, 2)])

        rt = ReservationTable(grid)
        rt.add_path(path1)

        grid.pause_action_cost = 5
        path2 = a.find_path((0, 0), (2, 2), reservation_table=rt)
        self.assertListEqual(
            path2, [(0, 0), (1, 0), (2, 0), (2, 1), (2, 0), (2, 1), (2, 2)]
        )

        grid.pause_action_cost = 0.1
        path2 = a.find_path((0, 0), (2, 2), reservation_table=rt)
        self.assertListEqual(path2, [(0, 0), (0, 0), (0, 1), (0, 2), (1, 2), (2, 2)])

    def test_max_steps(self):
        """
        + - - - - - - +
        | s x         |
        | # x     #   |
        | x x   #     |
        | x # #     # |
        | x x x   # # |
        |     #     e |
        + - - - - - - +
        """
        weights = [
            [1, 1, 1, 1, 1, 1],
            [-1, 1, 1, 1, -1, 1],
            [1, 1, 1, -1, 1, 1],
            [1, -1, -1, 1, 1, -1],
            [1, 1, 1, 1, -1, -1],
            [1, 1, -1, 1, 1, 1],
        ]
        start, end = (0, 0), (5, 5)

        grid = Grid(weights=weights, diagonal_movement=DiagonalMovement.never)

        a = HCAStar(grid)
        path = a.find_path(start, end, search_depth=8)
        self.assertEqual(len(path), 9)
        self.assertListEqual(
            path,
            [(0, 0), (1, 0), (1, 1), (1, 2), (0, 2), (0, 3), (0, 4), (1, 4), (2, 4)],
        )


def check_paths(paths, swapping_conflict=True):
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

        if swapping_conflict and time > 0:
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

    def test_3x3(self):
        """
        + -  -  - +
        | s1 s2   |
        | #     # |
        | e1 e2   |
        + -  -  - +
        """
        grid = Grid([[1, 1, 1], [-1, 1, -1], [1, 1, 1]])
        starts = [(0, 0), (1, 0)]
        goals = [(0, 2), (1, 2)]

        swapping_conflict = True

        for a in MAPF_ALGORITHMS:
            with self.subTest(a.__name__):
                paths = a(grid).mapf(
                    starts,
                    goals,
                    despawn_at_destination=False,
                    swapping_conflict=swapping_conflict,
                )

                self.assertTrue(check_paths(paths, swapping_conflict))
                for path, goal in zip(paths, goals):
                    self.assertEqual(len(paths[0]), len(path))
                    self.assertEqual(path[-1], goal)

    def test_3x7(self):
        """
        + -  -  - +
        | #  s1 # |
        | #  e2 # |
        | #     # |
        |       # |
        | #     # |
        | #  e2 # |
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
            ]
        )
        starts = [(1, 0), (1, 6)]
        goals = [(1, 5), (1, 1)]

        swapping_conflict = True

        for a in [WHCAStar]:
            with self.subTest(a.__name__):
                paths = a(grid).mapf(
                    starts,
                    goals,
                    despawn_at_destination=False,
                    swapping_conflict=swapping_conflict,
                )

                self.assertTrue(check_paths(paths, swapping_conflict))
                for path, goal in zip(paths, goals):
                    self.assertEqual(len(paths[0]), len(path))
                    self.assertEqual(path[-1], goal)
