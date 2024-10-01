import time
import argparse
from copy import copy
from collections import defaultdict

import w9_pathfinding as pf
from tests.stress_tests.utils import run_graph
from tests.stress_tests.random_instance import GridGenerator, random_queries

NUM_GRAPHS = 100
NUM_AGENTS = 5

UNWEIGHTED_GRID_GENERATOR = GridGenerator(
    width=10,
    height=10,
    obstacle_percentage=0.4,
    weighted=False,
)

WEIGHTED_GRID_GENERATOR = GridGenerator(
    width=10,
    height=10,
    obstacle_percentage=0.4,
    weighted=True,
    min_weight=1,
    max_weight=5,
)

ALGORITHMS = [
    {"name": "HCA*", "class": pf.HCAStar, "unw": 0, "w": 0},
    {"name": "WHCA*", "class": pf.WHCAStar, "unw": 0, "w": 0},
    {
        "name": "CBS (disjoint_splitting=False)",
        "class": pf.CBS,
        "params": {"max_time": 0.1, "disjoint_splitting": False},
        "unw": 1,
        "w": 1,
    },
    {
        "name": "CBS (disjoint_splitting=True)",
        "class": pf.CBS,
        "params": {"max_time": 0.1, "disjoint_splitting": True},
        "unw": 1,
        "w": 1,
    },
    {"name": "ICTS", "class": pf.ICTS, "params": {"max_time": 0.1}, "unw": 1, "w": 0},
    {
        "name": "A*",
        "class": pf.MultiAgentAStar,
        "params": {"max_time": 0.1, "od": False},
        "unw": 1,
        "w": 1,
    },
    {
        "name": "A* (OD)",
        "class": pf.MultiAgentAStar,
        "params": {"max_time": 0.1, "od": True},
        "unw": 1,
        "w": 1,
    },
]


def show_graph_info(graph, starts, goals):
    print(f"{graph.__class__.__name__}(**{graph.to_dict()})")
    print(f"starts, goals = {starts}, {goals}")


def find_path(finder, starts, goals, **kwargs):
    t = time.time()
    try:
        paths = finder.mapf(starts, goals, **kwargs)
    except RuntimeError:
        paths = []
    return paths, time.time() - t


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


def check_solution(paths, goals):
    if len(paths) != len(goals):
        return False

    for path, goal in zip(paths, goals):
        if not path or path[-1] != goal:
            return False

    return True


def compare_results(results):
    solved = [x for x in results if x["solved"]]
    if not solved:
        return True

    best_result = min(x["cost"] for x in solved)
    for x in solved:
        if x["optimal"] and abs(x["cost"] - best_result) > 0.001:
            return False

    return True


def run_graph(algorithms, graph, starts, goals):
    results = []
    for a in algorithms:
        params = a.get("params", {})
        paths, time = find_path(a["finder"], starts, goals, **params)

        total_cost = 0
        for path in paths:
            path_cost = graph.calculate_cost(path)
            if path_cost == -1:
                show_graph_info(graph, starts, goals)
                print(f"Error: algorithm {a['name']} returns the wrong path {path}")
                return False
            total_cost += path_cost

        a["total_time"] += time

        solved = False
        if paths:
            if not check_solution(paths, goals):
                print(f"Error: algorithm {a['name']} returns not a solution: {paths}")
                return False
            solved = True
        a["num_solved"] += solved

        if not check_paths(graph, paths):
            show_graph_info(graph, starts, goals)
            print(f"Error: algorithm {a['name']} returns paths with collisions")
            return False

        if solved:
            a["total_cost"] += total_cost

        results.append(
            {
                "algorithm": a["name"],
                "paths": paths,
                "solved": solved,
                "cost": total_cost,
                "optimal": a.get("optimal", False),
            }
        )

    if not compare_results(results):
        show_graph_info(graph, starts, goals)
        print("Error: ")
        for r in results:
            print(f" - {r['algorithm']} : cost = {r['cost']}, paths = {r['paths']}")
        return False

    return True


def create_graph_with_queries(generator):
    while True:
        graph = generator.instance()
        try:
            queries = random_queries(
                graph, num_queries=NUM_AGENTS, unique=True, connected=True
            )
        except ValueError:
            continue

        starts, goals = zip(*queries)
        return graph, starts, goals


def stress_test(weighted=False, edge_collision=True):

    algorithms = copy(ALGORITHMS)

    if not weighted:
        print(f"\nStress test with unweighted grid...")
        generator = UNWEIGHTED_GRID_GENERATOR
        optimal_flag = "unw"
    else:
        print(f"\nStress test with weighted grid...")
        generator = WEIGHTED_GRID_GENERATOR
        optimal_flag = "w"

    for a in algorithms:
        a["optimal"] = a[optimal_flag]
        a["total_time"] = 0
        a["total_cost"] = 0
        a["num_solved"] = 0

    for i in range(NUM_GRAPHS):
        print(f"run {i + 1}/{NUM_GRAPHS}", end="\r")

        graph, starts, goals = create_graph_with_queries(generator)
        graph.edge_collision = edge_collision
        for a in algorithms:
            a["finder"] = a["class"](graph)

        r = run_graph(algorithms, graph, starts, goals)
        if not r:
            return

    print("\nOverall results:")
    for a in algorithms:
        mean_time = a["total_time"] / NUM_GRAPHS
        mean_cost = a["total_cost"] / a["num_solved"] if a["num_solved"] > 0 else 0
        solved = a["num_solved"] / NUM_GRAPHS * 100
        print(f" - {a['name']}:")
        print(
            f"     mean time = {1000 * mean_time:.3f}ms, mean cost = {mean_cost:.2f}, "
            f"solved {solved:.1f}%"
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--weighted",
        "-w",
        default=False,
        action=argparse.BooleanOptionalAction,
        help="generate weighted graphs",
    )
    parser.add_argument(
        "--edge_collision",
        "-e",
        default=False,
        action=argparse.BooleanOptionalAction,
        help="enable edge collisions",
    )
    flags = parser.parse_args()
    print(flags)

    stress_test(
        weighted=flags.weighted,
        edge_collision=flags.edge_collision,
    )
