import time
import argparse
from copy import copy
from collections import defaultdict

import w9_pathfinding as pf
from tests.stress_tests.utils import run_graph
from tests.stress_tests.random_instance import GridGenerator, random_queries

NUM_GRAPHS = 100
NUM_AGENTS = 15

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
    {"name": "HCA*", "class": pf.HCAStar},
    {"name": "WHCA*", "class": pf.WHCAStar},
]


def show_graph_info(grid, starts, goals):
    print("weights =", grid.weights)
    print("diagonal_movement =", grid.diagonal_movement)
    print("passable_left_right_border =", grid.passable_left_right_border)
    print("passable_up_down_border =", grid.passable_up_down_border)
    print(f"starts, goals = {starts}, {goals}")


def find_path(finder, starts, goals, **kwargs):
    t = time.time()
    paths = finder.mapf(starts, goals, **kwargs)
    return paths, time.time() - t


def check_paths(paths):
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

        time += 1

    return True


def is_solved(paths, goals, despawn_at_destination):
    if len(paths) != len(goals):
        return False

    if not goals:
        return True

    path_length = len(paths[0])

    for path, goal in zip(paths, goals):
        if not despawn_at_destination and len(path) != path_length:
            return False

        if not path or path[-1] != goal:
            return False

    return True


def run_graph(algorithms, graph, starts, goals, despawn_at_destination=False):
    for a in algorithms:
        paths, time = find_path(
            a["finder"], starts, goals, despawn_at_destination=despawn_at_destination
        )
        a["total_time"] += time

        solved = is_solved(paths, goals, despawn_at_destination)
        a["num_solved"] += solved

        if not check_paths(paths):
            show_graph_info(graph, starts, goals)
            print(f"Error: algorithm {a['name']} returns paths with collisions")
            return False

        total_cost = 0
        for path in paths:
            path_cost = graph.calculate_cost(path)
            if path_cost == -1:
                show_graph_info(graph, starts, goals)
                print(f"Error: algorithm {a['name']} returns the wrong path {path}")
                return False
            total_cost += path_cost

        if solved:
            a["total_cost"] += total_cost

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


def stress_test(weighted=False, despawn_at_destination=False):

    algorithms = copy(ALGORITHMS)

    if not weighted:
        print(f"\nStress test with unweighted grid...")
        generator = UNWEIGHTED_GRID_GENERATOR
    else:
        print(f"\nStress test with weighted grid...")
        generator = WEIGHTED_GRID_GENERATOR

    for a in algorithms:
        a["total_time"] = 0
        a["total_cost"] = 0
        a["num_solved"] = 0

    for i in range(NUM_GRAPHS):
        print(f"run {i + 1}/{NUM_GRAPHS}", end="\r")

        graph, starts, goals = create_graph_with_queries(generator)
        for a in algorithms:
            a["finder"] = a["class"](graph)

        r = run_graph(
            algorithms,
            graph,
            starts,
            goals,
            despawn_at_destination=despawn_at_destination,
        )
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
        "--despawn",
        "-d",
        action=argparse.BooleanOptionalAction,
        help="despawn an agent at destination",
    )
    flags = parser.parse_args()
    print(flags)

    stress_test(weighted=False, despawn_at_destination=flags.despawn)
    stress_test(weighted=True, despawn_at_destination=flags.despawn)
