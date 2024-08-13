from copy import copy

import w9_pathfinding as pf
from tests.stress_tests.utils import run_graph
from tests.stress_tests.random_instance import (
    GraphGenerator,
    GraphWithCoordinatesGenerator,
    random_queries,
)

NUM_GRAPHS = 100
NUM_QUERIES_PER_GRAPH = 10

UNWEIGHTED_GRAPH_GENERATOR = GraphGenerator(
    num_vertices=5000,
    branching_factor=3,
    weighted=False,
    bidirectional=False,
)

WEIGHTED_GRAPH_GENERATOR = GraphGenerator(
    num_vertices=5000,
    branching_factor=3,
    weighted=True,
    bidirectional=False,
    max_weight=1000,
)

GRAPH_WITH_COORDINATES_GENERATOR = GraphWithCoordinatesGenerator(
    num_vertices=5000,
    branching_factor=3,
    bidirectional=False,
    num_dimensions=3,
    min_x=-1000,
    max_x=1000,
)

# - unw - can find the shortest path in an unweighted graph
# - w - can find the shortest path in a weighted graph
# - h - heuristic algorithm
ALGORITHMS = [
    {"name": "DFS", "class": pf.DFS, "unw": 0, "w": 0, "h": 0},
    {"name": "GBS", "class": pf.GBS, "unw": 0, "w": 0, "h": 1},
    {"name": "BFS", "class": pf.BFS, "unw": 1, "w": 0, "h": 0},
    {"name": "BiBFS", "class": pf.BiBFS, "unw": 1, "w": 0, "h": 0},
    {"name": "Dijkstra", "class": pf.Dijkstra, "unw": 1, "w": 1, "h": 0},
    {"name": "BiDijkstra", "class": pf.BiDijkstra, "unw": 1, "w": 1, "h": 0},
    {"name": "A*", "class": pf.AStar, "unw": 1, "w": 1, "h": 1},
    {"name": "Bi A*", "class": pf.BiAStar, "unw": 1, "w": 1, "h": 1},
    {
        "name": "Space-Time A*",
        "class": pf.SpaceTimeAStar,
        "unw": 1,
        "w": 1,
        "h": 0,
        "params": {"search_depth": 10**5},
    },
]


def stress_test(weighted, with_coordinates=False):

    algorithms = copy(ALGORITHMS)

    if not weighted:
        print(f"\nStress test with unweighted graph...")
        generator = UNWEIGHTED_GRAPH_GENERATOR
        shortest_path_flag = "unw"
    elif not with_coordinates:
        print(f"\nStress test with weighted graph...")
        generator = WEIGHTED_GRAPH_GENERATOR
        shortest_path_flag = "w"
    else:
        print(f"\nStress test graph with coordinates...")
        generator = GRAPH_WITH_COORDINATES_GENERATOR
        shortest_path_flag = "w"

    if not with_coordinates:
        algorithms = [a for a in algorithms if not a["h"]]

    for a in algorithms:
        a["shortest_path"] = a[shortest_path_flag]
        a["total_time"] = 0
        a["total_cost"] = 0

    for i in range(NUM_GRAPHS):
        print(f"run {i + 1}/{NUM_GRAPHS}", end="\r")

        graph = generator.instance()
        queries = random_queries(graph, num_queries=NUM_QUERIES_PER_GRAPH)

        for a in algorithms:
            a["finder"] = a["class"](graph)

        for start, end in queries:
            r = run_graph(algorithms, graph, start, end)
            if not r:
                return

    print("\nOverall results:")
    count = NUM_GRAPHS * NUM_QUERIES_PER_GRAPH
    for a in algorithms:
        mean_time = a["total_time"] / count
        mean_cost = a["total_cost"] / count
        print(f" - {a['name']}:")
        print(f"     mean time = {1000 * mean_time:.3f}ms, mean cost = {mean_cost:.2f}")


if __name__ == "__main__":
    stress_test(weighted=False)
    stress_test(weighted=True)
    stress_test(weighted=True, with_coordinates=True)
