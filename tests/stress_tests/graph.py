import time
from copy import copy

import w9_pathfinding as pf
from tests.stress_tests.generator import GraphGenerator, GraphWithCoordinatesGenerator

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
    {"name": "BFS", "class": pf.BFS, "unw": 1, "w": 0, "h": 0},
    {"name": "BiBFS", "class": pf.BiBFS, "unw": 1, "w": 0, "h": 0},
    {"name": "Dijkstra", "class": pf.Dijkstra, "unw": 1, "w": 1, "h": 0},
    {"name": "BiDijkstra", "class": pf.BiDijkstra, "unw": 1, "w": 1, "h": 0},
    {"name": "A*", "class": pf.AStar, "unw": 1, "w": 1, "h": 1},
    {"name": "Bi A*", "class": pf.BiAStar, "unw": 1, "w": 1, "h": 1},
]


def find_path(finder, start, end):
    t = time.time()
    path = finder.find_path(start, end)
    return path, time.time() - t


def show_graph_info(graph, start, end):
    print("num_vertices =", graph.num_vertices)
    print("edges: [")
    for x in sorted(graph.edges):
        print(f"    {x},")
    print("]")
    print(f"start, end = {start}, {end}")


def compare_results(results):
    # If one cant find a path, no one should find
    without_path = [x for x in results if x["path"] == []]
    if without_path and len(without_path) != len(results):
        return False

    # all algorithms that should find the shortest path must find it.
    best_result = min(x["cost"] for x in results)
    for x in results:
        if x["shortest_path"] and abs(x["cost"] - best_result) > 0.001:
            return False

    return True


def run_graph(algrithms, graph, start, end):
    results = []
    for a in algrithms:
        path, time = find_path(a["finder"], start, end)
        a["total_time"] += time

        path_cost = graph.calculate_cost(path)
        if path_cost == -1:
            show_graph_info(graph, start, end)
            print(f"Error algorithm {a['name']} return the wrong path {path}")
            return False

        a["total_cost"] += path_cost

        results.append(
            {
                "algorithm": a["name"],
                "path": path,
                "cost": path_cost,
                "shortest_path": a["shortest_path"],
            }
        )

    if not compare_results(results):
        show_graph_info(graph, start, end)
        print("Error, different results: ")
        for r in results:
            print(f" - {r['algorithm']} : cost = {r['cost']}, path = {r['path']}")
        return False

    return True


def stress_test(weighted, with_coordinates=False):

    algrithms = copy(ALGORITHMS)

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
        algrithms = [a for a in algrithms if not a["h"]]

    for a in algrithms:
        a["shortest_path"] = a[shortest_path_flag]
        a["total_time"] = 0
        a["total_cost"] = 0

    for i in range(NUM_GRAPHS):
        print(f"run {i + 1}/{NUM_GRAPHS}", end="\r")

        graph, queries = generator.generate(num_queries=NUM_QUERIES_PER_GRAPH)

        for a in algrithms:
            a["finder"] = a["class"](graph)

        for start, end in queries:
            r = run_graph(algrithms, graph, start, end)
            if not r:
                return

    print("\nOverall results:")
    count = NUM_GRAPHS * NUM_QUERIES_PER_GRAPH
    for a in algrithms:
        mean_time = a["total_time"] / count
        mean_cost = a["total_cost"] / count
        print(f" - {a['name']}:")
        print(f"     mean time = {1000 * mean_time:.3f}ms, mean cost = {mean_cost:.2f}")


if __name__ == "__main__":
    stress_test(weighted=False)
    stress_test(weighted=True)
    stress_test(weighted=True, with_coordinates=True)
