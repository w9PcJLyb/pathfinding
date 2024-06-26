import time
from copy import copy

import w9_pathfinding as pf
from tests.stress_tests.generator import GridGrnerator

NUM_GRAPHS = 100
NUM_QUERIES_PER_GRAPH = 10

UNWEIGHTED_GRID_GENERATOR = GridGrnerator(
    width=100,
    height=100,
    obstacle_percentage=0.2,
    weighted=False,
)

WEIGHTED_GRID_GENERATOR = GridGrnerator(
    width=100,
    height=100,
    obstacle_percentage=0.2,
    weighted=True,
)

# - unweighted - can find the shortest path in an unweighted graph
# - weighted - can find the shortest path in a weighted graph
ALGORITHMS = [
    {"name": "DFS", "class": pf.DFS, "unw": 0, "w": 0},
    {"name": "BFS", "class": pf.BFS, "unw": 1, "w": 0},
    {"name": "BiBFS", "class": pf.BiBFS, "unw": 1, "w": 0},
    {"name": "Dijkstra", "class": pf.Dijkstra, "unw": 1, "w": 1},
    {"name": "BiDijkstra", "class": pf.BiDijkstra, "unw": 1, "w": 1},
    {"name": "A*", "class": pf.AStar, "unw": 1, "w": 1},
    {"name": "Bi A*", "class": pf.BiAStar, "unw": 1, "w": 1},
]


def find_path(finder, start, end):
    t = time.time()
    path = finder.find_path(start, end)
    return path, time.time() - t


def show_grid_info(grid, start, end):
    print("obstacle_map =", grid.obstacle_map)
    print("diagonal_movement =", grid.diagonal_movement)
    print("passable_left_right_border =", grid.passable_left_right_border)
    print("passable_up_down_border =", grid.passable_up_down_border)
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


def run_grid(algrithms, graph, start, end):
    results = []
    for a in algrithms:
        path, time = find_path(a["finder"], start, end)
        a["total_time"] += time

        path_cost = graph.calculate_cost(path)
        if path_cost == -1:
            show_grid_info(graph, start, end)
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
        show_grid_info(graph, start, end)
        print("Error, different results: ")
        for r in results:
            print(f" - {r['algorithm']} : cost = {r['cost']}, path = {r['path']}")
        return False

    return True


def stress_test(weighted):

    algrithms = copy(ALGORITHMS)

    if not weighted:
        print(f"\nStress test with unweighted grid...")
        generator = UNWEIGHTED_GRID_GENERATOR
        shortest_path_flag = "unw"
    else:
        print(f"\nStress test with weighted grid...")
        generator = WEIGHTED_GRID_GENERATOR
        shortest_path_flag = "w"

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
            r = run_grid(algrithms, graph, start, end)
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
