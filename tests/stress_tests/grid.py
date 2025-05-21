from copy import copy

from w9_pathfinding import pf
from tests.stress_tests.utils import run_graph
from tests.stress_tests.random_instance import GridGenerator, random_queries

NUM_GRAPHS = 100
NUM_QUERIES_PER_GRAPH = 10

UNWEIGHTED_GRID_GENERATOR = GridGenerator(
    width=64,
    height=64,
    obstacle_percentage=0.2,
    weighted=False,
)

WEIGHTED_GRID_GENERATOR = GridGenerator(
    width=64,
    height=64,
    obstacle_percentage=0.2,
    weighted=True,
    min_weight=0.5,
    max_weight=1.5,
)

# - unweighted - can find the shortest path in an unweighted graph
# - weighted - can find the shortest path in a weighted graph
ALGORITHMS = [
    {"name": "DFS", "class": pf.DFS, "unw": 0, "w": 0},
    {"name": "GBS", "class": pf.GBS, "unw": 0, "w": 0},
    {"name": "BFS", "class": pf.BFS, "unw": 1, "w": 0},
    {"name": "BiBFS", "class": pf.BiBFS, "unw": 1, "w": 0},
    {"name": "Dijkstra", "class": pf.Dijkstra, "unw": 1, "w": 1},
    {"name": "BiDijkstra", "class": pf.BiDijkstra, "unw": 1, "w": 1},
    {"name": "A*", "class": pf.AStar, "unw": 1, "w": 1},
    {"name": "Bi A*", "class": pf.BiAStar, "unw": 1, "w": 1},
    {
        "name": "Space-Time A*",
        "class": pf.SpaceTimeAStar,
        "unw": 1,
        "w": 1,
        "params": {"search_depth": 10**5},
    },
]


def stress_test(weighted):

    algorithms = copy(ALGORITHMS)

    if not weighted:
        print(f"\nStress test with unweighted grid...")
        generator = UNWEIGHTED_GRID_GENERATOR
        shortest_path_flag = "unw"
    else:
        print(f"\nStress test with weighted grid...")
        generator = WEIGHTED_GRID_GENERATOR
        shortest_path_flag = "w"

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
