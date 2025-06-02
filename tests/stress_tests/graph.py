from copy import copy

from w9_pathfinding import pf, mapf
from tests.factory import GraphFactory, SpatialGraphFactory, QueryGenerator
from tests.stress_tests.utils import run_graph

NUM_GRAPHS = 100
NUM_QUERIES_PER_GRAPH = 10

UNWEIGHTED_GRAPH_FACTORY = GraphFactory(
    num_vertices=1000,
    branching_factor=3,
    weighted=False,
    directed=True,
    random_seed=42,
)

WEIGHTED_GRAPH_FACTORY = GraphFactory(
    num_vertices=1000,
    branching_factor=3,
    directed=True,
    weighted=True,
    min_weight=0.5,
    max_weight=10,
    random_seed=42,
)

GRAPH_WITH_COORDINATES_FACTORY = SpatialGraphFactory(
    num_vertices=1000,
    branching_factor=3,
    directed=True,
    num_dimensions=3,
    random_seed=42,
)

QUERY_GENERATOR = QueryGenerator(random_seed=9)

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
        "class": mapf.SpaceTimeAStar,
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
        factory = UNWEIGHTED_GRAPH_FACTORY
        shortest_path_flag = "unw"
    elif not with_coordinates:
        print(f"\nStress test with weighted graph...")
        factory = WEIGHTED_GRAPH_FACTORY
        shortest_path_flag = "w"
    else:
        print(f"\nStress test graph with coordinates...")
        factory = GRAPH_WITH_COORDINATES_FACTORY
        shortest_path_flag = "w"

    if not with_coordinates:
        algorithms = [a for a in algorithms if not a["h"]]

    for a in algorithms:
        a["shortest_path"] = a[shortest_path_flag]
        a["total_time"] = 0
        a["total_cost"] = 0

    for i in range(NUM_GRAPHS):
        print(f"run {i + 1}/{NUM_GRAPHS}", end="\r")

        graph = factory()
        queries = QUERY_GENERATOR.generate_queries(graph, NUM_QUERIES_PER_GRAPH)

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
