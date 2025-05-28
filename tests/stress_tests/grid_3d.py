from copy import copy

from tests.factory import Grid3DFactory, RANDOM
from tests.stress_tests.utils import run_graph
from tests.stress_tests.random_instance import random_queries
from tests.stress_tests.grid import ALGORITHMS

NUM_GRAPHS = 100
NUM_QUERIES_PER_GRAPH = 10

UNWEIGHTED_GRID_GENERATOR = Grid3DFactory(
    width=20,
    height=20,
    depth=20,
    obstacle_ratio=0.2,
    weighted=False,
    passable_borders=RANDOM,
    random_seed=42,
)

WEIGHTED_GRID_GENERATOR = Grid3DFactory(
    width=20,
    height=20,
    depth=20,
    obstacle_ratio=0.2,
    weighted=True,
    min_weight=0.5,
    max_weight=1.5,
    passable_borders=RANDOM,
    random_seed=42,
)


def stress_test(weighted):

    algorithms = copy(ALGORITHMS)

    if not weighted:
        print(f"\nStress test with unweighted grid...")
        factory = UNWEIGHTED_GRID_GENERATOR
        shortest_path_flag = "unw"
    else:
        print(f"\nStress test with weighted grid...")
        factory = WEIGHTED_GRID_GENERATOR
        shortest_path_flag = "w"

    for a in algorithms:
        a["shortest_path"] = a[shortest_path_flag]
        a["total_time"] = 0
        a["total_cost"] = 0

    for i in range(NUM_GRAPHS):
        print(f"run {i + 1}/{NUM_GRAPHS}", end="\r")

        graph = factory()
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
