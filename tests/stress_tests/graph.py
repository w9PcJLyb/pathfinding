import time
import random
from copy import copy

from w9_pathfinding import Graph, DFS, BFS, BiBFS, Dijkstra, BiDijkstra


NUM_GRAPHS = 100
NUM_QUERIES_PER_GRAPH = 10
NUM_VERTICES = 2000
BRANCHING_FACTOR = 10

# - unweighted - can find the shortest path in an unweighted graph
# - weighted - can find the shortest path in a weighted graph
ALGORITHMS = [
    {"name": "DFS", "class": DFS, "unweighted": False, "weighted": False},
    {"name": "BFS", "class": BFS, "unweighted": True, "weighted": False},
    {"name": "BiBFS", "class": BiBFS, "unweighted": True, "weighted": False},
    {"name": "Dijkstra", "class": Dijkstra, "unweighted": True, "weighted": True},
    {"name": "BiDijkstra", "class": BiDijkstra, "unweighted": True, "weighted": True},
]


def create_random_graph(num_vertices, branching_factor, weighted=True):
    graph = Graph(num_vertices)

    edges = []
    num_edges = int(num_vertices * branching_factor)
    for _ in range(num_edges):
        start = random.randint(0, num_vertices-1)
        end = random.randint(0, num_vertices-1)
        if weighted:
            cost = random.random() * 100
        else:
            cost = 1
        edges.append([start, end, cost])

    graph.add_edges(edges)

    return graph


def find_path(finder, start, end):
    t = time.time()
    path = finder.find_path(start, end)
    return path, time.time() - t


def calculate_cost(graph, path, start, end):
    if not path:
        return 0
    
    if path[0] != start or path[-1] != end:
        return -1

    total_cost = 0
    for i in range(len(path) - 1):
        p = path[i]
        next_p = path[i + 1]

        costs = [c for x, c in graph.get_neighbours(p) if x == next_p]
        if not costs:
            return -1

        total_cost += min(costs)

    return total_cost


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

        path_cost = calculate_cost(graph, path, start, end)
        if path_cost == -1:
            print(f"Error algorithm {a['name']} return the wrong path {path}")
            show_graph_info(graph, start, end)
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
        print(compare_results(results))
        print("Error, different results: ")
        for r in results:
            print(f" - {r['algorithm']} : cost = {r['cost']}, path = {r['path']}")
        show_graph_info(graph, start, end)
        return False
        
    return True


def stress_test(algrithms, weighted):

    for a in algrithms:
        a["total_time"] = 0
        a["total_cost"] = 0

    for i in range(NUM_GRAPHS):
        print(f"run {i + 1}/{NUM_GRAPHS}", end="\r")

        graph = create_random_graph(
            num_vertices=NUM_VERTICES,
            branching_factor=BRANCHING_FACTOR,
            weighted=weighted,
        )

        for a in algrithms:
            a["finder"] = a["class"](graph)

        for _ in range(NUM_QUERIES_PER_GRAPH):
            start = random.randint(0, graph.num_vertices - 1)
            end = random.randint(0, graph.num_vertices - 1)

            r = run_graph(algrithms, graph, start, end)
            if not r:
                return

    print("\nOverall results:")
    count = NUM_GRAPHS * NUM_QUERIES_PER_GRAPH
    for a in algrithms:
        mean_time = a['total_time'] / count
        mean_cost = a['total_cost'] / count
        print(f" - {a['name']}:")
        print(f"     mean time = {1000 * mean_time:.3f}ms, mean cost = {mean_cost:.2f}")


def test_unweighted_graph():
    print("\nStress test with unweighted graph...")

    algrithms = copy(ALGORITHMS)
    for a in algrithms:
        a["shortest_path"] = a["unweighted"]
    
    stress_test(algrithms, weighted=False)


def test_weighted_graph():
    print("\nStress test with weighted graph...")

    algrithms = copy(ALGORITHMS)
    for a in algrithms:
        a["shortest_path"] = a["weighted"]
    
    stress_test(algrithms, weighted=True)


if __name__ == "__main__":
    test_unweighted_graph()
    test_weighted_graph()
