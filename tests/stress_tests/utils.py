import time


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


def find_path(finder, start, end):
    t = time.time()
    path = finder.find_path(start, end)
    return path, time.time() - t


def show_graph_info(graph, start, end):
    print(f"{graph.__class__.__name__}(**{graph.to_dict()})")
    print(f"start, end = {start}, {end}")


def run_graph(algorithms, graph, start, end):
    results = []
    for a in algorithms:
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
