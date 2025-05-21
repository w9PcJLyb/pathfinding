import time
import argparse
from copy import copy
from collections import defaultdict

from w9_pathfinding import mapf
from w9_pathfinding.graph import ReservationTable
from w9_pathfinding.pf import SpaceTimeAStar
from tests.stress_tests.utils import run_graph
from tests.stress_tests.random_instance import GridGenerator, random_queries

GRID_SIZE = 8
NUM_GRAPHS = 100
NUM_AGENTS = 5
WEIGHTED = False
EDGE_COLLISION = True
OBSTACLE_PERCENTAGE = 0.2
NUM_DYNAMIC_OBSTACLES = 1
MAX_TIME = 0.1  # time limit in seconds
MAX_LENGTH = 100  # maximum agent path length

kw = {"weighted": False}
if WEIGHTED:
    kw = {"weighted": True, "min_weight": 1, "max_weight": 5}

GRID_GENERATOR = GridGenerator(
    width=GRID_SIZE,
    height=GRID_SIZE,
    obstacle_percentage=OBSTACLE_PERCENTAGE,
    **kw,
)

ALGORITHMS = [
    {"name": "HCA*", "class": mapf.HCAStar, "unw": 0, "w": 0},
    {"name": "WHCA*", "class": mapf.WHCAStar, "unw": 0, "w": 0},
    {
        "name": "CBS (disjoint_splitting=False)",
        "class": mapf.CBS,
        "params": {"disjoint_splitting": False},
        "unw": 1,
        "w": 1,
    },
    {
        "name": "CBS (disjoint_splitting=True)",
        "class": mapf.CBS,
        "params": {"disjoint_splitting": True},
        "unw": 1,
        "w": 1,
    },
    {
        "name": "ICTS(ict_pruning=False)",
        "class": mapf.ICTS,
        "params": {"ict_pruning": False},
        "unw": 1,
        "w": 0,
    },
    {
        "name": "ICTS(ict_pruning=True)",
        "class": mapf.ICTS,
        "params": {"ict_pruning": True},
        "unw": 1,
        "w": 0,
    },
    {
        "name": "A*(od=False)",
        "class": mapf.MultiAgentAStar,
        "params": {"operator_decomposition": False},
        "unw": 1,
        "w": 1,
    },
    {
        "name": "A*(od=True)",
        "class": mapf.MultiAgentAStar,
        "params": {"operator_decomposition": True},
        "unw": 1,
        "w": 1,
    },
]

for a in ALGORITHMS:
    if "params" not in a:
        a["params"] = {}

    if a["name"] not in ("HCA*", "WHCA*"):
        a["params"]["max_time"] = MAX_TIME

    a["params"]["max_length"] = MAX_LENGTH


def show_graph_info(graph, starts, goals, reserved_paths=None):
    print(f"grid = {graph.__class__.__name__}(**{graph.to_dict()})")
    print(f"starts, goals = {starts}, {goals}")
    print(f"reserved_paths = {reserved_paths}")


def find_path(finder, starts, goals, **kwargs):
    t = time.time()
    try:
        paths = finder.mapf(starts, goals, **kwargs)
    except RuntimeError:
        paths = []
    return paths, time.time() - t


def has_collision(graph, paths, print_info=False):
    if len(paths) < 2:
        return False

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
                if print_info:
                    print(f"Collision at {time}: node = {p}, agents = {agent_ids}")
                return True

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
                    if print_info:
                        print(
                            f"Collision at {time}: edge = {edge}, agents = {agent_ids}"
                        )
                    return True

        time += 1

    return False


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


def run_graph(algorithms, graph, starts, goals, reserved_paths):
    rt = None
    if reserved_paths:
        rt = ReservationTable(graph)
        for path in reserved_paths:
            rt.add_path(path, reserve_destination=True)

    results = []
    for a in algorithms:
        params = a.get("params", {})
        params["reservation_table"] = rt
        try:
            paths, time = find_path(a["finder"], starts, goals, **params)
        except Exception as e:
            show_graph_info(graph, starts, goals, reserved_paths)
            print(f"Error: algorithm {a['name']}:")
            print(e)
            return False

        total_cost = 0
        for path in paths:
            path_cost = graph.calculate_cost(path)
            if path_cost == -1:
                show_graph_info(graph, starts, goals, reserved_paths)
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

        if has_collision(graph, paths + reserved_paths, print_info=True):
            show_graph_info(graph, starts, goals, reserved_paths)
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
        show_graph_info(graph, starts, goals, reserved_paths)
        print("Error: ")
        for r in results:
            print(f" - {r['algorithm']} : cost = {r['cost']}, paths = {r['paths']}")
        return False

    return True


def create_graph_with_queries():
    while True:
        graph = GRID_GENERATOR.instance()
        graph.edge_collision = EDGE_COLLISION
        try:
            queries = random_queries(
                graph,
                num_queries=NUM_AGENTS + NUM_DYNAMIC_OBSTACLES,
                unique=True,
                connected=True,
            )
        except ValueError:
            continue

        reserved_paths = []
        if NUM_DYNAMIC_OBSTACLES > 0:
            astar = SpaceTimeAStar(graph)
            rt = ReservationTable(graph)
            build = True
            for start, goal in queries[:NUM_DYNAMIC_OBSTACLES]:
                reserved_path = astar.find_path_with_length_limit(
                    start, goal, max_length=GRID_SIZE * 2, reservation_table=rt
                )
                if reserved_path:
                    reserved_paths.append(reserved_path)
                    rt.add_path(reserved_path, reserve_destination=True)
                else:
                    build = False
                    break

            if not build or has_collision(graph, reserved_paths):
                continue

        starts, goals = zip(*queries[NUM_DYNAMIC_OBSTACLES:])
        return graph, starts, goals, reserved_paths


def stress_test():
    algorithms = copy(ALGORITHMS)

    if not WEIGHTED:
        optimal_flag = "unw"
    else:
        optimal_flag = "w"

    for a in algorithms:
        a["optimal"] = a[optimal_flag]
        a["total_time"] = 0
        a["total_cost"] = 0
        a["num_solved"] = 0

    for i in range(NUM_GRAPHS):
        print(f"run {i + 1}/{NUM_GRAPHS}", end="\r")

        graph, starts, goals, reserved_paths = create_graph_with_queries()
        for a in algorithms:
            a["finder"] = a["class"](graph)

        r = run_graph(algorithms, graph, starts, goals, reserved_paths)
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
    stress_test()
