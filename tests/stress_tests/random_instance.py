import random
from w9_pathfinding.envs import Grid, Graph, Grid3D, HexGrid, HexLayout


def random_queries(graph, num_queries, connected=False, unique=False):

    free_nodes = []

    if isinstance(graph, Graph):
        free_nodes = list(range(graph.size))
    elif isinstance(graph, (Grid, HexGrid)):
        obstacle_map = graph.obstacle_map
        for x in range(graph.width):
            for y in range(graph.height):
                if not obstacle_map[y][x]:
                    free_nodes.append((x, y))
    elif isinstance(graph, Grid3D):
        obstacle_map = graph.obstacle_map
        for x in range(graph.width):
            for y in range(graph.height):
                for z in range(graph.depth):
                    if not obstacle_map[z][y][x]:
                        free_nodes.append((x, y, z))

    if unique and len(free_nodes) < num_queries:
        raise ValueError(f"Can't generate {num_queries} unique queries")

    start_pool = list(free_nodes)
    goal_pool = list(free_nodes)

    components = []
    if connected:
        components = graph.find_components()
        components = [set(c) for c in components]

    queries = []
    while len(queries) < num_queries:
        start = random.choice(start_pool)
        goal = random.choice(goal_pool)

        if connected:
            is_connected = False
            for c in components:
                if start in c:
                    if goal in c:
                        is_connected = True
                    break

            if not is_connected:
                continue

        if unique:
            start_pool = [x for x in start_pool if x != start]
            goal_pool = [x for x in goal_pool if x != goal]

        queries.append((start, goal))

    return queries
