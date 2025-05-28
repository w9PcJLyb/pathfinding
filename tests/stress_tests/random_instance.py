import random
from w9_pathfinding.envs import Grid, Graph, Grid3D, HexGrid, HexLayout


class _Generator:
    def instance(self):
        raise NotImplementedError()


class GraphGenerator(_Generator):

    def __init__(
        self,
        num_vertices=10,
        branching_factor=3,
        weighted=False,
        max_weight=100,
        bidirectional=False,
    ):
        self.num_vertices = num_vertices
        self.branching_factor = branching_factor
        self.weighted = weighted
        self.max_weight = max_weight
        self.bidirectional = bidirectional

    def _generate_graph(self):
        graph = Graph(self.num_vertices)
        self._add_edges(graph)
        return graph

    def _add_edges(self, graph):
        edges = []
        num_edges = int(self.num_vertices * self.branching_factor)
        for _ in range(num_edges):
            start = random.randint(0, graph.num_vertices - 1)
            end = random.randint(0, graph.num_vertices - 1)
            if self.weighted:
                cost = random.random() * self.max_weight
            else:
                cost = 1
            edges.append([start, end, cost])
            if self.bidirectional:
                edges.append([end, start, cost])
        graph.add_edges(edges)

    def instance(self):
        return self._generate_graph()


class GraphWithCoordinatesGenerator(GraphGenerator):

    def __init__(
        self,
        num_vertices=10,
        branching_factor=3,
        bidirectional=False,
        num_dimensions=3,
        min_x=-100,
        max_x=100,
    ):
        self.num_vertices = num_vertices
        self.branching_factor = branching_factor
        self.bidirectional = bidirectional
        self.num_dimensions = num_dimensions
        self.min_x = min_x
        self.max_x = max_x

    def _generate_graph(self):
        graph = Graph(self.num_vertices)
        coordinates = []
        for _ in range(self.num_vertices):
            c = [
                random.uniform(self.min_x, self.max_x)
                for _ in range(self.num_dimensions)
            ]
            coordinates.append(c)

        graph.set_coordinates(coordinates)
        self._add_edges(graph)
        return graph

    def _add_edges(self, graph):
        edges = []
        num_edges = int(self.num_vertices * self.branching_factor)

        for _ in range(num_edges):
            start = random.randint(0, graph.num_vertices - 1)
            end = random.randint(0, graph.num_vertices - 1)

            min_distance = graph.estimate_distance(start, end)
            if min_distance == 0:
                cost = random.uniform(0, abs(self.max_x - self.min_x))
            else:
                cost = min_distance * random.uniform(1, 1.5)

            edges.append([start, end, cost])
            if self.bidirectional:
                edges.append([end, start, cost])

        graph.add_edges(edges)


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
