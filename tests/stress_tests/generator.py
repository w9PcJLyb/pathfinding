import random
from w9_pathfinding import Grid, Graph


class _Generator:
    def generate(self, num_queries=1):
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

    def generate(self, num_queries=1):
        graph = self._generate_graph()

        queries = []
        for _ in range(num_queries):
            start = random.randint(0, graph.num_vertices - 1)
            end = random.randint(0, graph.num_vertices - 1)
            queries.append((start, end))

        return graph, queries


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


class GridGrnerator(_Generator):
    def __init__(
        self,
        width=10,
        height=10,
        obstacle_percentage=0.2,
        weighted=False,
        min_weight=0,
        max_weight=100,
    ):
        self.width = width
        self.height = height
        self.obstacle_percentage = obstacle_percentage
        self.weighted = weighted
        self.min_weight = min_weight
        self.max_weight = max_weight

    def _generate_obstacle_map(self):
        obstacle_map = []
        for _ in range(self.height):
            row = [
                random.random() < self.obstacle_percentage for _ in range(self.width)
            ]
            obstacle_map.append(row)
        return obstacle_map

    def _generate_grid(self):
        grid = Grid(
            self._generate_obstacle_map(),
            diagonal_movement=random.randint(0, 3),
            passable_left_right_border=random.randint(0, 1),
            passable_up_down_border=random.randint(0, 1),
        )
        self._add_widths(grid)
        return grid

    def _add_widths(self, grid):
        if not self.weighted:
            return

        grid.diagonal_movement_cost_multiplier = random.uniform(1, 2)

        obstacle_map = grid.obstacle_map

        weights = []
        for y in range(grid.height):
            row = []
            for x in range(grid.width):
                if obstacle_map[y][x]:
                    row.append(-1)
                else:
                    row.append(random.uniform(self.min_weight, self.max_weight))
            weights.append(row)

        grid.weights = weights

    def _find_free_point(self, grid):
        while True:
            x = random.randint(0, grid.width - 1)
            y = random.randint(0, grid.height - 1)
            if not grid.has_obstacle(x, y):
                return x, y

    def generate(self, num_queries=1):
        grid = self._generate_grid()

        queries = []
        for _ in range(num_queries):
            start = self._find_free_point(grid)
            end = self._find_free_point(grid)
            queries.append((start, end))

        return grid, queries
