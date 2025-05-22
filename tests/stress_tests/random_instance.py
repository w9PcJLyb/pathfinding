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


class GridGenerator(_Generator):
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
                -1 if random.random() < self.obstacle_percentage else 1
                for _ in range(self.width)
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

    def instance(self):
        return self._generate_grid()


class Grid3DGenerator(GridGenerator):
    def __init__(
        self,
        width=10,
        height=10,
        depth=10,
        obstacle_percentage=0.2,
        weighted=False,
        min_weight=0,
        max_weight=100,
    ):
        self.width = width
        self.height = height
        self.depth = depth
        self.obstacle_percentage = obstacle_percentage
        self.weighted = weighted
        self.min_weight = min_weight
        self.max_weight = max_weight

    def _generate_obstacle_map(self):
        obstacle_map = []
        for _ in range(self.depth):
            obstacle_map.append([])
            for _ in range(self.height):
                row = [
                    -1 if random.random() < self.obstacle_percentage else 1
                    for _ in range(self.width)
                ]
                obstacle_map[-1].append(row)
        return obstacle_map

    def _generate_grid(self):
        grid = Grid3D(
            self._generate_obstacle_map(),
            passable_borders=random.randint(0, 1),
        )
        self._add_widths(grid)
        return grid

    def _add_widths(self, grid):
        if not self.weighted:
            return

        weights = grid.weights
        for z in range(self.depth):
            for y in range(self.height):
                for x in range(self.width):
                    if weights[z][y][x] >= 0:
                        weights[z][y][x] = random.uniform(
                            self.min_weight, self.max_weight
                        )

        grid.weights = weights

    def _find_free_point(self, grid):
        while True:
            x = random.randint(0, grid.width - 1)
            y = random.randint(0, grid.height - 1)
            z = random.randint(0, grid.depth - 1)
            if not grid.has_obstacle((x, y, z)):
                return x, y, z


class HexGridGenerator(GridGenerator):
    def __init__(self, *args, layout=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.layout = layout

    def _generate_grid(self):

        layout = self.layout
        if layout is None:
            layout = random.randint(0, 3)

        layout = HexLayout(layout)
        passable_up_down_border = random.randint(0, 1)
        passable_left_right_border = random.randint(0, 1)

        if layout.is_pointy_top():
            if self.height % 2 == 1:
                passable_up_down_border = False
        elif layout.is_flat_top():
            if self.width % 2 == 1:
                passable_left_right_border = False

        grid = HexGrid(
            self._generate_obstacle_map(),
            layout=layout,
            passable_up_down_border=passable_up_down_border,
            passable_left_right_border=passable_left_right_border,
        )

        self._add_widths(grid)
        return grid

    def _add_widths(self, grid):
        if not self.weighted:
            return

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


def random_queries(graph, num_queries, connected=False, unique=False):
    is_grid = isinstance(graph, (Grid, Grid3D, HexGrid))

    free_nodes = []
    for node_id in range(graph.size):
        if is_grid and graph.has_obstacle(graph.get_coordinates(node_id)):
            continue
        free_nodes.append(node_id)

    if unique and len(free_nodes) < num_queries:
        raise ValueError(f"Can't generate {num_queries} unique queries")

    start_pool = list(free_nodes)
    goal_pool = list(free_nodes)

    components = []
    if connected:
        components = graph.find_components()
        if is_grid:
            components = [[graph.get_node_id(x) for x in c] for c in components]
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

        if is_grid:
            start, goal = graph.get_coordinates(start), graph.get_coordinates(goal)

        queries.append((start, goal))

    return queries
