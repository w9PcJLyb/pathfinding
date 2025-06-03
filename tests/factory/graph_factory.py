import numpy as np
from typing import Optional
from pydantic import Field, BaseModel, ConfigDict, model_validator

from w9_pathfinding.envs import Graph

RANDOM = "random"


class EnvFactory(BaseModel):
    random_seed: Optional[int] = None
    model_config = ConfigDict(extra="forbid")

    def __init__(self, **data):
        super().__init__(**data)
        self._rng = np.random.default_rng(self.random_seed)

    @model_validator(mode="after")
    def check_weights(self):
        min_weight = getattr(self, "min_weight", None)
        max_weight = getattr(self, "max_weight", None)
        if (
            min_weight is not None
            and max_weight is not None
            and min_weight > max_weight
        ):
            raise ValueError("min_weight must be less than or equal to max_weight.")

        return self

    def __call__(self):
        raise NotImplementedError()


class _GraphEnvFactory(EnvFactory):

    num_vertices: int = Field(..., gt=0)
    branching_factor: float = Field(..., ge=0.0)
    directed: bool = True
    allow_multiedges: bool = True
    allow_selfloops: bool = True

    @model_validator(mode="after")
    def validate_branching_factor(self):
        if not self.allow_multiedges:
            max_branching_factor = self.num_vertices - 1
            if self.allow_selfloops:
                max_branching_factor += 1
            if self.branching_factor > max_branching_factor:
                raise ValueError(
                    f"branching_factor ({self.branching_factor}) exceeds max possible "
                    f"outgoing edges per node ({max_branching_factor}) without multiedges."
                )

        return self

    def __call__(self) -> Graph:
        return Graph(
            num_vertices=self.num_vertices,
            directed=self.directed,
            edges=self._generate_edges(),
        )

    def _generate_edges(self):

        max_out_degree = self.num_vertices - 1
        if self.allow_selfloops:
            max_out_degree += 1

        lam = self.branching_factor
        if not self.directed:
            # For undirected graphs, each edge contributes
            # to the branching_factor of both connected nodes.
            lam = self.branching_factor / 2

        out_degrees = self._rng.poisson(lam=lam, size=self.num_vertices)
        if not self.allow_multiedges:
            out_degrees = np.clip(out_degrees, 0, max_out_degree)

        edges = []

        if self.directed or self.allow_multiedges:
            for node, out_degree in enumerate(out_degrees):
                ends = self._rng.choice(
                    max_out_degree, size=out_degree, replace=self.allow_multiedges
                )
                if self.allow_selfloops:
                    edges += [(node, e) for e in ends]
                else:
                    edges += [(node, e) if e < node else (node, e + 1) for e in ends]

        else:
            used_edges = [set() for _ in range(self.num_vertices)]
            for n1, out_degree in enumerate(out_degrees):
                n1_edges = used_edges[n1]

                free_edges = []
                for n2 in range(self.num_vertices):
                    if not self.allow_selfloops and n1 == n2:
                        continue
                    if n2 in n1_edges:
                        continue
                    free_edges.append(n2)

                ends = self._rng.choice(
                    free_edges, size=min(len(free_edges), out_degree), replace=False
                )

                for n2 in ends:
                    edges.append((n1, n2))
                    n1_edges.add(n2)
                    used_edges[n2].add(n1)

        return edges


class GraphFactory(_GraphEnvFactory):
    """
    A factory for generating random graphs.
    Note: The graph is not guaranteed to be fully connected.

    Parameters
    ----------
    num_vertices : int
        Number of nodes in the graph.

    branching_factor : float
        Average number of outgoing edges per node.

    weighted : bool, default=False
        If True, edges are assigned random weights in the range [min_weight, max_weight]

    min_weight : float, default=0.0
        Minimum edge weight when `weighted` is True

    max_weight : float, default=1.0
        Maximum edge weight when `weighted` is True

    directed : bool, default=True
        If True, creates a directed graph (each edge has a direction).
        If False, creates an undirected graph

    allow_multiedges : bool, default=True
        If True, multiple edges between the same pair of nodes are allowed

    allow_selfloops : bool, default=True
        allows edges to connect a vertex to itself

    random_seed : int, optional
        Optional seed to produce reproducible sequences of generated grids

    Usage
    -----
    >>> factory = GraphFactory(num_vertices=100, branching_factor=3.5, weighted=True)
    >>> graph = factory()
    >>> graph = factory()  # different graph
    """

    weighted: bool = False
    min_weight: float = Field(0.0, ge=0.0)
    max_weight: float = Field(1.0, ge=0.0)

    def __call__(self) -> Graph:
        edges = self._generate_edges()
        if self.weighted:
            weights = self._rng.uniform(
                self.min_weight, self.max_weight, size=len(edges)
            )
            edges = [(s, e, w) for (s, e), w in zip(edges, weights)]

        return Graph(
            num_vertices=self.num_vertices, directed=self.directed, edges=edges
        )


class SpatialGraphFactory(_GraphEnvFactory):
    """
    A factory for generating random spatial weighted graphs.

    Each node is assigned coordinates in `num_dimensions`-dimensional Euclidean space.
    Edges are created randomly, and each edge is assigned a weight that is
    guaranteed to be greater than or equal to the Euclidean distance between the two connected nodes.

    This structure is designed for testing pathfinding algorithms such as A*, where
    Euclidean distance can be used as an admissible heuristic.

    Note: The graph is not guaranteed to be fully connected.

    Attributes
    ----------
    num_vertices : int
        Number of nodes in the graph

    branching_factor : float
        Average number of outgoing edges per node

    directed : bool, default=True
        If True, creates a directed graph (each edge has a direction).
        If False, creates an undirected graph

    allow_multiedges : bool, default=True
        If True, multiple edges between the same pair of nodes are allowed

    allow_selfloops : bool, default=True
        allows edges to connect a vertex to itself

    num_dimensions : int, default=2
        Number of dimensions for node coordinates

    max_distance : int, default=100
        Maximum coordinate value for any axis

    random_seed : int, optional
        Optional seed to produce reproducible sequences of generated grids

    Usage
    -----
    >>> factory = SpatialGraphFactory(num_vertices=100, branching_factor=3.5, num_dimensions=3)
    >>> graph = factory()
    >>> graph = factory()  # different graph
    """

    num_dimensions: int = 2
    max_distance: int = 100

    def __call__(self) -> Graph:
        coordinates = self._rng.uniform(
            0, self.max_distance, size=(self.num_vertices, self.num_dimensions)
        )
        edges = self._generate_edges()

        weighted_edges = []
        for start, end in edges:

            distance = np.linalg.norm(coordinates[start] - coordinates[end])
            if distance == 0:
                cost = self._rng.uniform(0, self.max_distance)
            else:
                cost = distance * self._rng.uniform(1, 1.5)

            weighted_edges.append((start, end, cost))

        return Graph(
            num_vertices=self.num_vertices,
            directed=self.directed,
            edges=weighted_edges,
            coordinates=coordinates,
        )
