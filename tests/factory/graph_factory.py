import numpy as np
from typing import Optional
from pydantic import Field, BaseModel, model_validator

from w9_pathfinding.envs import Graph

RANDOM = "random"


class EnvFactory(BaseModel):
    random_seed: Optional[int] = None

    class Config:
        extra = "forbid"

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

        if self.directed:
            all_edges = [
                (v, e)
                for v in range(self.num_vertices)
                for e in range(self.num_vertices)
                if self.allow_selfloops or v != e
            ]
        else:
            all_edges = []
            for v in range(self.num_vertices):
                s = v if self.allow_selfloops else v + 1
                all_edges += [(v, e) for e in range(s, self.num_vertices)]

        num_edges = int(self.branching_factor * self.num_vertices)
        if not self.directed:
            # For undirected graphs, each edge contributes
            # to the branching_factor of both connected nodes.
            num_edges //= 2

        if not self.allow_multiedges and num_edges > len(all_edges):
            raise ValueError(
                f"Cannot create {num_edges} unique edges "
                f"from only {len(all_edges)} possible combinations "
                f"(directed={self.directed}, self-loops={self.allow_selfloops})"
            )

        edges = self._rng.choice(all_edges, num_edges, replace=self.allow_multiedges)

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
