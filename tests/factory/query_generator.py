import numpy as np
from typing import Optional

from w9_pathfinding.envs import Graph, Grid, HexGrid, Grid3D


class QueryGenerator:
    """
    A reusable query generator for pathfinding benchmarks.

    Supports generating start-goal node pairs from different graph instances.
    Can enforce uniqueness and connectivity constraints.

    Attributes
    ----------
    random_seed : int or None
        Seed for reproducible query generation.
    """

    def __init__(self, random_seed: Optional[int] = None):
        self._rng = np.random.default_rng(random_seed)

    def generate_queries(
        self, graph, n: int, unique: bool = False, connected: bool = False
    ):
        """
        Generate a list of (start_node, goal_node) pairs for a given graph.

        Parameters
        ----------
        graph : Graph or Grid
            The graph instance to generate queries from

        n : int
            Number of queries to generate

        unique : bool, default=False
            If True, all start and goal nodes will be unique (no overlap)

        connected : bool, default=False
            Ensure start and goal nodes are connected by a path.
            Only supported for undirected graphs.

        Returns
        -------
        queries : list of tuples [(start, goal), ...]
        """

        if not connected:

            if isinstance(graph, Graph):
                nodes = np.arange(graph.size)
            elif isinstance(graph, (Grid, HexGrid, Grid3D)):
                obstacle_map = np.array(graph.obstacle_map)
                nodes = np.transpose(np.where(obstacle_map == 0))
            else:
                raise NotImplementedError()

            if unique and len(nodes) < n:
                raise ValueError(f"Can't generate {n} unique queries")

            starts = self._rng.choice(nodes, size=n, replace=not unique).tolist()
            goals = self._rng.choice(nodes, size=n, replace=not unique).tolist()
            return list(zip(starts, goals))

        else:
            nodes = []
            for comp_id, comp in enumerate(graph.find_components()):
                for node in comp:
                    nodes.append((node, comp_id))

            if unique and len(nodes) < n:
                raise ValueError(f"Can't generate {n} unique queries")

            starts = nodes
            goals = nodes

            queries = []
            while len(queries) < n:
                i = self._rng.choice(len(starts))
                start, comp_id = starts[i]
                connected_nodes = [n for n, c in goals if c == comp_id]
                if not connected_nodes:
                    continue

                j = self._rng.choice(len(connected_nodes))
                goal = connected_nodes[j]

                if unique:
                    starts = [x for x in starts if x[0] != start]
                    goals = [x for x in goals if x[0] != goal]

                queries.append((start, goal))

            return queries
