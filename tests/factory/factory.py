import random
from typing import Union, Literal, Optional, Annotated
from pydantic import BaseModel, Field

from w9_pathfinding.envs import (
    Grid,
    Graph,
    Grid3D,
    HexGrid,
    HexLayout,
    DiagonalMovement,
)

RANDOM = "random"


class _EnvFactory(BaseModel):
    random_seed: Optional[int] = None

    class Config:
        extra = "forbid"

    def __init__(self, **data):
        super().__init__(**data)
        self._rng = random.Random(self.random_seed)

    @property
    def rng(self):
        return self._rng

    def __call__(self):
        NotImplementedError()


class GridFactory(_EnvFactory):
    """
    A configurable generator for creating random 2D grid graphs.

    Parameters
    ----------
    width : int
        Number of columns in the grid.

    height : int
        Number of rows in the grid.

    obstacle_ratio : float, default=0.0
        Fraction of grid cells to turn into obstacles (0.0 = none, 1.0 = fully blocked).

    weighted : bool, default=False
        If True, nodes are assigned random weights between min_weight and max_weight.

    min_weight : float, default=1.0
        Minimum edge weight when `weighted` is True.

    max_weight : float, default=1.0
        Maximum edge weight when `weighted` is True.

    diagonal_movement : DiagonalMovement | "random", default="never"
        Specifies when diagonal movement is allowed.

    diagonal_movement_cost_multiplier : float | "random", default=1.0
        Multiplier applied to diagonal movement weights (typically ~1.41).

    passable_left_right_border : bool | "random", default=False
        If True, wraparound is enabled from the left edge to the right edge

    passable_up_down_border : bool | "random", default=False
        If True, wraparound is enabled from the top edge to the bottom edge

    random_seed : int, optional
        Optional seed to produce reproducible sequences of generated grids

    Usage
    -----
    >>> factory = GridFactory(width=10, height=5, obstacle_ratio=0.2, random_seed=42)
    >>> grid1 = factory()
    >>> grid2 = factory()  # different grid
    """

    width: int = Field(..., gt=0)
    height: int = Field(..., gt=0)
    obstacle_ratio: float = Field(0.0, ge=0.0, le=1.0)
    weighted: bool = False
    min_weight: float = Field(1.0, ge=0.0)
    max_weight: float = Field(1.0, ge=0.0)
    diagonal_movement: Union[DiagonalMovement, Literal[RANDOM]] = DiagonalMovement.never
    diagonal_movement_cost_multiplier: Union[
        Annotated[float, Field(ge=1.0, le=2.0)], Literal[RANDOM]
    ] = 1.0
    passable_left_right_border: Union[bool, Literal[RANDOM]] = False
    passable_up_down_border: Union[bool, Literal[RANDOM]] = False

    def __call__(self):
        weights = self._generate_weights()

        dm = self.diagonal_movement
        if dm == RANDOM:
            dm = self.rng.choice(list(DiagonalMovement))

        dm_cost = self.diagonal_movement_cost_multiplier
        if dm_cost == RANDOM:
            dm_cost = self.rng.uniform(1, 2)

        lr_border = self.passable_left_right_border
        if lr_border == RANDOM:
            lr_border = self.rng.choice([False, True])

        ud_border = self.passable_up_down_border
        if ud_border == RANDOM:
            ud_border = self.rng.choice([False, True])

        grid = Grid(
            weights,
            diagonal_movement=dm,
            diagonal_movement_cost_multiplier=dm_cost,
            passable_left_right_border=lr_border,
            passable_up_down_border=ud_border,
        )

        return grid

    def _generate_weights(self):
        weights = []
        for _ in range(self.height):
            row = [0] * self.width
            for i in range(self.width):
                if self.rng.random() < self.obstacle_ratio:
                    w = -1  # obstacle
                elif self.weighted:
                    w = self.rng.uniform(self.min_weight, self.max_weight)
                else:
                    w = 1
                row[i] = w
            weights.append(row)
        return weights
