from typing import Union, Literal, Annotated
from pydantic import Field, model_validator

from .graph_factory import EnvFactory, RANDOM
from w9_pathfinding.envs import (
    Grid,
    Grid3D,
    HexGrid,
    HexLayout,
    DiagonalMovement,
)


class _GridEnvFactory(EnvFactory):

    obstacle_ratio: float = Field(0.0, ge=0.0, le=1.0)
    weighted: bool = False
    min_weight: float = Field(0.0, ge=0.0)
    max_weight: float = Field(1.0, ge=0.0)

    def _generate_weights(self, size):
        obstacles = self._rng.choice(
            [-1, 1], size=size, p=(self.obstacle_ratio, 1 - self.obstacle_ratio)
        )
        if not self.weighted:
            return obstacles

        weights = self._rng.uniform(self.min_weight, self.max_weight, size=size)
        weights[obstacles == -1] = -1
        return weights


class GridFactory(_GridEnvFactory):
    """
    A factory for generating random 2D grids.

    Parameters
    ----------
    width : int
        Number of columns in the grid

    height : int
        Number of rows in the grid

    obstacle_ratio : float, default=0.0
        Fraction of grid cells to turn into obstacles (0.0 = none, 1.0 = fully blocked)

    weighted : bool, default=False
        If True, nodes are assigned random weights in the range [min_weight, max_weight]

    min_weight : float, default=0.0
        Minimum edge weight when `weighted` is True

    max_weight : float, default=1.0
        Maximum edge weight when `weighted` is True

    diagonal_movement : DiagonalMovement | "random", default="never"
        Specifies when diagonal movement is allowed.

    diagonal_movement_cost_multiplier : float | "random", default=1.0
        Multiplier applied to diagonal movement weights (typically ~1.41)

    passable_up_down_border : bool | "random", default=False
        If True, wraparound is enabled from the top edge to the bottom edge

    passable_left_right_border : bool | "random", default=False
        If True, wraparound is enabled from the left edge to the right edge

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
    diagonal_movement: Union[DiagonalMovement, Literal[RANDOM]] = DiagonalMovement.never
    diagonal_movement_cost_multiplier: Union[
        Annotated[float, Field(ge=1.0, le=2.0)], Literal[RANDOM]
    ] = 1.0
    passable_up_down_border: Union[bool, Literal[RANDOM]] = False
    passable_left_right_border: Union[bool, Literal[RANDOM]] = False

    def __call__(self) -> Grid:
        weights = self._generate_weights((self.height, self.width))

        dm = self.diagonal_movement
        if dm == RANDOM:
            dm = self._rng.choice(list(DiagonalMovement))

        dm_cost = self.diagonal_movement_cost_multiplier
        if dm_cost == RANDOM:
            dm_cost = self._rng.uniform(1, 2)

        lr_border = self.passable_left_right_border
        if lr_border == RANDOM:
            lr_border = self._rng.choice([False, True])

        ud_border = self.passable_up_down_border
        if ud_border == RANDOM:
            ud_border = self._rng.choice([False, True])

        grid = Grid(
            weights,
            diagonal_movement=dm,
            diagonal_movement_cost_multiplier=dm_cost,
            passable_left_right_border=lr_border,
            passable_up_down_border=ud_border,
        )

        return grid


class Grid3DFactory(_GridEnvFactory):
    """
    A factory for generating random 3D grids.

    Parameters
    ----------
    width : int
        Number of columns in the grid

    height : int
        Number of rows in the grid

    depth : int
        Number of layers along the Z-axis (depth) of the grid

    obstacle_ratio : float, default=0.0
        Fraction of grid cells to turn into obstacles (0.0 = none, 1.0 = fully blocked)

    weighted : bool, default=False
        If True, nodes are assigned random weights in the range [min_weight, max_weight]

    min_weight : float, default=0.0
        Minimum edge weight when `weighted` is True

    max_weight : float, default=1.0
        Maximum edge weight when `weighted` is True

    passable_borders : bool | "random", default=False
        If True, allows wraparound movement across opposite borders of the grid (along any axis)

    random_seed : int, optional
        Optional seed to produce reproducible sequences of generated grids

    Usage
    -----
    >>> factory = Grid3DFactory(width=5, height=4, depth=2, obstacle_ratio=0.2, random_seed=42)
    >>> grid1 = factory()
    >>> grid2 = factory()  # different grid
    """

    width: int = Field(..., gt=0)
    height: int = Field(..., gt=0)
    depth: int = Field(..., gt=0)
    passable_borders: Union[bool, Literal[RANDOM]] = False

    def __call__(self) -> Grid3D:
        weights = self._generate_weights((self.depth, self.height, self.width))

        passable_borders = self.passable_borders
        if passable_borders == RANDOM:
            passable_borders = self._rng.choice([False, True])

        grid = Grid3D(weights, passable_borders=passable_borders)

        return grid


class HexGridFactory(_GridEnvFactory):
    """
    A factory for generating random hexagonal grids.

    Parameters
    ----------
    width : int
        Number of columns in the grid

    height : int
        Number of rows in the grid

    obstacle_ratio : float, default=0.0
        Fraction of grid cells to turn into obstacles (0.0 = none, 1.0 = fully blocked)

    weighted : bool, default=False
        If True, nodes are assigned random weights in the range [min_weight, max_weight]

    min_weight : float, default=0.0
        Minimum edge weight when `weighted` is True

    max_weight : float, default=1.0
        Maximum edge weight when `weighted` is True

    layout : HexLayout | "random", default="odd_r"
        Specifies the hex layout. Can be pointy-top, flat-top, or random

    passable_up_down_border : bool | "random", default=False
        If True, wraparound is enabled from the top edge to the bottom edge

    passable_left_right_border : bool | "random", default=False
        If True, wraparound is enabled from the left edge to the right edge

    random_seed : int, optional
        Optional seed to produce reproducible sequences of generated grids

    Usage
    -----
    >>> factory = HexGridFactory(width=5, height=4, obstacle_ratio=0.2, random_seed=42)
    >>> grid1 = factory()
    >>> grid2 = factory()  # different grid
    """

    width: int = Field(..., gt=0)
    height: int = Field(..., gt=0)
    layout: Union[HexLayout, Literal[RANDOM]] = HexLayout.odd_r
    passable_up_down_border: Union[bool, Literal[RANDOM]] = False
    passable_left_right_border: Union[bool, Literal[RANDOM]] = False

    @model_validator(mode="after")
    def check_wraparound(self):
        if (
            self.height % 2 == 1
            and self.layout != RANDOM
            and self.layout.is_pointy_top()
        ):
            if self.passable_up_down_border is True:
                raise ValueError(
                    "passable_up_down_border=True is not supported for odd-height grids "
                    f"with pointy-top layout (height={self.height}, layout={self.layout})."
                )
            elif self.passable_up_down_border == RANDOM:
                self.passable_up_down_border = False

        if self.width % 2 == 1 and self.layout != RANDOM and self.layout.is_flat_top():
            if self.passable_left_right_border is True:
                raise ValueError(
                    "passable_left_right_border=True is not supported for odd-width grids "
                    f"with flat-top layout (width={self.width}, layout={self.layout})."
                )
            elif self.passable_left_right_border == RANDOM:
                self.passable_left_right_border = False

        return self

    @model_validator(mode="after")
    def check_layouts(self):
        if not self.available_layouts:
            raise ValueError(
                "Cannot determine a valid layout for the given configuration: "
                f"width={self.width}, height={self.height}, "
                f"passable_up_down_border={self.passable_up_down_border}, "
                f"passable_left_right_border={self.passable_left_right_border}."
            )
        return self

    @property
    def available_layouts(self):
        if self.layout != RANDOM:
            return [self.layout]

        available_layouts = list(HexLayout)

        if self.height % 2 == 1 and self.passable_up_down_border is True:
            available_layouts = [x for x in available_layouts if not x.is_pointy_top()]

        if self.width % 2 == 1 and self.passable_left_right_border is True:
            available_layouts = [x for x in available_layouts if not x.is_flat_top()]

        return available_layouts

    def __call__(self) -> HexGrid:
        weights = self._generate_weights((self.height, self.width))

        layout = HexLayout(self._rng.choice(self.available_layouts))

        ud_border = self.passable_up_down_border
        if ud_border == RANDOM:
            if layout.is_pointy_top() and self.height % 2 == 1:
                ud_border = False
            else:
                ud_border = self._rng.choice([False, True])

        lr_border = self.passable_left_right_border
        if lr_border == RANDOM:
            if layout.is_flat_top() and self.width % 2 == 1:
                lr_border = False
            else:
                lr_border = self._rng.choice([False, True])

        grid = HexGrid(
            weights,
            layout=layout,
            passable_up_down_border=ud_border,
            passable_left_right_border=lr_border,
        )

        return grid
