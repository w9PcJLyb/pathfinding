import enum


class DiagonalMovement(enum.IntEnum):
    """
    Enum defining when diagonal movement is allowed in grid-based pathfinding.

    Modes:

        never (0):
            Disallow diagonal movement entirely

        only_when_no_obstacle (1):
            Allow diagonal movement only if both adjacent cardinal cells are free

        if_at_most_one_obstacle (2):
            Allow diagonal movement if at least one of the adjacent cardinal cells is free

        always (3):
            Allow diagonal movement unconditionally, regardless of obstacles in cardinal directions

    """

    #: Disallow diagonal movement entirely
    never = 0

    #: Allow diagonal movement only if both adjacent cardinal cells are free
    only_when_no_obstacle = 1

    #: Allow diagonal movement if at least one of the adjacent cardinal cells is free
    if_at_most_one_obstacle = 2

    #: Allow diagonal movement unconditionally, regardless of obstacles in cardinal directions
    always = 3

    def __str__(self) -> str:
        return self.__repr__()

    def __repr__(self) -> str:
        return f"DiagonalMovement.{self.name}"
