import enum


class DiagonalMovement(enum.IntEnum):
    never = 0
    only_when_no_obstacle = 1
    if_at_most_one_obstacle = 2
    always = 3

    def __str__(self) -> str:
        return self.__repr__()

    def __repr__(self) -> str:
        return f"DiagonalMovement.{self.name}"
