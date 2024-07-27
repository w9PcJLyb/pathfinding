import enum


class HexLayout(enum.IntEnum):
    # pointy top
    odd_r = 0
    even_r = 1
    # flat top
    odd_q = 2
    even_q = 3

    def __str__(self) -> str:
        return self.__repr__()

    def __repr__(self) -> str:
        return f"HexLayout.{self.name}"

    def is_pointy_top(self):
        return self == HexLayout.odd_r or self == HexLayout.even_r

    def is_flat_top(self):
        return self == HexLayout.odd_q or self == HexLayout.even_q
