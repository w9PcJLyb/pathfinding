import enum


class HexLayout(enum.IntEnum):
    """
    Enum representing hex grid layouts, defined by orientation and offset pattern.

    Layout types:

    - 'r' (row-based): pointy-top hexes aligned in rows
    - 'q' (column-based): flat-top hexes aligned in columns
    - 'odd': offset applies to odd rows or columns
    - 'even': offset applies to even rows or columns

    Modes:

        odd_r (0): Pointy-top hexes, odd rows are offset to the right::

            / \     / \     / \     / \   
          /     \ /     \ /     \ /     \ 
         | (0,0) | (1,0) | (2,0) | (3,0) |
         | (x,y) |       |       |       |
          \     / \     / \     / \     / \   
            \ /     \ /     \ /     \ /     \ 
             | (0,1) | (1,1) | (2,1) | (3,1) |
             |       |       |       |       |
            / \     / \     / \     / \     /
          /     \ /     \ /     \ /     \ /
         | (0,2) | (1,2) | (2,2) | (3,2) |
         |       |       |       |       |
          \     / \     / \     / \     /
            \ /     \ /     \ /     \ /

        even_r (1): Pointy-top hexes, even rows are offset to the right::

                / \     / \     / \     / \   
              /     \ /     \ /     \ /     \ 
             | (0,0) | (1,0) | (2,0) | (3,0) |
             | (x,y) |       |       |       |
            / \     / \     / \     / \     /
          /     \ /     \ /     \ /     \ /
         | (0,1) | (1,1) | (2,1) | (3,1) |
         |       |       |       |       |
          \     / \     / \     / \     / \   
            \ /     \ /     \ /     \ /     \ 
             | (0,2) | (1,2) | (2,2) | (3,2) |
             |       |       |       |       |
              \     / \     / \     / \     /
                \ /     \ /     \ /     \ /

        odd_q (2): Flat-top hexes, odd columns are offset downward::

            _ _           _ _
          /     \       /     \ 
         / (0,0) \ _ _ / (2,0) \ _ _
         \ (x,y) /     \       /     \  
          \ _ _ / (1,0) \ _ _ / (3,0) \ 
          /     \       /     \       /
         / (0,1) \ _ _ / (2,1) \ _ _ /
         \       /     \       /     \  
          \ _ _ / (1,1) \ _ _ / (3,1) \ 
          /     \       /     \       /
         / (0,2) \ _ _ / (2,2) \ _ _ /
         \       /     \       /     \  
          \ _ _ / (1,2) \ _ _ / (3,2) \ 
                \       /     \       /
                 \ _ _ /       \ _ _ /

        even_q (3): Flat-top hexes, even columns are offset downward::

                   _ _           _ _
                 /     \       /     \  
            _ _ / (1,0) \ _ _ / (3,0) \ 
          /     \       /     \       /
         / (0,0) \ _ _ / (2,0) \ _ _ /
         \ (x,y) /     \       /     \  
          \ _ _ / (1,1) \ _ _ / (3,1) \ 
          /     \       /     \       /
         / (0,1) \ _ _ / (2,1) \ _ _ /
         \       /     \       /     \  
          \ _ _ / (1,2) \ _ _ / (3,2) \ 
          /     \       /     \       /
         / (0,2) \ _ _ / (2,2) \ _ _ /
         \       /     \       /
          \ _ _ /       \ _ _ /

    """

    #: Pointy-top hexes, odd rows are offset to the right
    odd_r = 0

    #: Pointy-top hexes, even rows are offset to the right
    even_r = 1

    #: Flat-top hexes, odd columns are offset downward
    odd_q = 2

    #: Flat-top hexes, even columns are offset downward
    even_q = 3

    def __str__(self) -> str:
        return self.__repr__()

    def __repr__(self) -> str:
        return f"HexLayout.{self.name}"

    def is_pointy_top(self) -> bool:
        """
        Return whether this layout is a pointy-top (row-aligned) hex layout.
        """
        return self == HexLayout.odd_r or self == HexLayout.even_r

    def is_flat_top(self) -> bool:
        """
        Return whether this layout is a flat-top (column-aligned) hex layout.
        """
        return self == HexLayout.odd_q or self == HexLayout.even_q
