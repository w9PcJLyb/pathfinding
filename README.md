# W9-Pathfinding

W9-Pathfinding is a versatile pathfinding library written in C++ with a Python interface provided by Cython. It offers a variety of pathfinding algorithms for navigating different types of maps, including grids, graphs, and 3D spaces. The library includes both classic pathfinding algorithms and multi-agent pathfinding algorithms.

Full documentation is available at: https://w9-pathfinding.readthedocs.io/

[![Apache-2.0 license](https://img.shields.io/github/license/w9PcJLyb/w9-pathfinding)](https://github.com/w9PcJLyb/w9-pathfinding/blob/main/LICENSE)
[![PyPI](https://img.shields.io/pypi/v/w9-pathfinding)](https://pypi.org/project/w9-pathfinding/)

### Key Features:

- Supports multiple environments: graphs, 2D/3D grids, and hexagonal grids
- Works with both weighted and unweighted environments
- Includes classical pathfinding algorithms (BFS, Dijkstra, A*, etc.)
- Includes multi-agent pathfinding algorithms (CBS, ICTS, WHCA*, and more)
- Supports pathfinding with dynamic obstacles
- Built-in visualization tools for debugging and demonstrations

### Quick start:

```python
from w9_pathfinding.envs import Grid
from w9_pathfinding.pf import Dijkstra

grid = Grid(width=4, height=3)
grid.add_obstacle((1, 1))

finder = Dijkstra(grid)
path = finder.find_path((0, 0), (3, 2))
print(path)
```

See more examples in the [Usage Guide](https://w9-pathfinding.readthedocs.io/latest/usage.html).

### Installation:

```bash
pip install w9-pathfinding
```
