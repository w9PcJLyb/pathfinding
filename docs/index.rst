.. w9-pathfinding documentation master file, created by
   sphinx-quickstart on Fri Jun 13 15:21:56 2025.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

w9-pathfinding
============================

W9-Pathfinding is a versatile pathfinding library written in C++ with a Python
interface provided by Cython. It offers a variety of pathfinding algorithms for
navigating different types of maps, including grids, graphs, and 3D spaces.
The library includes both classic pathfinding algorithms and multi-agent
pathfinding algorithms.

.. image:: https://img.shields.io/badge/source-GitHub-blue?logo=github
   :target: https://github.com/w9PcJLyb/pathfinding
   :alt: GitHub repository

.. image:: https://img.shields.io/pypi/v/w9-pathfinding
    :target: https://pypi.org/project/w9-pathfinding/
    :alt: PyPI version

.. image:: https://img.shields.io/github/license/w9PcJLyb/pathfinding
    :target: https://github.com/w9PcJLyb/pathfinding/blob/main/LICENSE
    :alt: License

Key Features:
----------------------------

- Supports multiple environments: graphs, 2D/3D grids, and hexagonal grids
- Works with both weighted and unweighted environments
- Includes classical pathfinding algorithms (BFS, Dijkstra, A*, etc.)
- Includes multi-agent pathfinding algorithms (CBS, ICTS, WHCA*, and more)
- Supports pathfinding with dynamic obstacles
- Built-in visualization tools for debugging and demonstrations

Why "W9"?
----------------------------

The name w9-pathfinding comes from the **weighted 9-connected grid** you can
use for **pathfinding** in the library. The "W" stands for Weighted, indicating
that the library supports both weighted and non-weighted maps, where movement
costs may vary. The "9" refers to the 9 possible movement directions: the 4 cardinal
directions (North, South, East, West), the 4 diagonal directions (NE, SE, SW, NW),
and a center movement (staying in place, which is crucial in multi-agent scenarios).

.. toctree::
   :maxdepth: 2
   :caption: Table of Contents:

   Home <self>
   Installation <./installation.rst>
   Usage <./usage.rst>
   Environments <./envs/index.rst>
   Pathfinding <./pf/index.rst>
   Multi-Agent Pathfinding <./mapf/index.rst>
   Visualization <./visualization>
   Changelog <./changelog.rst>
