Visualization
================

Basic visualization is available for both `Grid` and `HexGrid` environments.
To use visualization features, you need to have `matplotlib` installed.

With the visualization tools, you can:

- Create **static images** of the grid (e.g., to show environments or agent paths)
- Generate **animated GIFs** that show agents moving over time (useful for MAPF problems)

Some examples of visualizations can be found in the :doc:`usage` section.

Code example:

.. code-block:: python

    from w9_pathfinding.envs import HexGrid
    from w9_pathfinding.visualization import plot_grid, animate_grid

    grid = HexGrid(
        weights =[
            [1,  1,  1, -1],
            [-1, 1,  1, -1],
            [1,  1, -1, -1],
            [1,  1,  1,  1],
        ]
    )

    agents = [
        {'start': (0, 0), 'goal': (2, 0), 'path': [(0, 0), (1, 0), (2, 0)]},
        {'start': (1, 1), 'goal': (1, 0), 'path': [(1, 1), (1, 1), (1, 0)]},
    ]

    # plot_grid returns a static matplotlib figure (useful for single-path problems)
    fig = plot_grid(grid, agents)

    # animate_grid returns a matplotlib animation (useful for MAPF)
    anim = animate_grid(grid, agents)

    # To visualize inline in Jupyter
    from IPython.display import HTML
    HTML(anim.to_html5_video())

    # Or save as a GIF
    # anim.save("out.gif", fps=10, dpi=200)
