import math
import copy
from matplotlib import animation, colors, pyplot as plt, patheffects as pe
from matplotlib.path import Path
from matplotlib.patches import Circle, Rectangle, PathPatch

from w9_pathfinding import Grid, HexGrid


class GridVisualizer:
    node_zorder = 1
    path_zorder = 1
    goal_zorder = 2
    start_zorder = 3
    text_zorder = 4

    grid_color = "black"
    obstacle_color = "gray"
    agents_colormap = plt.colormaps["Set3"]
    weights_colormap = plt.colormaps["Oranges"]

    def __init__(self, grid, agents=None, show_grid=True, show_weights=False) -> None:
        self.grid = grid
        self.agents = self._init_agents(agents)
        self.show_grid = show_grid
        self.show_weights = show_weights

        weight_list = sum(grid.weights, [])
        self.max_weight = max(weight_list)
        self.min_weight = min(x for x in weight_list if x != -1)

    def _init_agents(self, agents):
        agents = copy.deepcopy(agents or [])
        num_colors = len(self.agents_colormap.colors)
        for i, a in enumerate(agents):
            a["id"] = i
            if "color" not in a:
                a["color"] = self.agents_colormap(i % num_colors)
            if "label" not in a:
                a["label"] = str(i) if len(agents) > 1 else None
        return agents

    def _get_node_color(self, weight):
        if weight == -1:
            return self.obstacle_color

        if not self.show_weights or self.min_weight == self.max_weight:
            return

        norm = colors.Normalize(vmin=0, vmax=self.max_weight)
        return self.weights_colormap(norm(weight))

    def _draw_node(self, ax, x, y, weight):
        colors = {}

        face_color = self._get_node_color(weight)
        if face_color:
            colors["facecolor"] = face_color

        if self.show_grid:
            colors["edgecolor"] = self.grid_color

        if not colors:
            return

        if "facecolor" not in colors:
            colors["facecolor"] = "none"

        patch = Rectangle(
            xy=(x - 0.5, y - 0.5), width=1, height=1, zorder=self.node_zorder, **colors
        )
        ax.add_patch(patch)

    def _draw_map(self, ax):
        weights = self.grid.weights
        for x in range(self.grid.width):
            for y in range(self.grid.height):
                self._draw_node(ax, x, y, weights[y][x])

    def _plot_range(self):
        return -0.5, -0.5, self.grid.width - 0.5, self.grid.height - 0.5

    def _create_plot(self, size=4, padding=0.25):
        aspect = self.grid.width / self.grid.height

        fig = plt.figure(frameon=False, figsize=(size * aspect, size))
        fig.subplots_adjust(left=0, right=1, bottom=0, top=1, wspace=None, hspace=None)
        ax = fig.add_subplot(111, aspect="equal")

        ax.axis("off")
        ax.get_xaxis().set_visible(False)
        ax.get_yaxis().set_visible(False)

        min_x, min_y, max_x, max_y = self._plot_range()
        ax.set_ylim(max_y + padding, min_y - padding)
        ax.set_xlim(min_x - padding, max_x + padding)

        return fig, ax

    def _draw_agent(self, ax, xy, *, color="black", label=None):
        patch = Circle(
            xy=xy,
            radius=0.25,
            facecolor=color,
            edgecolor="black",
            alpha=1,
            zorder=self.start_zorder,
        )
        ax.add_patch(patch)

        text = None
        if label is not None:
            text = ax.text(*xy, label, zorder=self.text_zorder)
            text.set_horizontalalignment("center")
            text.set_verticalalignment("center")
            ax.add_artist(text)

        return patch, text

    def _draw_goal(self, ax, xy, *, color="black"):
        patch = Rectangle(
            xy=(xy[0] - 0.25, xy[1] - 0.25),
            width=0.5,
            height=0.5,
            facecolor=color,
            edgecolor="black",
            zorder=self.goal_zorder,
        )
        ax.add_patch(patch)

    def _get_warped_points(self, p, n):
        p1, n1 = list(n), list(p)
        w = self.grid.width
        h = self.grid.height

        if p[0] - n[0] > 1:
            p1[0] += w
            n1[0] -= w
        elif p[0] - n[0] < -1:
            p1[0] -= w
            n1[0] += w

        if p[1] - n[1] > 1:
            p1[1] += h
            n1[1] -= h
        elif p[1] - n[1] < -1:
            p1[1] -= h
            n1[1] += h

        return p1, n1

    def _split_path(self, path):
        if not path:
            return []

        s_path = [[path[0]]]
        for i, n in enumerate(path[1:], start=1):
            p = path[i - 1]

            if not self.grid.adjacent(p, n) or (
                abs(p[0] - n[0]) <= 1 and abs(p[1] - n[1]) <= 1
            ):
                s_path[-1].append(n)
                continue

            p1, n1 = self._get_warped_points(p, n)

            s_path[-1].append(p1)
            s_path.append([n1, n])

        return s_path

    def _draw_path(self, ax, paths, *, color="black"):
        for p in paths:
            ax.plot(
                [x[0] for x in p],
                [x[1] for x in p],
                color=color,
                linewidth=5,
                zorder=self.path_zorder,
                alpha=0.7,
                path_effects=[
                    pe.Stroke(linewidth=7, foreground="black", alpha=0.7),
                    pe.Normal(),
                ],
            )

    def _draw_paths(self, ax):
        for agent in self.agents:
            color = agent["color"]

            if "goal" in agent:
                self._draw_goal(ax, agent["goal"], color=color)

            if "start" in agent:
                self._draw_agent(ax, agent["start"], color=color, label=agent["label"])

            if "path" in agent:
                self._draw_path(ax, self._split_path(agent["path"]), color=color)

    @staticmethod
    def _find_position(p1, p2, time):
        middle = list(p1)
        for i in (0, 1):
            middle[i] += time * (p2[i] - p1[i])
        return middle

    def _get_position(self, path, time):
        if time > len(path) - 1:
            return

        if time == int(time):
            return self._find_position(path[int(time)], path[int(time)], 0)

        p = path[int(time)]
        n = path[int(time) + 1]

        if not self.grid.adjacent(p, n) or (
            abs(p[0] - n[0]) <= 1 and abs(p[1] - n[1]) <= 1
        ):
            return self._find_position(p, n, time - int(time))

        p1, n1 = self._get_warped_points(p, n)

        if time - int(time) < 0.5:
            return self._find_position(p, p1, (time - int(time)))
        else:
            return self._find_position(n1, n, (time - int(time)))

    def plot(self, size=4):
        fig, ax = self._create_plot(size=size)
        self._draw_map(ax)
        self._draw_paths(ax)
        plt.close()
        return fig

    def animate(self, size=4):
        fig, ax = self._create_plot(size=size)
        self._draw_map(ax)
        agents = self.agents

        plot_objects = []
        for agent in agents:
            color = agent["color"]

            if "goal" in agent:
                self._draw_goal(ax, agent["goal"], color=color)

            start = agent.get("start")
            if not start and agent.get("path"):
                start = agent["path"][0]
            if start:
                patch, text = self._draw_agent(
                    ax, start, color=color, label=agent["label"]
                )
                agent["patch"] = patch
                plot_objects.append(patch)
                if text:
                    plot_objects.append(patch)
                    agent["text"] = text

        def init_func():
            for agent in agents:
                if "patch" in agent:
                    ax.add_patch(agent["patch"])
                if "text" in agent:
                    ax.add_artist(agent["text"])

            return plot_objects

        def animate(step):
            time = step / 10

            for agent in agents:
                patch = agent.get("patch")
                text = agent.get("text")

                p = self._get_position(agent["path"], time)
                if p:
                    if patch:
                        patch.set_center(p)
                    if text:
                        text.set_position(p)
                else:
                    # disable
                    if patch:
                        patch.set_fill(False)
                        patch.set_edgecolor(agent["color"])
                    if text:
                        text.set_color("grey")

            return plot_objects

        path_lengths = [len(a["path"]) for a in agents if "path" in a]
        if not path_lengths:
            num_frames = 1
        else:
            num_frames = (max(path_lengths) - 1) * 10 + 2

        anim = animation.FuncAnimation(
            fig,
            func=animate,
            init_func=init_func,
            interval=100,
            blit=True,
            repeat=False,
            frames=num_frames,
        )
        # HTML(anim.to_html5_video())  # to visualize
        # anim.save(file_name, fps=10, dpi=200)  # to save

        plt.close()
        return anim


class HexVisualizer(GridVisualizer):

    tan30 = math.sqrt(3) / 3
    cos30 = math.sqrt(3) / 2
    hex_vertices = [
        (0.5, 0.5 * tan30),
        (0.5, -0.5 * tan30),
        (0, -0.5 / cos30),
        (-0.5, -0.5 * tan30),
        (-0.5, 0.5 * tan30),
        (0, 0.5 / cos30),
        (0.5, 0.5 * tan30),
    ]
    hex_codes = (
        [Path.MOVETO] + [Path.LINETO] * (len(hex_vertices) - 2) + [Path.CLOSEPOLY]
    )

    @staticmethod
    def _hex_to_cartesian(hex_x, hex_y):
        y = hex_y * math.sqrt(3) / 2
        if hex_y % 2 == 0:
            x = hex_x
        else:
            x = hex_x + 0.5
        return x, y

    def _plot_range(self):
        min_x, min_y = self._hex_to_cartesian(0, 0)
        max_x, max_y = self._hex_to_cartesian(self.grid.width - 1, self.grid.height - 1)
        if self.grid.height % 2 == 1:
            max_x += 0.5
        dy = 1 / math.sqrt(3)
        return min_x - 0.5, min_y - dy, max_x + 0.5, max_y + dy

    def _draw_node(self, ax, x, y, weight):
        colors = {}

        face_color = self._get_node_color(weight)
        if face_color:
            colors["facecolor"] = face_color

        if self.show_grid:
            colors["edgecolor"] = self.grid_color

        if not colors:
            return

        if "facecolor" not in colors:
            colors["facecolor"] = "none"

        x, y = self._hex_to_cartesian(x, y)

        hex_vertices = [(x + x_, y + y_) for x_, y_ in self.hex_vertices]
        path = Path(hex_vertices, self.hex_codes)
        patch = PathPatch(path, zorder=self.node_zorder, **colors)
        ax.add_patch(patch)

    def _draw_agent(self, ax, xy, **kwargs):
        xy = self._hex_to_cartesian(*xy)
        return super()._draw_agent(ax, xy, **kwargs)

    def _draw_goal(self, ax, xy, **kwargs):
        xy = self._hex_to_cartesian(*xy)
        super()._draw_goal(ax, xy, **kwargs)

    def _draw_path(self, ax, paths, **kwargs):
        for i in range(len(paths)):
            paths[i] = [self._hex_to_cartesian(*x) for x in paths[i]]
        super()._draw_path(ax, paths, **kwargs)

    def _find_position(self, p1, p2, time):
        return super()._find_position(
            self._hex_to_cartesian(*p1), self._hex_to_cartesian(*p2), time
        )


def get_visualizer(grid):
    if isinstance(grid, Grid):
        return GridVisualizer
    elif isinstance(grid, HexGrid):
        return HexVisualizer
    else:
        raise NotImplementedError()


def plot_grid(grid, agents=None, show_grid=True, show_weights=False, **kwargs):
    visualizer = get_visualizer(grid)
    return visualizer(
        grid, agents, show_grid=show_grid, show_weights=show_weights
    ).plot(**kwargs)


def animate_grid(grid, agents=None, show_grid=True, show_weights=False, **kwargs):
    visualizer = get_visualizer(grid)
    return visualizer(
        grid, agents, show_grid=show_grid, show_weights=show_weights
    ).animate(**kwargs)
