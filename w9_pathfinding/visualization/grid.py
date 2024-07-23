import copy
from matplotlib import animation, colors, pyplot as plt
from matplotlib.patches import Circle, Rectangle


def draw_grid(ax, grid, show_grid=True, weight_colormap=plt.colormaps["Oranges"]):
    if show_grid:
        for x in range(grid.height + 1):
            ax.plot(
                [-0.5, grid.width - 0.5], [x - 0.5, x - 0.5], color="black", zorder=1
            )

        for x in range(grid.width + 1):
            ax.plot(
                [x - 0.5, x - 0.5], [-0.5, grid.height - 0.5], color="black", zorder=1
            )

    weights = grid.weights

    for x in range(grid.width):
        for y in range(grid.height):
            w = weights[y][x]
            if w == -1:
                ax.add_patch(
                    Rectangle(
                        xy=(x - 0.5, y - 0.5),
                        width=1,
                        height=1,
                        facecolor="gray",
                        zorder=0,
                    )
                )

    if weight_colormap:
        weight_list = sum(weights, [])
        max_weigts = max(weight_list)
        min_weight = min(x for x in weight_list if x != -1)
        if min_weight != max_weigts:
            norm = colors.Normalize(vmin=0, vmax=max_weigts)
            for x in range(grid.width):
                for y in range(grid.height):
                    w = weights[y][x]
                    if w >= 0:
                        color = weight_colormap(norm(w))
                        ax.add_patch(
                            Rectangle(
                                xy=(x - 0.5, y - 0.5),
                                width=1,
                                height=1,
                                facecolor=color,
                                zorder=0,
                            )
                        )


def draw_goal(ax, agent):
    if "goal" not in agent:
        return

    x, y = agent["goal"]
    ax.add_patch(
        Rectangle(
            xy=(x - 0.25, y - 0.25),
            width=0.5,
            height=0.5,
            facecolor=agent["color"],
            edgecolor="black",
            alpha=1,
            zorder=2,
        )
    )


def draw_start(ax, agent, agent_id=None):
    start = agent.get("start")
    if not start and agent.get("path"):
        start = agent["path"][0]

    if not start:
        return

    patch = Circle(
        xy=start,
        radius=0.25,
        facecolor=agent["color"],
        edgecolor="black",
        alpha=1,
        zorder=3,
    )
    ax.add_patch(patch)

    if agent_id is not None:
        text = ax.text(start[0], start[1], agent_id, zorder=4)
        text.set_horizontalalignment("center")
        text.set_verticalalignment("center")
        ax.add_artist(text)


def get_warped_points(grid, p, n):
    p1, n1 = list(n), list(p)

    if p[0] - n[0] > 1:
        p1[0] += grid.width
        n1[0] -= grid.width
    elif p[0] - n[0] < -1:
        p1[0] -= grid.width
        n1[0] += grid.width

    if p[1] - n[1] > 1:
        p1[1] += grid.height
        n1[1] -= grid.height
    elif p[1] - n[1] < -1:
        p1[1] -= grid.height
        n1[1] += grid.height

    return p1, n1


def split_path(grid, path):
    if not path:
        return []

    s_path = [[path[0]]]
    for i, n in enumerate(path[1:], start=1):
        p = path[i - 1]

        if not grid.adjacent(p, n) or (abs(p[0] - n[0]) <= 1 and abs(p[1] - n[1]) <= 1):
            s_path[-1].append(n)
            continue

        p1, n1 = get_warped_points(grid, p, n)

        s_path[-1].append(p1)
        s_path.append([n1, n])

    return s_path


def draw_path(ax, grid, agent):
    path = agent.get("path")
    if not path:
        return

    s_path = split_path(grid, path)
    for p in s_path:
        import matplotlib.patheffects as pe

        plt.plot(
            [x[0] for x in p],
            [x[1] for x in p],
            color=agent["color"],
            linewidth=5,
            zorder=1,
            alpha=0.7,
            path_effects=[
                pe.Stroke(linewidth=7, foreground="black", alpha=0.7),
                pe.Normal(),
            ],
        )


def draw_paths(ax, grid, agents):
    for agent_id, agent in enumerate(agents):
        draw_goal(ax, agent)
        draw_start(ax, agent, agent_id=agent_id if len(agents) > 1 else None)
        draw_path(ax, grid, agent)


class GridAnimation:

    def __init__(self, ax, grid, agents: list[dict]):
        self.ax = ax
        self.grid = grid
        self.agents = agents

        self.plot_objects = []

        for agent_id, agent in enumerate(self.agents):
            draw_goal(ax, agent)

            start = agent.get("start")
            if not start and agent.get("path"):
                start = agent["path"][0]
            if start:
                patch = Circle(
                    xy=start,
                    radius=0.25,
                    facecolor=agent["color"],
                    edgecolor="black",
                    alpha=0.7,
                    zorder=3,
                )
                agent["patch"] = patch
                self.plot_objects.append(patch)

                if len(self.agents) > 1:
                    text = self.ax.text(start[0], start[1], agent_id, zorder=4)
                    self.plot_objects.append(text)
                    text.set_horizontalalignment("center")
                    text.set_verticalalignment("center")
                    agent["text"] = text

    def init_func(self):
        for agent in self.agents:
            if "patch" in agent:
                self.ax.add_patch(agent["patch"])
            if "text" in agent:
                self.ax.add_artist(agent["text"])

        return self.plot_objects

    @staticmethod
    def get_middle_position(last_position, next_position, t):
        middle = list(last_position)
        for i in (0, 1):
            middle[i] += t * (next_position[i] - last_position[i])
        return tuple(middle)

    def get_position(self, path, time):
        if time > len(path) - 1:
            return

        if time == int(time):
            return path[int(time)]

        p = path[int(time)]
        n = path[int(time) + 1]

        if not self.grid.adjacent(p, n) or (
            abs(p[0] - n[0]) <= 1 and abs(p[1] - n[1]) <= 1
        ):
            return self.get_middle_position(p, n, time - int(time))

        p1, n1 = get_warped_points(self.grid, p, n)

        if time - int(time) < 0.5:
            return self.get_middle_position(p, p1, (time - int(time)))
        else:
            return self.get_middle_position(n1, n, (time - int(time)))

    def animate(self, step):
        time = step / 10

        for agent in self.agents:
            patch = agent.get("patch")
            text = agent.get("text")

            p = self.get_position(agent["path"], time)
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

        return self.plot_objects


def render_grid(
    grid,
    agents=None,
    size=4,
    padding=0.25,
    show_grid=False,
    weight_colormap=None,
    animate=False,
    animation_interval=100,
):
    agents = agents or []
    agents = copy.copy(agents)

    for i, a in enumerate(agents):
        a["id"] = i
        if "color" not in a:
            a["color"] = plt.colormaps["Set3"](i % 12)

    aspect = grid.width / grid.height

    fig = plt.figure(frameon=False, figsize=(size * aspect, size))
    fig.subplots_adjust(left=0, right=1, bottom=0, top=1, wspace=None, hspace=None)
    ax = fig.add_subplot(111, aspect="equal")

    ax.axis("off")
    ax.get_xaxis().set_visible(False)
    ax.get_yaxis().set_visible(False)
    ax.set_ylim(grid.height - 0.5 + padding, -0.5 - padding)
    ax.set_xlim(-0.5 - padding, grid.width - 0.5 + padding)

    draw_grid(ax, grid, show_grid=show_grid, weight_colormap=weight_colormap)

    if animate:
        grid_animation = GridAnimation(ax, grid, agents)

        path_lengths = [len(a["path"]) for a in agents if "path" in a]
        if not path_lengths:
            num_frames = 1
        else:
            num_frames = (max(path_lengths) - 1) * 10 + 2

        anim = animation.FuncAnimation(
            fig,
            func=grid_animation.animate,
            init_func=grid_animation.init_func,
            interval=animation_interval,
            blit=True,
            repeat=False,
            frames=num_frames,
        )

        # HTML(anim.to_html5_video())  # to visualize
        # anim.save(file_name, fps=10, dpi=200)  # to save
        out = anim

    else:
        draw_paths(ax, grid, agents)
        out = fig

    plt.close()
    return out
