import matplotlib.pyplot as plt
import matplotlib.animation as animation

# =========================
# NEW: Modular Plot Helpers
# =========================

def plot_waypoints(ax, waypoints):
    xs = [wp.transform.location.x for wp in waypoints]
    ys = [wp.transform.location.y for wp in waypoints]
    ax.scatter(xs, ys, c='lightgray', s=5, alpha=0.6, label="Waypoints")


def plot_topology(ax, topology):
    for wp1, wp2 in topology:
        x1 = wp1.transform.location.x
        y1 = wp1.transform.location.y
        x2 = wp2.transform.location.x
        y2 = wp2.transform.location.y

        ax.plot(
            [x1, x2], [y1, y2],
            color='gray',
            linewidth=0.5,
            alpha=0.5
        )


def plot_path(ax, path):
    xs = [wp.transform.location.x for wp, _ in path]
    ys = [wp.transform.location.y for wp, _ in path]

    ax.plot(xs, ys, c='blue', linewidth=3, label="Planned Path")


def plot_start_goal(ax, start, goal):
    ax.scatter(
        start.transform.location.x,
        start.transform.location.y,
        c='green', s=100, label='Start'
    )

    ax.scatter(
        goal.transform.location.x,
        goal.transform.location.y,
        c='red', s=100, label='Goal'
    )


# =========================
# REPLACED: Main Render
# =========================

def render(waypoints=None, topology=None, path=None, start=None, goal=None):
    fig, ax = plt.subplots(figsize=(10, 10))

    if waypoints:
        plot_waypoints(ax, waypoints)

    if topology:
        plot_topology(ax, topology)

    if path:
        plot_path(ax, path)

    if start and goal:
        plot_start_goal(ax, start, goal)

    ax.set_title("Waypoint Graph Visualization")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.axis("equal")
    ax.legend()

    plt.show()


# =========================
# NEW: Agent Animation
# =========================

def animate_agent(path):
    fig, ax = plt.subplots()

    xs = [wp.transform.location.x for wp, _ in path]
    ys = [wp.transform.location.y for wp, _ in path]

    ax.plot(xs, ys, 'b-', label="Path")

    agent_dot, = ax.plot([], [], 'ro', markersize=8)

    def update(frame):
        agent_dot.set_data(xs[frame], ys[frame])
        return agent_dot,

    ani = animation.FuncAnimation(
        fig,
        update,
        frames=len(xs),
        interval=100,
        repeat=False
    )

    plt.legend()
    plt.show()


# =========================
# EXISTING ENTRY POINT (MODIFIED)
# =========================

def visualize_map_cache(waypoints, topology=None, path=None, start=None, goal=None):
    """
    Existing function upgraded to use new rendering system.
    """

    render(
        waypoints=waypoints,
        topology=topology,
        path=path,
        start=start,
        goal=goal
    )
    