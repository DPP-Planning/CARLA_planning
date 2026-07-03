import argparse
import ast
import heapq
import json
import math
import os

from pathlib import Path
from collections import deque

import matplotlib.pyplot as plt
import matplotlib.animation as animation

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def animate_agent(ax, graph, path):

    # Extract coordinates
    coords = [graph[n]["location"] for n in path]
    xs = [c[0] for c in coords]
    ys = [c[1] for c in coords]

    # Agent marker (DO NOT recreate fig)
    agent_dot, = ax.plot([], [], 'ro', markersize=10, zorder=10)

    # Interpolate for smooth motion
    interp_x, interp_y = [], []
    for i in range(len(xs) - 1):
        interp_x.extend(np.linspace(xs[i], xs[i+1], 20))
        interp_y.extend(np.linspace(ys[i], ys[i+1], 20))

    def update(frame):
        agent_dot.set_data(
            [interp_x[frame]],
            [interp_y[frame]]
        )
        return agent_dot,

    anim = FuncAnimation(
        ax.figure,   # 👈 IMPORTANT: use existing figure
        update,
        frames=len(interp_x),
        interval=100,
        blit=True,
        repeat=False
    )

    return anim

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

    ax.plot(xs, ys, color='red', linewidth=3, label='Planned Path', zorder=3)
    plt.scatter(xs[0], ys[0], color='green', s=100, edgecolors='black', zorder=5, label='Start')
    plt.text(xs[0], ys[0], ' START', fontsize=10, weight='bold', color='green')

    # Goal point
    plt.scatter(xs[-1], ys[-1], color='blue', s=100, edgecolors='black', zorder=5, label='Goal')
    plt.text(xs[-1], ys[-1], ' GOAL', fontsize=10, weight='bold', color='blue')


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

PROJECT_ROOT = Path(__file__).resolve().parent
DEFAULT_MPLCONFIGDIR = PROJECT_ROOT / ".matplotlib-cache"
os.environ.setdefault("MPLCONFIGDIR", str(DEFAULT_MPLCONFIGDIR))


def resolve_path(path_text):
    path = Path(path_text).expanduser()
    if not path.is_absolute():
        path = PROJECT_ROOT / path
    return path


def parse_waypoint_graph(cache_path):
    graph = {}
    current_id = None

    with open(cache_path, "r", encoding="utf-8") as cache_file:
        for raw_line in cache_file:
            line = raw_line.strip()
            if not line:
                current_id = None
                continue

            key, _, value = line.partition(": ")
            if key == "waypoint_id":
                current_id = int(value)
                graph[current_id] = {}
            elif current_id is not None:
                if key in {"location", "predecessors", "successors"}:
                    graph[current_id][key] = ast.literal_eval(value)
                elif key in {"g", "rhs"}:
                    graph[current_id][key] = float(value)

    return graph


def parse_location(location_text):
    values = [float(value.strip()) for value in location_text.split(",")]
    if len(values) == 2:
        values.append(0.0)
    if len(values) != 3:
        raise ValueError("Locations must be formatted as x,y or x,y,z.")
    return tuple(values)


def parse_id_payload(path):
    resolved_path = resolve_path(path)
    with open(resolved_path, "r", encoding="utf-8") as input_file:
        text = input_file.read().strip()

    if not text:
        return {}

    if resolved_path.suffix.lower() == ".json":
        payload = json.loads(text)
    else:
        payload = [int(token) for token in text.replace(",", " ").split()]

    if isinstance(payload, list):
        return {"route": [int(waypoint_id) for waypoint_id in payload]}

    parsed = {}
    actual_route = payload.get("actual_route") or payload.get("route", [])
    parsed["route"] = [int(waypoint_id) for waypoint_id in actual_route]

    for key in ("explored", "obstacles"):
        parsed[key] = [int(waypoint_id) for waypoint_id in payload.get(key, [])]
    if payload.get("perception", {}).get("seen_obstacles"):
        parsed.setdefault("obstacles", [])
        parsed["obstacles"].extend(
            int(waypoint_id)
            for waypoint_id in payload["perception"].get("seen_obstacles", [])
        )
    for key in ("start", "goal"):
        if payload.get(key) is not None:
            parsed[key] = int(payload[key])

    parsed["planned_routes"] = [
        {
            **route_record,
            "route": [int(waypoint_id) for waypoint_id in route_record.get("route", [])],
            "obstacles": [int(waypoint_id) for waypoint_id in route_record.get("obstacles", [])],
        }
        for route_record in payload.get("planned_routes", [])
    ]
    parsed["reroutes"] = [
        {
            **route_record,
            "route": [int(waypoint_id) for waypoint_id in route_record.get("route", [])],
            "obstacles": [int(waypoint_id) for waypoint_id in route_record.get("obstacles", [])],
            "trigger_obstacles": [
                int(waypoint_id) for waypoint_id in route_record.get("trigger_obstacles", [])
            ],
        }
        for route_record in payload.get("reroutes", [])
    ]
    parsed["obstacle_events"] = [
        {
            **event,
            "obstacles": [int(waypoint_id) for waypoint_id in event.get("obstacles", [])],
        }
        for event in payload.get("obstacle_events", [])
    ]

    return parsed


def closest_waypoint_id(graph, location):
    closest_id = None
    closest_distance = float("inf")
    for waypoint_id, data in graph.items():
        waypoint_location = data.get("location")
        if waypoint_location is None:
            continue
        distance = math.dist(location, waypoint_location)
        if distance < closest_distance:
            closest_id = waypoint_id
            closest_distance = distance
    return closest_id


def edge_cost(graph, start_id, end_id):
    start_location = graph[start_id]["location"]
    end_location = graph[end_id]["location"]
    return math.dist(start_location, end_location)


def find_shortest_route(graph, start_id, goal_id, blocked_ids=None):
    blocked_ids = set(blocked_ids or [])
    missing_ids = [waypoint_id for waypoint_id in (start_id, goal_id) if waypoint_id not in graph]
    if missing_ids:
        available_ids = sorted(graph.keys())
        preview_ids = ", ".join(str(waypoint_id) for waypoint_id in available_ids[:12])
        raise ValueError(
            f"Waypoint ID(s) not found in map cache: {missing_ids}. "
            f"The first available IDs are: {preview_ids}. "
            "Use --list-waypoints to inspect valid IDs, or use --start-location/--goal-location."
        )

    if start_id in blocked_ids or goal_id in blocked_ids:
        return []

    queue = [(0.0, start_id)]
    distances = {start_id: 0.0}
    parents = {}
    visited = set()

    while queue:
        current_distance, current_id = heapq.heappop(queue)
        if current_id in visited:
            continue
        visited.add(current_id)

        if current_id == goal_id:
            route = [goal_id]
            while route[-1] != start_id:
                route.append(parents[route[-1]])
            route.reverse()
            return route

        for successor_id in graph[current_id].get("successors", []):
            if successor_id in blocked_ids or successor_id not in graph:
                continue
            next_distance = current_distance + edge_cost(graph, current_id, successor_id)
            if next_distance < distances.get(successor_id, float("inf")):
                distances[successor_id] = next_distance
                parents[successor_id] = current_id
                heapq.heappush(queue, (next_distance, successor_id))

    return []


def build_line_segments(graph):
    locations = {
        waypoint_id: data["location"]
        for waypoint_id, data in graph.items()
        if "location" in data
    }
    segments = []

    for waypoint_id, data in graph.items():
        start = locations.get(waypoint_id)
        if start is None:
            continue

        for successor_id in data.get("successors", []):
            end = locations.get(successor_id)
            if end is not None:
                segments.append(((start[0], start[1]), (end[0], end[1])))

    return locations, segments


def build_segments_for_ids(graph, waypoint_ids):
    segments = []
    for index in range(len(waypoint_ids) - 1):
        start_id = waypoint_ids[index]
        end_id = waypoint_ids[index + 1]
        start = graph.get(start_id, {}).get("location")
        end = graph.get(end_id, {}).get("location")
        if start is not None and end is not None:
            segments.append(((start[0], start[1]), (end[0], end[1])))
    return segments


def build_explored_segments(graph, explored_ids):
    explored_ids = set(explored_ids)
    segments = []
    for waypoint_id in explored_ids:
        start = graph.get(waypoint_id, {}).get("location")
        if start is None:
            continue
        for successor_id in graph[waypoint_id].get("successors", []):
            if successor_id not in explored_ids:
                continue
            end = graph.get(successor_id, {}).get("location")
            if end is not None:
                segments.append(((start[0], start[1]), (end[0], end[1])))
    return segments


def add_route_layer(axis, graph, route_ids, color, label, linewidth, alpha=0.9, linestyle="solid", zorder=4):
    from matplotlib.collections import LineCollection

    route_segments = build_segments_for_ids(graph, route_ids)
    if not route_segments:
        return

    route_collection = LineCollection(
        route_segments,
        colors=color,
        linewidths=linewidth,
        alpha=alpha,
        linestyles=linestyle,
        label=label,
        zorder=zorder,
    )
    axis.add_collection(route_collection)

def draw_waypoint_marker(axis, graph, waypoint_id, color, label, marker):
    """Draw a highlighted waypoint (start/goal) with clear visibility."""
    if waypoint_id is None:
        return

    wp = graph.nodes[waypoint_id]
    x, y = wp["pos"]

    axis.scatter(
        x,
        y,
        c=color,
        s=180,  
        marker=marker,
        edgecolors="black",
        linewidths=0.9,
        zorder=5,
        label=label,
    )

    axis.annotate(
        f"{label}",
        (x, y),
        textcoords="offset points",
        xytext=(6, 6),
        fontsize=11,
        weight="bold",
        color=color,
        zorder=6,
    )


def add_route_layer(axis, graph, route_ids):
    """Draw the main route with stronger visibility."""
    if not route_ids or len(route_ids) < 2:
        return

    xs = []
    ys = []

    for wp_id in route_ids:
        x, y = graph.nodes[wp_id]["pos"]
        xs.append(x)
        ys.append(y)

    axis.plot(
        xs,
        ys,
        color="#dc2626",   
        linewidth=5.0,     
        alpha=1.0,        
        zorder=4,
        label="Route",
    )

def extract_path(graph, start_id, goal_id):
    queue = deque([start_id])
    parent = {start_id: None}

    while queue:
        current = queue.popleft()

        if current == goal_id:
            break

        for neighbor in graph[current].get("successors", []):
            if neighbor not in parent:
                parent[neighbor] = current
                queue.append(neighbor)

    # Reconstruct path
    if goal_id not in parent:
        print("No path found!")
        return []

    path = []
    node = goal_id

    while node is not None:
        path.append(node)
        node = parent[node]

    path.reverse()
    return path

def draw_background_graph(ax, graph):
    for node_id, node_data in graph.items():
        x1, y1 = get_xy(node_data)

        for neighbor in node_data.get("successors", []):
            if neighbor not in graph:
                continue

            x2, y2 = get_xy(graph[neighbor])

            ax.plot(
                [x1, x2],
                [y1, y2],
                linewidth=1,
                color="gray",
                alpha=0.7
            )

def save_plot(figure, axis, output_path):
    """Finalize and save plot."""
    axis.set_title("Waypoint Graph Visualization", fontsize=12)
    axis.legend(loc="best", fontsize=8, framealpha=0.9)

    plt.savefig(output_path, dpi=200)
    plt.close(figure)

def get_xy(node_data):
    """
    Extract (x, y) from multiple possible formats.
    """

    # Case 1: direct x/y
    if isinstance(node_data, dict) and "x" in node_data and "y" in node_data:
        return node_data["x"], node_data["y"]

    # Case 2: pos tuple
    if "pos" in node_data:
        return node_data["pos"]

    # Case 3: location
    if "location" in node_data:
        loc = node_data["location"]

        if isinstance(loc, (tuple, list)):
            return loc[0], loc[1]

        if isinstance(loc, dict):
            return loc["x"], loc["y"]

    # Case 4: transform.location
    if "transform" in node_data:
        loc = node_data["transform"]["location"]

        if isinstance(loc, (tuple, list)):
            return loc[0], loc[1]

        return loc["x"], loc["y"]

    # Case 5: CARLA waypoint
    if "waypoint" in node_data:
        loc = node_data["waypoint"].transform.location
        return loc.x, loc.y

    raise ValueError(f"Unknown node format: {node_data}")

def pick_start_and_goal(graph):
    """
    Picks two nodes from the graph as start and goal.
    Simple fallback: first and last node.
    """

    nodes = list(graph.keys())

    if len(nodes) < 2:
        return None, None

    return nodes[0], nodes[-1]

def draw_path(axis, graph, path):
    for i in range(len(path) - 1):
        n1 = path[i]
        n2 = path[i + 1]

        x1, y1 = get_xy(graph[n1])
        x2, y2 = get_xy(graph[n2])

        axis.plot(
            [x1, x2],
            [y1, y2],
            color="blue",
            linewidth=3,
            zorder=5,
            label="Path" if i == 0 else ""
        )

def main():
    import matplotlib.pyplot as plt

    parser = argparse.ArgumentParser()
    parser.add_argument("--input", default="map_cache.txt")
    args = parser.parse_args()

    # --- load graph using YOUR function ---
    input_path = resolve_path(args.input)
    graph = parse_waypoint_graph(input_path)

    fig, ax = plt.subplots(figsize=(10, 6))

    # Draw background graph
    draw_background_graph(ax, graph)

    # Pick start/goal
    start_id, goal_id = pick_start_and_goal(graph)

    print("Start ID:", start_id)
    print("Goal ID:", goal_id)

    # Plot START
    if start_id is not None:
        sx, sy = get_xy(graph[start_id])
        ax.scatter(
            sx, sy,
            color="lime",
            s=120,
            edgecolors="black",
            linewidths=1.5,
            zorder=5,
            label="Start"
        )

    # Plot GOAL
    if goal_id is not None:
        gx, gy = get_xy(graph[goal_id])
        ax.scatter(
            gx, gy,
            color="red",
            s=120,
            edgecolors="black",
            linewidths=1.5,
            zorder=5,
            label="Goal"
        )

    path = extract_path(graph, start_id, goal_id)
    if path:
        print("Path:", path)
        draw_path(ax, graph, path)

        anim = animate_agent(ax, graph, path)

        # Styling
        ax.set_aspect("equal")
        ax.set_title("Map Cache Visualization")

        # Only show legend if something exists
        handles, labels = ax.get_legend_handles_labels()
        if handles:
            ax.legend(loc="best", fontsize=8, framealpha=0.9)

        plt.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)
        plt.show()
    else:
        print("No path found")

if __name__ == "__main__":
    main()

