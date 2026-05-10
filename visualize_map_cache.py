import argparse
import ast
import heapq
import json
import math
import os
from pathlib import Path


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
    for key in ("route", "explored", "obstacles"):
        parsed[key] = [int(waypoint_id) for waypoint_id in payload.get(key, [])]
    for key in ("start", "goal"):
        if payload.get(key) is not None:
            parsed[key] = int(payload[key])

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


def draw_waypoint_marker(axis, graph, waypoint_id, color, label, marker):
    location = graph.get(waypoint_id, {}).get("location")
    if location is None:
        return
    axis.scatter(
        [location[0]],
        [location[1]],
        s=90,
        c=color,
        marker=marker,
        edgecolors="white",
        linewidths=1.2,
        zorder=6,
        label=label,
    )
    axis.annotate(
        f"{label}\n{waypoint_id}",
        xy=(location[0], location[1]),
        xytext=(8, 8),
        textcoords="offset points",
        fontsize=8,
        color=color,
        weight="bold",
        zorder=7,
    )


def save_plot(
    graph,
    output_path,
    dpi,
    show_nodes,
    route_ids=None,
    explored_ids=None,
    obstacle_ids=None,
    start_id=None,
    goal_id=None,
):
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection

    locations, segments = build_line_segments(graph)
    if not locations:
        raise ValueError("No waypoint locations found in cache file.")

    route_ids = route_ids or []
    explored_ids = explored_ids or []
    obstacle_ids = obstacle_ids or []
    route_segments = build_segments_for_ids(graph, route_ids)
    explored_segments = build_explored_segments(graph, explored_ids)

    figure, axis = plt.subplots(figsize=(14, 14), dpi=dpi)
    axis.set_aspect("equal", adjustable="box")
    axis.set_facecolor("#f7f8fa")
    figure.patch.set_facecolor("white")

    if segments:
        line_collection = LineCollection(
            segments,
            colors="#9ca3af",
            linewidths=0.32,
            alpha=0.34,
            label="possible graph edges",
        )
        axis.add_collection(line_collection)

    if explored_segments:
        explored_collection = LineCollection(
            explored_segments,
            colors="#f59e0b",
            linewidths=1.0,
            alpha=0.55,
            label="explored route space",
            zorder=3,
        )
        axis.add_collection(explored_collection)

    if route_segments:
        route_collection = LineCollection(
            route_segments,
            colors="#dc2626",
            linewidths=2.8,
            alpha=0.95,
            label="chosen route",
            zorder=5,
        )
        axis.add_collection(route_collection)

    if show_nodes:
        xs = [location[0] for location in locations.values()]
        ys = [location[1] for location in locations.values()]
        axis.scatter(xs, ys, s=1.2, c="#111827", alpha=0.25, linewidths=0, zorder=2)

    if route_ids:
        route_locations = [graph[waypoint_id]["location"] for waypoint_id in route_ids if waypoint_id in graph]
        if route_locations:
            axis.scatter(
                [location[0] for location in route_locations],
                [location[1] for location in route_locations],
                s=12,
                c="#dc2626",
                alpha=0.9,
                linewidths=0,
                zorder=6,
            )

    if obstacle_ids:
        obstacle_locations = [graph[waypoint_id]["location"] for waypoint_id in obstacle_ids if waypoint_id in graph]
        if obstacle_locations:
            axis.scatter(
                [location[0] for location in obstacle_locations],
                [location[1] for location in obstacle_locations],
                s=45,
                c="#111827",
                marker="x",
                linewidths=1.7,
                zorder=7,
                label="obstacle",
            )

    if start_id is not None:
        draw_waypoint_marker(axis, graph, start_id, "#16a34a", "START", "o")
    if goal_id is not None:
        draw_waypoint_marker(axis, graph, goal_id, "#be123c", "GOAL", "*")

    axis.autoscale()
    axis.margins(0.03)
    axis.set_title(
        f"CARLA Waypoint Graph ({len(locations)} nodes, {len(segments)} directed edges, {len(route_ids)} route nodes)",
        fontsize=13,
        pad=12,
    )
    axis.set_xlabel("x")
    axis.set_ylabel("y")
    axis.grid(color="#d6dbe1", linewidth=0.35, alpha=0.5)
    axis.legend(loc="best", fontsize=8, framealpha=0.88)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, bbox_inches="tight")
    plt.close(figure)


def save_svg(
    graph,
    output_path,
    show_nodes,
    route_ids=None,
    explored_ids=None,
    obstacle_ids=None,
    start_id=None,
    goal_id=None,
):
    locations, segments = build_line_segments(graph)
    if not locations:
        raise ValueError("No waypoint locations found in cache file.")

    route_ids = route_ids or []
    explored_ids = explored_ids or []
    obstacle_ids = obstacle_ids or []
    route_segments = build_segments_for_ids(graph, route_ids)
    explored_segments = build_explored_segments(graph, explored_ids)

    xs = [location[0] for location in locations.values()]
    ys = [location[1] for location in locations.values()]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    width = max(max_x - min_x, 1.0)
    height = max(max_y - min_y, 1.0)
    canvas_size = 1200
    padding = 60
    scale = min((canvas_size - 2 * padding) / width, (canvas_size - 2 * padding) / height)

    def project(point):
        x, y = point
        projected_x = padding + (x - min_x) * scale
        projected_y = canvas_size - padding - (y - min_y) * scale
        return projected_x, projected_y

    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, "w", encoding="utf-8") as svg_file:
        svg_file.write(
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{canvas_size}" '
            f'height="{canvas_size}" viewBox="0 0 {canvas_size} {canvas_size}">\n'
        )
        svg_file.write('<rect width="100%" height="100%" fill="#f7f8fa"/>\n')
        svg_file.write(
            f'<text x="40" y="34" fill="#111827" font-family="Arial, sans-serif" '
            f'font-size="20">CARLA Waypoint Graph ({len(locations)} nodes, '
            f'{len(segments)} directed edges, {len(route_ids)} route nodes)</text>\n'
        )

        for start, end in segments:
            x1, y1 = project(start)
            x2, y2 = project(end)
            svg_file.write(
                f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
                'stroke="#9ca3af" stroke-width="1.2" stroke-opacity="0.34"/>\n'
            )

        for start, end in explored_segments:
            x1, y1 = project(start)
            x2, y2 = project(end)
            svg_file.write(
                f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
                'stroke="#f59e0b" stroke-width="3" stroke-opacity="0.55"/>\n'
            )

        for start, end in route_segments:
            x1, y1 = project(start)
            x2, y2 = project(end)
            svg_file.write(
                f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
                'stroke="#dc2626" stroke-width="6" stroke-opacity="0.95"/>\n'
            )

        if show_nodes:
            for location in locations.values():
                x, y = project((location[0], location[1]))
                svg_file.write(
                    f'<circle cx="{x:.2f}" cy="{y:.2f}" r="4" '
                    'fill="#111827" fill-opacity="0.72"/>\n'
                )

        for waypoint_id in route_ids:
            location = graph.get(waypoint_id, {}).get("location")
            if location is None:
                continue
            x, y = project((location[0], location[1]))
            svg_file.write(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="4" fill="#dc2626"/>\n')

        for waypoint_id in obstacle_ids:
            location = graph.get(waypoint_id, {}).get("location")
            if location is None:
                continue
            x, y = project((location[0], location[1]))
            svg_file.write(
                f'<text x="{x - 5:.2f}" y="{y + 5:.2f}" fill="#111827" '
                'font-family="Arial, sans-serif" font-size="18" font-weight="700">x</text>\n'
            )

        for waypoint_id, color, label in (
            (start_id, "#16a34a", "START"),
            (goal_id, "#be123c", "GOAL"),
        ):
            if waypoint_id is None:
                continue
            location = graph.get(waypoint_id, {}).get("location")
            if location is None:
                continue
            x, y = project((location[0], location[1]))
            svg_file.write(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="9" fill="{color}" stroke="white" stroke-width="2"/>\n')
            svg_file.write(
                f'<text x="{x + 12:.2f}" y="{y - 10:.2f}" fill="{color}" '
                f'font-family="Arial, sans-serif" font-size="16" font-weight="700">{label} {waypoint_id}</text>\n'
            )

        svg_file.write('<rect x="40" y="55" width="236" height="82" rx="6" fill="white" fill-opacity="0.86" stroke="#d1d5db"/>\n')
        svg_file.write('<line x1="58" y1="78" x2="108" y2="78" stroke="#9ca3af" stroke-width="2" stroke-opacity="0.5"/>\n')
        svg_file.write('<text x="120" y="83" fill="#374151" font-family="Arial, sans-serif" font-size="14">possible graph edges</text>\n')
        svg_file.write('<line x1="58" y1="102" x2="108" y2="102" stroke="#f59e0b" stroke-width="4" stroke-opacity="0.65"/>\n')
        svg_file.write('<text x="120" y="107" fill="#374151" font-family="Arial, sans-serif" font-size="14">explored route space</text>\n')
        svg_file.write('<line x1="58" y1="126" x2="108" y2="126" stroke="#dc2626" stroke-width="6"/>\n')
        svg_file.write('<text x="120" y="131" fill="#374151" font-family="Arial, sans-serif" font-size="14">chosen route</text>\n')

        svg_file.write("</svg>\n")


def resolve_visualization_inputs(graph, args):
    route_payload = parse_id_payload(args.route) if args.route else {}
    explored_payload = parse_id_payload(args.explored) if args.explored else {}
    obstacle_payload = parse_id_payload(args.obstacles) if args.obstacles else {}

    route_ids = route_payload.get("route", [])
    explored_ids = explored_payload.get("route", []) or explored_payload.get("explored", [])
    obstacle_ids = obstacle_payload.get("route", []) or obstacle_payload.get("obstacles", [])

    start_id = args.start_id or route_payload.get("start")
    goal_id = args.goal_id or route_payload.get("goal")

    if args.start_location:
        start_id = closest_waypoint_id(graph, parse_location(args.start_location))
    if args.goal_location:
        goal_id = closest_waypoint_id(graph, parse_location(args.goal_location))

    if route_ids and start_id is None:
        start_id = route_ids[0]
    if route_ids and goal_id is None:
        goal_id = route_ids[-1]

    if not route_ids and start_id is not None and goal_id is not None:
        route_ids = find_shortest_route(graph, start_id, goal_id, blocked_ids=obstacle_ids)
        if not route_ids:
            print(f"No route found from {start_id} to {goal_id}.")

    return route_ids, explored_ids, obstacle_ids, start_id, goal_id


def main():
    parser = argparse.ArgumentParser(description="Visualize Generate_map.py map_cache.txt output.")
    parser.add_argument("--input", default="map_cache.txt", help="Path to map_cache.txt")
    parser.add_argument("--output", default="map_cache_graph.png", help="Output image path, such as .png or .svg")
    parser.add_argument("--dpi", default=220, type=int, help="Output DPI for raster images")
    parser.add_argument("--hide-nodes", action="store_true", help="Draw only edges")
    parser.add_argument("--route", help="Route file from D* Lite (.json) or whitespace/comma-separated waypoint IDs")
    parser.add_argument("--explored", help="Explored waypoint IDs file (.json or plain IDs)")
    parser.add_argument("--obstacles", help="Obstacle waypoint IDs file (.json or plain IDs)")
    parser.add_argument("--start-id", type=int, help="Start waypoint ID")
    parser.add_argument("--goal-id", type=int, help="Goal waypoint ID")
    parser.add_argument("--start-location", help="Start location as x,y or x,y,z; nearest waypoint is used")
    parser.add_argument("--goal-location", help="Goal location as x,y or x,y,z; nearest waypoint is used")
    args = parser.parse_args()

    input_path = resolve_path(args.input)
    output_path = resolve_path(args.output)

    if not input_path.exists():
        raise FileNotFoundError(f"Could not find input cache file: {input_path}")

    graph = parse_waypoint_graph(input_path)
    route_ids, explored_ids, obstacle_ids, start_id, goal_id = resolve_visualization_inputs(graph, args)
    if output_path.suffix.lower() == ".svg":
        save_svg(
            graph,
            output_path,
            show_nodes=not args.hide_nodes,
            route_ids=route_ids,
            explored_ids=explored_ids,
            obstacle_ids=obstacle_ids,
            start_id=start_id,
            goal_id=goal_id,
        )
    else:
        save_plot(
            graph,
            output_path,
            args.dpi,
            show_nodes=not args.hide_nodes,
            route_ids=route_ids,
            explored_ids=explored_ids,
            obstacle_ids=obstacle_ids,
            start_id=start_id,
            goal_id=goal_id,
        )
    print(f"Saved graph visualization to {output_path}")


if __name__ == "__main__":
    main()
