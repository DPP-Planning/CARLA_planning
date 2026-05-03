import argparse
import ast
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


def save_plot(graph, output_path, dpi, show_nodes):
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection

    locations, segments = build_line_segments(graph)
    if not locations:
        raise ValueError("No waypoint locations found in cache file.")

    figure, axis = plt.subplots(figsize=(14, 14), dpi=dpi)
    axis.set_aspect("equal", adjustable="box")
    axis.set_facecolor("#f7f8fa")
    figure.patch.set_facecolor("white")

    if segments:
        line_collection = LineCollection(
            segments,
            colors="#2f6f9f",
            linewidths=0.35,
            alpha=0.65,
        )
        axis.add_collection(line_collection)

    if show_nodes:
        xs = [location[0] for location in locations.values()]
        ys = [location[1] for location in locations.values()]
        axis.scatter(xs, ys, s=1.2, c="#111827", alpha=0.55, linewidths=0)

    axis.autoscale()
    axis.margins(0.03)
    axis.set_title(
        f"CARLA Waypoint Graph ({len(locations)} nodes, {len(segments)} directed edges)",
        fontsize=13,
        pad=12,
    )
    axis.set_xlabel("x")
    axis.set_ylabel("y")
    axis.grid(color="#d6dbe1", linewidth=0.35, alpha=0.5)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, bbox_inches="tight")
    plt.close(figure)


def save_svg(graph, output_path, show_nodes):
    locations, segments = build_line_segments(graph)
    if not locations:
        raise ValueError("No waypoint locations found in cache file.")

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
            f'{len(segments)} directed edges)</text>\n'
        )

        for start, end in segments:
            x1, y1 = project(start)
            x2, y2 = project(end)
            svg_file.write(
                f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
                'stroke="#2f6f9f" stroke-width="2" stroke-opacity="0.68"/>\n'
            )

        if show_nodes:
            for location in locations.values():
                x, y = project((location[0], location[1]))
                svg_file.write(
                    f'<circle cx="{x:.2f}" cy="{y:.2f}" r="4" '
                    'fill="#111827" fill-opacity="0.72"/>\n'
                )

        svg_file.write("</svg>\n")


def main():
    parser = argparse.ArgumentParser(description="Visualize Generate_map.py map_cache.txt output.")
    parser.add_argument("--input", default="map_cache.txt", help="Path to map_cache.txt")
    parser.add_argument("--output", default="map_cache_graph.png", help="Output image path, such as .png or .svg")
    parser.add_argument("--dpi", default=220, type=int, help="Output DPI for raster images")
    parser.add_argument("--hide-nodes", action="store_true", help="Draw only edges")
    args = parser.parse_args()

    input_path = resolve_path(args.input)
    output_path = resolve_path(args.output)

    if not input_path.exists():
        raise FileNotFoundError(f"Could not find input cache file: {input_path}")

    graph = parse_waypoint_graph(input_path)
    if output_path.suffix.lower() == ".svg":
        save_svg(graph, output_path, show_nodes=not args.hide_nodes)
    else:
        save_plot(graph, output_path, args.dpi, show_nodes=not args.hide_nodes)
    print(f"Saved graph visualization to {output_path}")


if __name__ == "__main__":
    main()
