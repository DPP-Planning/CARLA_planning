import argparse
import json

from visualize_map_cache import (
    parse_waypoint_graph,
    print_waypoint_summary,
    resolve_path,
    resolve_visualization_inputs,
)


HTML_TEMPLATE = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>CARLA Waypoint Graph 3D</title>
  <style>
    :root {
      color-scheme: light;
      font-family: Arial, sans-serif;
    }

    body {
      margin: 0;
      background: #f5f7fa;
      color: #111827;
      overflow: hidden;
    }

    header {
      position: fixed;
      top: 0;
      left: 0;
      right: 0;
      z-index: 2;
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 18px;
      padding: 12px 16px;
      background: rgba(255, 255, 255, 0.9);
      border-bottom: 1px solid rgba(17, 24, 39, 0.12);
      backdrop-filter: blur(10px);
    }

    h1 {
      margin: 0;
      font-size: 18px;
      line-height: 1.2;
    }

    .stats {
      margin-top: 3px;
      font-size: 13px;
      color: #4b5563;
    }

    .controls {
      display: flex;
      align-items: center;
      flex-wrap: wrap;
      justify-content: flex-end;
      gap: 10px 14px;
      font-size: 13px;
      color: #374151;
    }

    label {
      display: inline-flex;
      align-items: center;
      gap: 6px;
      white-space: nowrap;
    }

    input[type="range"] {
      width: 120px;
    }

    button {
      border: 1px solid #cbd5e1;
      border-radius: 6px;
      background: #ffffff;
      color: #111827;
      padding: 7px 10px;
      font: inherit;
      cursor: pointer;
    }

    button:hover {
      background: #f1f5f9;
    }

    canvas {
      display: block;
      width: 100vw;
      height: 100vh;
      cursor: grab;
    }

    canvas:active {
      cursor: grabbing;
    }

    .legend {
      position: fixed;
      left: 16px;
      bottom: 16px;
      z-index: 2;
      width: 230px;
      padding: 10px 12px;
      background: rgba(255, 255, 255, 0.9);
      border: 1px solid rgba(17, 24, 39, 0.12);
      border-radius: 8px;
      font-size: 13px;
      color: #374151;
      backdrop-filter: blur(10px);
    }

    .legend-row {
      display: flex;
      align-items: center;
      gap: 8px;
      margin: 6px 0;
    }

    .swatch {
      width: 34px;
      height: 0;
      border-top: 4px solid var(--color);
    }

    .swatch.dashed {
      border-top-style: dashed;
    }

    .swatch.dotdash {
      border-top-style: dashed;
    }
  </style>
</head>
<body>
  <header>
    <div>
      <h1>CARLA Waypoint Graph 3D</h1>
      <div class="stats">__NODE_COUNT__ nodes, __EDGE_COUNT__ graph edges, __ROUTE_COUNT__ actual-route nodes, __OBSTACLE_COUNT__ obstacles</div>
    </div>
    <div class="controls">
      <label>z scale <input id="zScale" type="range" min="1" max="45" value="8"></label>
      <label><input id="showGraph" type="checkbox" checked> graph</label>
      <label><input id="showExplored" type="checkbox" checked> explored</label>
      <label><input id="showPlans" type="checkbox" checked> plans</label>
      <label><input id="showLabels" type="checkbox" checked> labels</label>
      <button id="resetView" type="button">Reset</button>
    </div>
  </header>
  <canvas id="scene"></canvas>
  <div class="legend">
    <div class="legend-row"><span class="swatch" style="--color:#9ca3af"></span> possible graph edges</div>
    <div class="legend-row"><span class="swatch" style="--color:#f59e0b"></span> explored space</div>
    <div class="legend-row"><span class="swatch dashed" style="--color:#2563eb"></span> initial planned route</div>
    <div class="legend-row"><span class="swatch dotdash" style="--color:#7c3aed"></span> D* Lite reroute</div>
    <div class="legend-row"><span class="swatch" style="--color:#dc2626"></span> actual route taken</div>
    <div class="legend-row"><strong style="color:#111827">x</strong> obstacle waypoint</div>
  </div>
  <script>
    const data = __GRAPH_JSON__;
    const canvas = document.getElementById("scene");
    const ctx = canvas.getContext("2d");
    const zScaleInput = document.getElementById("zScale");
    const showGraphInput = document.getElementById("showGraph");
    const showExploredInput = document.getElementById("showExplored");
    const showPlansInput = document.getElementById("showPlans");
    const showLabelsInput = document.getElementById("showLabels");
    const resetButton = document.getElementById("resetView");

    const pointById = new Map(data.points.map((point) => [point.id, point]));
    const bounds = data.bounds;
    const displayedBounds = data.displayedBounds;
    const center = {
      x: (bounds.minX + bounds.maxX) / 2,
      y: (bounds.minY + bounds.maxY) / 2,
      z: (displayedBounds.minZ + displayedBounds.maxZ) / 2,
    };

    let state = {
      yaw: -0.78,
      pitch: 0.68,
      zoom: 1,
      zScale: Number(zScaleInput.value),
      panX: 0,
      panY: 24,
      dragging: false,
      lastX: 0,
      lastY: 0,
    };

    function resetView() {
      state.yaw = -0.78;
      state.pitch = 0.68;
      state.zoom = 1;
      state.panX = 0;
      state.panY = 24;
      draw();
    }

    function resize() {
      const ratio = window.devicePixelRatio || 1;
      canvas.width = Math.floor(window.innerWidth * ratio);
      canvas.height = Math.floor(window.innerHeight * ratio);
      ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
      draw();
    }

    function rotate(point) {
      const x = point.x - center.x;
      const y = point.y - center.y;
      const z = (point.z - center.z) * state.zScale;
      const cosY = Math.cos(state.yaw);
      const sinY = Math.sin(state.yaw);
      const yawX = x * cosY - y * sinY;
      const yawY = x * sinY + y * cosY;
      const cosP = Math.cos(state.pitch);
      const sinP = Math.sin(state.pitch);
      return {
        x: yawX,
        y: yawY * cosP - z * sinP,
        z: yawY * sinP + z * cosP,
      };
    }

    function project(point, scale) {
      const rotated = rotate(point);
      return {
        x: window.innerWidth / 2 + rotated.x * scale + state.panX,
        y: window.innerHeight / 2 - rotated.y * scale + state.panY,
        depth: rotated.z,
      };
    }

    function computeScale() {
      const width = Math.max(bounds.maxX - bounds.minX, 1);
      const height = Math.max(bounds.maxY - bounds.minY, 1);
      const depth = Math.max((displayedBounds.maxZ - displayedBounds.minZ) * state.zScale, 1);
      return Math.min(window.innerWidth, window.innerHeight) * 0.66 / Math.max(width, height, depth) * state.zoom;
    }

    function drawText(text, screen, color = "#374151", align = "center") {
      ctx.save();
      ctx.font = "12px Arial, sans-serif";
      ctx.textAlign = align;
      ctx.textBaseline = "middle";
      ctx.lineWidth = 4;
      ctx.strokeStyle = "rgba(245, 247, 250, 0.94)";
      ctx.strokeText(text, screen.x, screen.y);
      ctx.fillStyle = color;
      ctx.fillText(text, screen.x, screen.y);
      ctx.restore();
    }

    function drawSegment(startPoint, endPoint, scale, color, width, alpha, dash = []) {
      const a = project(startPoint, scale);
      const b = project(endPoint, scale);
      ctx.save();
      ctx.globalAlpha = alpha;
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.lineCap = "round";
      ctx.setLineDash(dash);
      ctx.beginPath();
      ctx.moveTo(a.x, a.y);
      ctx.lineTo(b.x, b.y);
      ctx.stroke();
      ctx.restore();
      return (a.depth + b.depth) / 2;
    }

    function drawEdgeList(edges, scale, color, width, alpha, dash = []) {
      const drawable = [];
      for (const edge of edges) {
        const start = pointById.get(edge[0]);
        const end = pointById.get(edge[1]);
        if (!start || !end) continue;
        const a = project(start, scale);
        const b = project(end, scale);
        drawable.push({ start, end, depth: (a.depth + b.depth) / 2 });
      }
      drawable.sort((left, right) => left.depth - right.depth);
      for (const edge of drawable) {
        drawSegment(edge.start, edge.end, scale, color, width, alpha, dash);
      }
    }

    function routeToEdges(route) {
      const edges = [];
      for (let index = 0; index < route.length - 1; index += 1) {
        edges.push([route[index], route[index + 1]]);
      }
      return edges;
    }

    function drawGrid(scale) {
      const minX = Math.floor(bounds.minX / 10) * 10;
      const maxX = Math.ceil(bounds.maxX / 10) * 10;
      const minY = Math.floor(bounds.minY / 10) * 10;
      const maxY = Math.ceil(bounds.maxY / 10) * 10;
      for (let x = minX; x <= maxX; x += 10) {
        drawSegment({ x, y: minY, z: displayedBounds.minZ }, { x, y: maxY, z: displayedBounds.minZ }, scale, "#cbd5e1", 1, 0.35);
      }
      for (let y = minY; y <= maxY; y += 10) {
        drawSegment({ x: minX, y, z: displayedBounds.minZ }, { x: maxX, y, z: displayedBounds.minZ }, scale, "#cbd5e1", 1, 0.35);
      }
    }

    function drawAxes(scale) {
      const origin = { x: bounds.minX, y: bounds.minY, z: displayedBounds.minZ };
      const axes = [
        { end: { x: bounds.maxX, y: bounds.minY, z: displayedBounds.minZ }, color: "#b91c1c", label: "x" },
        { end: { x: bounds.minX, y: bounds.maxY, z: displayedBounds.minZ }, color: "#047857", label: "y" },
        { end: { x: bounds.minX, y: bounds.minY, z: displayedBounds.maxZ }, color: "#6d28d9", label: "z" },
      ];
      for (const axis of axes) {
        drawSegment(origin, axis.end, scale, axis.color, 2.4, 0.9);
        const labelPoint = project(axis.end, scale);
        drawText(axis.label, { x: labelPoint.x + 14, y: labelPoint.y - 8 }, axis.color, "left");
      }
      for (let z = displayedBounds.minZ; z <= displayedBounds.maxZ + 0.001; z += data.zTickStep) {
        const tick = project({ x: bounds.minX, y: bounds.minY, z }, scale);
        drawText(`z=${Number(z.toFixed(2))}`, { x: tick.x + 12, y: tick.y }, "#4c1d95", "left");
      }
    }

    function drawPoint(waypointId, scale, color, radius, label = null, marker = "circle") {
      const point = pointById.get(waypointId);
      if (!point) return;
      const screen = project(point, scale);
      ctx.save();
      ctx.fillStyle = color;
      ctx.strokeStyle = "white";
      ctx.lineWidth = 2;
      if (marker === "x") {
        ctx.strokeStyle = color;
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(screen.x - radius, screen.y - radius);
        ctx.lineTo(screen.x + radius, screen.y + radius);
        ctx.moveTo(screen.x + radius, screen.y - radius);
        ctx.lineTo(screen.x - radius, screen.y + radius);
        ctx.stroke();
      } else if (marker === "star") {
        ctx.beginPath();
        ctx.arc(screen.x, screen.y, radius + 2, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
      } else {
        ctx.beginPath();
        ctx.arc(screen.x, screen.y, radius, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
      }
      ctx.restore();
      if (label && showLabelsInput.checked) {
        drawText(label, { x: screen.x + 10, y: screen.y - 12 }, color, "left");
      }
    }

    function draw() {
      ctx.clearRect(0, 0, window.innerWidth, window.innerHeight);
      ctx.fillStyle = "#f5f7fa";
      ctx.fillRect(0, 0, window.innerWidth, window.innerHeight);
      const scale = computeScale();
      drawGrid(scale);
      drawAxes(scale);

      if (showGraphInput.checked) {
        drawEdgeList(data.edges, scale, "#9ca3af", 1.1, 0.28);
      }
      if (showExploredInput.checked) {
        drawEdgeList(data.exploredEdges, scale, "#f59e0b", 2.0, 0.5);
      }
      if (showPlansInput.checked) {
        for (let index = 0; index < data.plannedRoutes.length; index += 1) {
          const color = index === 0 ? "#2563eb" : "#7c3aed";
          drawEdgeList(routeToEdges(data.plannedRoutes[index].route), scale, color, 2.2, 0.72, [10, 7]);
        }
        for (const reroute of data.reroutes) {
          drawEdgeList(routeToEdges(reroute.route), scale, "#7c3aed", 3.0, 0.88, [13, 5, 4, 5]);
        }
      }

      drawEdgeList(routeToEdges(data.actualRoute), scale, "#dc2626", 4.2, 0.96);

      for (const point of data.points) {
        const screen = project(point, scale);
        ctx.fillStyle = "rgba(17, 24, 39, 0.32)";
        ctx.beginPath();
        ctx.arc(screen.x, screen.y, 2, 0, Math.PI * 2);
        ctx.fill();
      }

      for (const waypointId of data.actualRoute) {
        drawPoint(waypointId, scale, "#dc2626", 3.3);
      }
      for (const waypointId of data.obstacles) {
        drawPoint(waypointId, scale, "#111827", 8, `OBS ${waypointId}`, "x");
      }
      if (data.startId !== null) {
        drawPoint(data.startId, scale, "#16a34a", 7, `START ${data.startId}`);
      }
      if (data.goalId !== null) {
        drawPoint(data.goalId, scale, "#be123c", 8, `GOAL ${data.goalId}`, "star");
      }
    }

    canvas.addEventListener("pointerdown", (event) => {
      state.dragging = true;
      state.lastX = event.clientX;
      state.lastY = event.clientY;
      canvas.setPointerCapture(event.pointerId);
    });

    canvas.addEventListener("pointermove", (event) => {
      if (!state.dragging) return;
      const dx = event.clientX - state.lastX;
      const dy = event.clientY - state.lastY;
      state.lastX = event.clientX;
      state.lastY = event.clientY;
      state.yaw += dx * 0.008;
      state.pitch = Math.max(-1.35, Math.min(1.35, state.pitch + dy * 0.008));
      draw();
    });

    canvas.addEventListener("pointerup", () => {
      state.dragging = false;
    });

    canvas.addEventListener("wheel", (event) => {
      event.preventDefault();
      state.zoom = Math.max(0.2, Math.min(8, state.zoom * (event.deltaY > 0 ? 0.9 : 1.1)));
      draw();
    }, { passive: false });

    zScaleInput.addEventListener("input", () => {
      state.zScale = Number(zScaleInput.value);
      draw();
    });
    showGraphInput.addEventListener("change", draw);
    showExploredInput.addEventListener("change", draw);
    showPlansInput.addEventListener("change", draw);
    showLabelsInput.addEventListener("change", draw);
    resetButton.addEventListener("click", resetView);
    window.addEventListener("resize", resize);
    resize();
  </script>
</body>
</html>
"""


def edge_list_from_graph(graph):
    edges = []
    for waypoint_id, data in graph.items():
        for successor_id in data.get("successors", []):
            if successor_id in graph:
                edges.append([waypoint_id, successor_id])
    return edges


def route_to_edges(route_ids, graph):
    edges = []
    for index in range(len(route_ids) - 1):
        start_id = route_ids[index]
        end_id = route_ids[index + 1]
        if start_id in graph and end_id in graph:
            edges.append([start_id, end_id])
    return edges


def explored_edges(graph, explored_ids):
    explored_id_set = set(explored_ids)
    edges = []
    for waypoint_id in explored_id_set:
        if waypoint_id not in graph:
            continue
        for successor_id in graph[waypoint_id].get("successors", []):
            if successor_id in explored_id_set and successor_id in graph:
                edges.append([waypoint_id, successor_id])
    return edges


def nice_step(span, target_ticks=5):
    if span <= 0:
        return 1.0
    raw_step = span / max(target_ticks, 1)
    magnitude = 10 ** math_floor_log10(raw_step)
    normalized = raw_step / magnitude
    if normalized <= 1:
        return magnitude
    if normalized <= 2:
        return 2 * magnitude
    if normalized <= 5:
        return 5 * magnitude
    return 10 * magnitude


def math_floor_log10(value):
    exponent = 0
    if value <= 0:
        return 1
    while value >= 10:
        value /= 10
        exponent += 1
    while value < 1:
        value *= 10
        exponent -= 1
    return exponent


def build_graph_data(graph, args):
    (
        route_ids,
        explored_ids,
        obstacle_ids,
        planned_routes,
        reroutes,
        start_id,
        goal_id,
    ) = resolve_visualization_inputs(graph, args)

    points = []
    for waypoint_id, data in graph.items():
        location = data.get("location")
        if location is None:
            continue
        points.append(
            {
                "id": waypoint_id,
                "x": float(location[0]),
                "y": float(location[1]),
                "z": float(location[2]),
            }
        )

    if not points:
        raise ValueError("No waypoint locations found in cache file.")

    min_z = min(point["z"] for point in points)
    max_z = max(point["z"] for point in points)
    z_padding = 5.0 if min_z == max_z else (max_z - min_z) * 0.12
    displayed_min_z = min_z - z_padding
    displayed_max_z = max_z + z_padding

    return {
        "points": points,
        "edges": edge_list_from_graph(graph),
        "exploredEdges": explored_edges(graph, explored_ids),
        "actualRoute": route_ids,
        "plannedRoutes": planned_routes,
        "reroutes": reroutes,
        "obstacles": obstacle_ids,
        "startId": start_id,
        "goalId": goal_id,
        "bounds": {
            "minX": min(point["x"] for point in points),
            "maxX": max(point["x"] for point in points),
            "minY": min(point["y"] for point in points),
            "maxY": max(point["y"] for point in points),
            "minZ": min_z,
            "maxZ": max_z,
        },
        "displayedBounds": {
            "minZ": displayed_min_z,
            "maxZ": displayed_max_z,
        },
        "zTickStep": nice_step(displayed_max_z - displayed_min_z),
    }


def save_html(graph_data, output_path):
    html = HTML_TEMPLATE
    html = html.replace("__NODE_COUNT__", str(len(graph_data["points"])))
    html = html.replace("__EDGE_COUNT__", str(len(graph_data["edges"])))
    html = html.replace("__ROUTE_COUNT__", str(len(graph_data["actualRoute"])))
    html = html.replace("__OBSTACLE_COUNT__", str(len(graph_data["obstacles"])))
    html = html.replace("__GRAPH_JSON__", json.dumps(graph_data))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as output_file:
        output_file.write(html)


def main():
    parser = argparse.ArgumentParser(description="Create an interactive 3D visualization of map_cache.txt.")
    parser.add_argument("--input", default="map_cache.txt", help="Path to map_cache.txt")
    parser.add_argument("--output", default="map_cache_graph_3d.html", help="Output HTML path")
    parser.add_argument("--route", help="Route file from D* Lite (.json) or whitespace/comma-separated waypoint IDs")
    parser.add_argument("--explored", help="Explored waypoint IDs file (.json or plain IDs)")
    parser.add_argument("--obstacles", help="Obstacle waypoint IDs file (.json or plain IDs)")
    parser.add_argument("--start-id", type=int, help="Start waypoint ID")
    parser.add_argument("--goal-id", type=int, help="Goal waypoint ID")
    parser.add_argument("--start-location", help="Start location as x,y or x,y,z; nearest waypoint is used")
    parser.add_argument("--goal-location", help="Goal location as x,y or x,y,z; nearest waypoint is used")
    parser.add_argument("--list-waypoints", action="store_true", help="Print available waypoint IDs and exit")
    parser.add_argument("--list-limit", default=25, type=int, help="Number of waypoint IDs to print with --list-waypoints")
    args = parser.parse_args()

    input_path = resolve_path(args.input)
    output_path = resolve_path(args.output)
    if not input_path.exists():
        raise FileNotFoundError(f"Could not find input cache file: {input_path}")

    graph = parse_waypoint_graph(input_path)
    if args.list_waypoints:
        print_waypoint_summary(graph, args.list_limit)
        return

    graph_data = build_graph_data(graph, args)
    save_html(graph_data, output_path)
    print(f"Saved 3D graph visualization to {output_path}")


if __name__ == "__main__":
    main()
