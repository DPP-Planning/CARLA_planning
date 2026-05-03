import argparse
import ast
import json
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent


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
      gap: 16px;
      padding: 14px 18px;
      background: rgba(255, 255, 255, 0.88);
      border-bottom: 1px solid rgba(17, 24, 39, 0.12);
      backdrop-filter: blur(10px);
    }

    h1 {
      margin: 0;
      font-size: 18px;
      font-weight: 700;
      letter-spacing: 0;
    }

    .stats {
      font-size: 13px;
      color: #4b5563;
      white-space: nowrap;
    }

    .controls {
      display: flex;
      align-items: center;
      gap: 10px;
      font-size: 13px;
      color: #374151;
    }

    label {
      display: inline-flex;
      align-items: center;
      gap: 6px;
    }

    input[type="range"] {
      width: 130px;
    }

    input[type="checkbox"] {
      margin: 0;
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
  </style>
</head>
<body>
  <header>
    <div>
      <h1>CARLA Waypoint Graph 3D</h1>
      <div class="stats">__NODE_COUNT__ nodes, __EDGE_COUNT__ directed edges</div>
    </div>
    <div class="controls">
      <label>z scale <input id="zScale" type="range" min="1" max="40" value="8"></label>
      <label><input id="showZLabels" type="checkbox" checked> z labels</label>
      <button id="resetView" type="button">Reset</button>
    </div>
  </header>
  <canvas id="scene"></canvas>
  <script>
    const graphData = __GRAPH_JSON__;
    const canvas = document.getElementById("scene");
    const ctx = canvas.getContext("2d");
    const zScaleInput = document.getElementById("zScale");
    const showZLabelsInput = document.getElementById("showZLabels");
    const resetButton = document.getElementById("resetView");

    const points = graphData.points;
    const edges = graphData.edges;
    const bounds = graphData.bounds;
    const displayedBounds = {
      ...bounds,
      minZ: graphData.displayedZBounds.minZ,
      maxZ: graphData.displayedZBounds.maxZ,
    };
    const center = {
      x: (bounds.minX + bounds.maxX) / 2,
      y: (bounds.minY + bounds.maxY) / 2,
      z: (displayedBounds.minZ + displayedBounds.maxZ) / 2,
    };

    let state = {
      yaw: -0.78,
      pitch: 0.72,
      zoom: 1,
      zScale: Number(zScaleInput.value),
      panX: 0,
      panY: 20,
      dragging: false,
      lastX: 0,
      lastY: 0,
    };

    function resetView() {
      state.yaw = -0.78;
      state.pitch = 0.72;
      state.zoom = 1;
      state.panX = 0;
      state.panY = 20;
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
      const pitchY = yawY * cosP - z * sinP;
      const pitchZ = yawY * sinP + z * cosP;

      return { x: yawX, y: pitchY, z: pitchZ };
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
      const maxSpan = Math.max(width, height, depth);
      return Math.min(window.innerWidth, window.innerHeight) * 0.68 / maxSpan * state.zoom;
    }

    function niceStep(span, targetTicks) {
      const rawStep = span / Math.max(targetTicks, 1);
      const magnitude = Math.pow(10, Math.floor(Math.log10(Math.max(rawStep, 0.000001))));
      const normalized = rawStep / magnitude;
      if (normalized <= 1) return magnitude;
      if (normalized <= 2) return 2 * magnitude;
      if (normalized <= 5) return 5 * magnitude;
      return 10 * magnitude;
    }

    function drawLabel(text, point, fillStyle = "#374151", align = "center") {
      ctx.save();
      ctx.font = "12px Arial, sans-serif";
      ctx.textAlign = align;
      ctx.textBaseline = "middle";
      ctx.lineWidth = 4;
      ctx.strokeStyle = "rgba(245, 247, 250, 0.92)";
      ctx.strokeText(text, point.x, point.y);
      ctx.fillStyle = fillStyle;
      ctx.fillText(text, point.x, point.y);
      ctx.restore();
    }

    function drawGrid(scale) {
      const minX = Math.floor(bounds.minX / 10) * 10;
      const maxX = Math.ceil(bounds.maxX / 10) * 10;
      const minY = Math.floor(bounds.minY / 10) * 10;
      const maxY = Math.ceil(bounds.maxY / 10) * 10;
      ctx.lineWidth = 1;
      ctx.strokeStyle = "rgba(148, 163, 184, 0.28)";

      for (let x = minX; x <= maxX; x += 10) {
        const a = project({ x, y: minY, z: displayedBounds.minZ }, scale);
        const b = project({ x, y: maxY, z: displayedBounds.minZ }, scale);
        ctx.beginPath();
        ctx.moveTo(a.x, a.y);
        ctx.lineTo(b.x, b.y);
        ctx.stroke();
      }

      for (let y = minY; y <= maxY; y += 10) {
        const a = project({ x: minX, y, z: displayedBounds.minZ }, scale);
        const b = project({ x: maxX, y, z: displayedBounds.minZ }, scale);
        ctx.beginPath();
        ctx.moveTo(a.x, a.y);
        ctx.lineTo(b.x, b.y);
        ctx.stroke();
      }
    }

    function drawAxes(scale) {
      const xStart = { x: bounds.minX, y: bounds.minY, z: displayedBounds.minZ };
      const xEnd = { x: bounds.maxX, y: bounds.minY, z: displayedBounds.minZ };
      const yStart = { x: bounds.minX, y: bounds.minY, z: displayedBounds.minZ };
      const yEnd = { x: bounds.minX, y: bounds.maxY, z: displayedBounds.minZ };
      const zStart = { x: bounds.minX, y: bounds.minY, z: displayedBounds.minZ };
      const zEnd = { x: bounds.minX, y: bounds.minY, z: displayedBounds.maxZ };

      const axes = [
        { start: xStart, end: xEnd, color: "#b91c1c", label: "x" },
        { start: yStart, end: yEnd, color: "#047857", label: "y" },
        { start: zStart, end: zEnd, color: "#6d28d9", label: "z" },
      ];

      ctx.lineCap = "round";
      for (const axis of axes) {
        const start = project(axis.start, scale);
        const end = project(axis.end, scale);
        ctx.strokeStyle = axis.color;
        ctx.lineWidth = 2.4;
        ctx.beginPath();
        ctx.moveTo(start.x, start.y);
        ctx.lineTo(end.x, end.y);
        ctx.stroke();
        drawLabel(axis.label, { x: end.x + 14, y: end.y - 8 }, axis.color, "left");
      }

      const xStep = niceStep(bounds.maxX - bounds.minX, 5);
      for (let x = Math.ceil(bounds.minX / xStep) * xStep; x <= bounds.maxX + 0.0001; x += xStep) {
        const tick = project({ x, y: bounds.minY, z: displayedBounds.minZ }, scale);
        drawLabel(Number(x.toFixed(2)).toString(), { x: tick.x, y: tick.y + 16 }, "#7f1d1d");
      }

      const yStep = niceStep(bounds.maxY - bounds.minY, 5);
      for (let y = Math.ceil(bounds.minY / yStep) * yStep; y <= bounds.maxY + 0.0001; y += yStep) {
        const tick = project({ x: bounds.minX, y, z: displayedBounds.minZ }, scale);
        drawLabel(Number(y.toFixed(2)).toString(), { x: tick.x - 10, y: tick.y + 14 }, "#064e3b", "right");
      }

      const zStep = niceStep(displayedBounds.maxZ - displayedBounds.minZ, 5);
      for (let z = Math.ceil(displayedBounds.minZ / zStep) * zStep; z <= displayedBounds.maxZ + 0.0001; z += zStep) {
        const tick = project({ x: bounds.minX, y: bounds.minY, z }, scale);
        ctx.strokeStyle = "#6d28d9";
        ctx.lineWidth = 1.4;
        ctx.beginPath();
        ctx.moveTo(tick.x - 5, tick.y);
        ctx.lineTo(tick.x + 5, tick.y);
        ctx.stroke();
        drawLabel(`z=${Number(z.toFixed(2))}`, { x: tick.x + 12, y: tick.y }, "#4c1d95", "left");
      }
    }

    function draw() {
      ctx.clearRect(0, 0, window.innerWidth, window.innerHeight);
      ctx.fillStyle = "#f5f7fa";
      ctx.fillRect(0, 0, window.innerWidth, window.innerHeight);

      const scale = computeScale();
      drawGrid(scale);
      drawAxes(scale);

      const projected = new Map(points.map((point) => [point.id, project(point, scale)]));
      const sortedEdges = edges.map((edge) => {
        const a = projected.get(edge[0]);
        const b = projected.get(edge[1]);
        return { a, b, depth: (a.depth + b.depth) / 2 };
      }).sort((left, right) => left.depth - right.depth);

      ctx.lineCap = "round";
      for (const edge of sortedEdges) {
        ctx.strokeStyle = "rgba(47, 111, 159, 0.72)";
        ctx.lineWidth = 1.8;
        ctx.beginPath();
        ctx.moveTo(edge.a.x, edge.a.y);
        ctx.lineTo(edge.b.x, edge.b.y);
        ctx.stroke();
      }

      const sortedPoints = [...points].map((point) => {
        const screen = projected.get(point.id);
        return { point, screen };
      }).sort((left, right) => left.screen.depth - right.screen.depth);

      for (const item of sortedPoints) {
        ctx.fillStyle = "#111827";
        ctx.globalAlpha = 0.75;
        ctx.beginPath();
        ctx.arc(item.screen.x, item.screen.y, 3.5, 0, Math.PI * 2);
        ctx.fill();
      }
      ctx.globalAlpha = 1;

      if (showZLabelsInput.checked && points.length <= 150) {
        for (const item of sortedPoints) {
          drawLabel(
            `z=${Number(item.point.z.toFixed(2))}`,
            { x: item.screen.x + 7, y: item.screen.y - 9 },
            "#4c1d95",
            "left"
          );
        }
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
      const delta = Math.sign(event.deltaY);
      state.zoom = Math.max(0.2, Math.min(8, state.zoom * (delta > 0 ? 0.9 : 1.1)));
      draw();
    }, { passive: false });

    zScaleInput.addEventListener("input", () => {
      state.zScale = Number(zScaleInput.value);
      draw();
    });

    showZLabelsInput.addEventListener("change", draw);
    resetButton.addEventListener("click", resetView);
    window.addEventListener("resize", resize);
    resize();
  </script>
</body>
</html>
"""


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


def build_graph_data(graph):
    points = []
    edges = []

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

        for successor_id in data.get("successors", []):
            if successor_id in graph:
                edges.append([waypoint_id, successor_id])

    if not points:
        raise ValueError("No waypoint locations found in cache file.")

    min_z = min(point["z"] for point in points)
    max_z = max(point["z"] for point in points)
    if min_z == max_z:
        z_padding = 5.0
    else:
        z_padding = (max_z - min_z) * 0.12

    return {
        "points": points,
        "edges": edges,
        "bounds": {
            "minX": min(point["x"] for point in points),
            "maxX": max(point["x"] for point in points),
            "minY": min(point["y"] for point in points),
            "maxY": max(point["y"] for point in points),
            "minZ": min_z,
            "maxZ": max_z,
        },
        "displayedZBounds": {
            "minZ": min_z - z_padding,
            "maxZ": max_z + z_padding,
        },
    }


def save_html(graph_data, output_path):
    html = HTML_TEMPLATE
    html = html.replace("__NODE_COUNT__", str(len(graph_data["points"])))
    html = html.replace("__EDGE_COUNT__", str(len(graph_data["edges"])))
    html = html.replace("__GRAPH_JSON__", json.dumps(graph_data))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as output_file:
        output_file.write(html)


def main():
    parser = argparse.ArgumentParser(description="Create an interactive 3D view of map_cache.txt.")
    parser.add_argument("--input", default="map_cache.txt", help="Path to map_cache.txt")
    parser.add_argument("--output", default="map_cache_graph_3d.html", help="Output HTML path")
    args = parser.parse_args()

    input_path = resolve_path(args.input)
    output_path = resolve_path(args.output)

    if not input_path.exists():
        raise FileNotFoundError(f"Could not find input cache file: {input_path}")

    graph = parse_waypoint_graph(input_path)
    graph_data = build_graph_data(graph)
    save_html(graph_data, output_path)
    print(f"Saved 3D graph visualization to {output_path}")


if __name__ == "__main__":
    main()
