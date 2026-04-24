import argparse
import math
import pickle
import sys
from collections import defaultdict
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import carla


DEFAULT_RESOLUTION = 1.0


@dataclass
class GeneratedMapData:
    resolution: float
    waypoint_graph: Dict[int, dict]
    waypoint_lookup: Dict[int, carla.Waypoint]
    all_waypoints: List[carla.Waypoint]
    wp_pts: Dict[int, int]
    start_waypoint: Optional[carla.Waypoint] = None
    goal_waypoint: Optional[carla.Waypoint] = None


def normalize_resolution(resolution: float) -> float:
    if resolution != DEFAULT_RESOLUTION:
        print(
            f"Requested resolution {resolution}m overridden to "
            f"{DEFAULT_RESOLUTION}m for waypoint-cache consistency."
        )
    return DEFAULT_RESOLUTION


def build_generated_waypoint_lookup(carla_map, resolution: float) -> Dict[int, carla.Waypoint]:
    waypoint_lookup = {}

    for waypoint in carla_map.generate_waypoints(resolution):
        waypoint_lookup[waypoint.id] = waypoint

    return waypoint_lookup


def get_location_key(location, bucket_size: float) -> Tuple[int, int, int]:
    return (
        math.floor(location.x / bucket_size),
        math.floor(location.y / bucket_size),
        math.floor(location.z / bucket_size),
    )


def build_spatial_index(waypoint_lookup: Dict[int, carla.Waypoint], bucket_size: float):
    spatial_index = defaultdict(list)

    for waypoint in waypoint_lookup.values():
        location_key = get_location_key(waypoint.transform.location, bucket_size)
        spatial_index[location_key].append(waypoint)

    return spatial_index


def get_closest_generated_waypoint(
    target_waypoint: Optional[carla.Waypoint],
    waypoint_lookup: Dict[int, carla.Waypoint],
    spatial_index,
    resolution: float,
) -> Optional[carla.Waypoint]:
    if target_waypoint is None:
        return None

    if target_waypoint.id in waypoint_lookup:
        return waypoint_lookup[target_waypoint.id]

    target_location = target_waypoint.transform.location
    base_key = get_location_key(target_location, resolution)
    closest_waypoint = None
    closest_distance = float("inf")

    for search_radius in range(3):
        for dx in range(-search_radius, search_radius + 1):
            for dy in range(-search_radius, search_radius + 1):
                for dz in range(-search_radius, search_radius + 1):
                    candidate_key = (base_key[0] + dx, base_key[1] + dy, base_key[2] + dz)
                    for candidate_waypoint in spatial_index.get(candidate_key, []):
                        candidate_distance = target_location.distance(candidate_waypoint.transform.location)
                        if candidate_distance < closest_distance:
                            closest_waypoint = candidate_waypoint
                            closest_distance = candidate_distance

        if closest_waypoint is not None:
            return closest_waypoint

    for candidate_waypoint in waypoint_lookup.values():
        candidate_distance = target_location.distance(candidate_waypoint.transform.location)
        if candidate_distance < closest_distance:
            closest_waypoint = candidate_waypoint
            closest_distance = candidate_distance

    return closest_waypoint


def get_driving_adjacent_lanes(waypoint: carla.Waypoint) -> List[carla.Waypoint]:
    adjacent_lanes = []

    if waypoint.lane_change & carla.LaneChange.Left:
        left_lane = waypoint.get_left_lane()
        if left_lane and left_lane.lane_type == carla.LaneType.Driving:
            adjacent_lanes.append(left_lane)

    if waypoint.lane_change & carla.LaneChange.Right:
        right_lane = waypoint.get_right_lane()
        if right_lane and right_lane.lane_type == carla.LaneType.Driving:
            adjacent_lanes.append(right_lane)

    return adjacent_lanes


def collect_neighbor_ids(
    neighbors: List[carla.Waypoint],
    waypoint_lookup: Dict[int, carla.Waypoint],
    spatial_index,
    resolution: float,
) -> List[int]:
    neighbor_ids = []
    seen_ids = set()

    for neighbor in neighbors:
        resolved_neighbor = get_closest_generated_waypoint(
            neighbor,
            waypoint_lookup,
            spatial_index,
            resolution,
        )
        if resolved_neighbor is None:
            continue

        neighbor_id = resolved_neighbor.id
        if neighbor_id in seen_ids:
            continue

        seen_ids.add(neighbor_id)
        neighbor_ids.append(neighbor_id)

    return neighbor_ids


def build_predecessor_ids(
    waypoint: carla.Waypoint,
    resolution: float,
    waypoint_lookup: Dict[int, carla.Waypoint],
    spatial_index,
) -> List[int]:
    predecessors = list(waypoint.previous(resolution))
    adjacent_lanes = get_driving_adjacent_lanes(waypoint)

    predecessors.extend(adjacent_lanes)

    for adjacent_lane in adjacent_lanes:
        predecessors.extend(adjacent_lane.previous(resolution))

    return collect_neighbor_ids(predecessors, waypoint_lookup, spatial_index, resolution)


def build_successor_ids(
    waypoint: carla.Waypoint,
    resolution: float,
    waypoint_lookup: Dict[int, carla.Waypoint],
    spatial_index,
) -> List[int]:
    successors = list(waypoint.next(resolution))
    adjacent_lanes = get_driving_adjacent_lanes(waypoint)

    successors.extend(adjacent_lanes)

    for adjacent_lane in adjacent_lanes:
        successors.extend(adjacent_lane.next(resolution))

    return collect_neighbor_ids(successors, waypoint_lookup, spatial_index, resolution)


def build_waypoint_graph(
    carla_map,
    resolution: float = DEFAULT_RESOLUTION,
) -> Tuple[Dict[int, dict], Dict[int, carla.Waypoint], dict]:
    resolution = normalize_resolution(resolution)
    waypoint_lookup = build_generated_waypoint_lookup(carla_map, resolution)
    spatial_index = build_spatial_index(waypoint_lookup, resolution)

    waypoint_graph = {}
    for waypoint_id, waypoint in waypoint_lookup.items():
        location = waypoint.transform.location
        waypoint_graph[waypoint_id] = {
            "location": (location.x, location.y, location.z),
            "g": float("inf"),
            "rhs": float("inf"),
            "predecessors": build_predecessor_ids(waypoint, resolution, waypoint_lookup, spatial_index),
            "successors": build_successor_ids(waypoint, resolution, waypoint_lookup, spatial_index),
        }

    return waypoint_graph, waypoint_lookup, spatial_index


def build_dlite_inputs(
    carla_map,
    resolution: float = DEFAULT_RESOLUTION,
    start_waypoint: Optional[carla.Waypoint] = None,
    goal_waypoint: Optional[carla.Waypoint] = None,
) -> GeneratedMapData:
    waypoint_graph, waypoint_lookup, spatial_index = build_waypoint_graph(carla_map, resolution)
    normalized_resolution = normalize_resolution(resolution)

    resolved_start = get_closest_generated_waypoint(
        start_waypoint,
        waypoint_lookup,
        spatial_index,
        normalized_resolution,
    )
    resolved_goal = get_closest_generated_waypoint(
        goal_waypoint,
        waypoint_lookup,
        spatial_index,
        normalized_resolution,
    )

    all_waypoints = list(waypoint_lookup.values())
    wp_pts = {waypoint.id: pos for pos, waypoint in enumerate(all_waypoints)}

    return GeneratedMapData(
        resolution=normalized_resolution,
        waypoint_graph=waypoint_graph,
        waypoint_lookup=waypoint_lookup,
        all_waypoints=all_waypoints,
        wp_pts=wp_pts,
        start_waypoint=resolved_start,
        goal_waypoint=resolved_goal,
    )


def connect_and_build_dlite_inputs(
    host: str = "127.0.0.1",
    port: int = 2000,
    resolution: float = DEFAULT_RESOLUTION,
    start_location=None,
    goal_location=None,
) -> GeneratedMapData:
    client = carla.Client(host, port)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()

    start_waypoint = carla_map.get_waypoint(start_location) if start_location is not None else None
    goal_waypoint = carla_map.get_waypoint(goal_location) if goal_location is not None else None

    return build_dlite_inputs(
        carla_map,
        resolution=resolution,
        start_waypoint=start_waypoint,
        goal_waypoint=goal_waypoint,
    )


def save_waypoint_graph(waypoint_graph: Dict[int, dict], output_path: str) -> None:
    if output_path.endswith(".txt"):
        with open(output_path, "w", encoding="utf-8") as output_file:
            for waypoint_id, data in waypoint_graph.items():
                output_file.write(f"waypoint_id: {waypoint_id}\n")
                output_file.write(f"location: {data['location']}\n")
                output_file.write(f"g: {data['g']}\n")
                output_file.write(f"rhs: {data['rhs']}\n")
                output_file.write(f"predecessors: {data['predecessors']}\n")
                output_file.write(f"successors: {data['successors']}\n")
                output_file.write("\n")
        return

    with open(output_path, "wb") as output_file:
        pickle.dump(waypoint_graph, output_file)


def main():
    argparser = argparse.ArgumentParser(description="CARLA Map Generator / Waypoint API")
    argparser.add_argument("--host", metavar="H", default="127.0.0.1", help="IP of the host server (default: 127.0.0.1)")
    argparser.add_argument("--port", metavar="P", default=2000, type=int, help="TCP port to listen to (default: 2000)")
    argparser.add_argument("--res", default=DEFAULT_RESOLUTION, type=float, help="Resolution of waypoints (fixed at 1.0 for cache consistency)")
    argparser.add_argument("--output", default="map_cache.pkl", help="Output file (.pkl or .txt)")
    args = argparser.parse_args()

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(10.0)
        world = client.get_world()
        carla_map = world.get_map()

        map_data = build_dlite_inputs(carla_map, resolution=args.res)

        print(f"Connected to CARLA: {carla_map.name}")
        print(f"Generated {len(map_data.all_waypoints)} unique waypoints at {map_data.resolution}m resolution.")

        save_waypoint_graph(map_data.waypoint_graph, args.output)
        print(f"Successfully saved {len(map_data.waypoint_graph)} waypoint graph entries to '{args.output}'.")

    except Exception as exc:
        print(f"Error during map generation: {exc}")
        sys.exit(1)


if __name__ == "__main__":
    main()
