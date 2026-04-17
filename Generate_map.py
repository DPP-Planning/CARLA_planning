import argparse
import math
import pickle
import sys
from collections import defaultdict

import carla


def build_generated_waypoint_lookup(carla_map, resolution):
    waypoint_lookup = {}

    for waypoint in carla_map.generate_waypoints(resolution):
        waypoint_lookup[waypoint.id] = waypoint

    return waypoint_lookup


def get_location_key(location, bucket_size):
    return (
        math.floor(location.x / bucket_size),
        math.floor(location.y / bucket_size),
        math.floor(location.z / bucket_size),
    )


def build_spatial_index(waypoint_lookup, bucket_size):
    spatial_index = defaultdict(list)

    for waypoint in waypoint_lookup.values():
        location_key = get_location_key(waypoint.transform.location, bucket_size)
        spatial_index[location_key].append(waypoint)

    return spatial_index


def get_closest_generated_waypoint(target_waypoint, waypoint_lookup, spatial_index, resolution):
    if target_waypoint is None:
        return None

    if target_waypoint.id in waypoint_lookup:
        return waypoint_lookup[target_waypoint.id]

    target_location = target_waypoint.transform.location
    base_key = get_location_key(target_location, resolution)
    closest_waypoint = None
    closest_distance = float('inf')

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


def get_driving_adjacent_lanes(waypoint):
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


def collect_neighbor_ids(neighbors, waypoint_lookup, spatial_index, resolution):
    neighbor_ids = []
    seen_ids = set()

    for neighbor in neighbors:
        resolved_neighbor = get_closest_generated_waypoint(neighbor, waypoint_lookup, spatial_index, resolution)
        if resolved_neighbor is None:
            continue

        neighbor_id = resolved_neighbor.id
        if neighbor_id in seen_ids:
            continue

        seen_ids.add(neighbor_id)
        neighbor_ids.append(neighbor_id)

    return neighbor_ids


def build_predecessor_ids(waypoint, resolution, waypoint_lookup, spatial_index):
    predecessors = list(waypoint.previous(resolution))
    adjacent_lanes = get_driving_adjacent_lanes(waypoint)

    predecessors.extend(adjacent_lanes)

    for adjacent_lane in adjacent_lanes:
        predecessors.extend(adjacent_lane.previous(resolution))

    return collect_neighbor_ids(predecessors, waypoint_lookup, spatial_index, resolution)


def build_successor_ids(waypoint, resolution, waypoint_lookup, spatial_index):
    successors = list(waypoint.next(resolution))
    adjacent_lanes = get_driving_adjacent_lanes(waypoint)

    successors.extend(adjacent_lanes)

    for adjacent_lane in adjacent_lanes:
        successors.extend(adjacent_lane.next(resolution))

    return collect_neighbor_ids(successors, waypoint_lookup, spatial_index, resolution)


def main():
    argparser = argparse.ArgumentParser(description='CARLA Map Generator / Waypoint Cacher')
    argparser.add_argument('--host', metavar='H', default='127.0.0.1', help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument('--port', metavar='P', default=2000, type=int, help='TCP port to listen to (default: 2000)')
    argparser.add_argument('--res', default=1.0, type=float, help='Resolution of waypoints (fixed at 1.0 for cache consistency)')
    argparser.add_argument('--output', default='map_cache.pkl', help='Output pickle file (default: map_cache.pkl)')
    args = argparser.parse_args()

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(10.0)
        world = client.get_world()
        carla_map = world.get_map()

        resolution = 1.0
        if args.res != resolution:
            print(f"Requested resolution {args.res}m overridden to {resolution}m for waypoint-cache consistency.")

        print(f"Connected to CARLA: {carla_map.name}")
        print(f"Generating waypoints with resolution {resolution}m...")

        waypoint_lookup = build_generated_waypoint_lookup(carla_map, resolution)
        spatial_index = build_spatial_index(waypoint_lookup, resolution)
        print(f"Generated {len(waypoint_lookup)} unique waypoints.")

        waypoint_graph = {}
        for waypoint_id, waypoint in waypoint_lookup.items():
            location = waypoint.transform.location
            waypoint_graph[waypoint_id] = {
                'location': (location.x, location.y, location.z),
                'g': float('inf'),
                'rhs': float('inf'),
                'predecessors': build_predecessor_ids(waypoint, resolution, waypoint_lookup, spatial_index),
                'successors': build_successor_ids(waypoint, resolution, waypoint_lookup, spatial_index),
            }

        with open(args.output, 'wb') as output_file:
            pickle.dump(waypoint_graph, output_file)

        print(f"Successfully saved {len(waypoint_graph)} waypoint graph entries to '{args.output}'.")

    except Exception as exc:
        print(f"Error during map generation: {exc}")
        sys.exit(1)


if __name__ == '__main__':
    main()
    