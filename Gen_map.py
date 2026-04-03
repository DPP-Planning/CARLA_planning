import argparse
import pickle
import sys

import carla


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


def collect_neighbor_ids(neighbors, waypoint_lookup):
    neighbor_ids = []
    seen_ids = set()

    for neighbor in neighbors:
        if neighbor is None:
            continue

        neighbor_id = neighbor.id
        if neighbor_id not in waypoint_lookup or neighbor_id in seen_ids:
            continue

        seen_ids.add(neighbor_id)
        neighbor_ids.append(neighbor_id)

    return neighbor_ids


def build_predecessor_ids(waypoint, resolution, waypoint_lookup):
    predecessors = list(waypoint.previous(resolution))
    adjacent_lanes = get_driving_adjacent_lanes(waypoint)

    predecessors.extend(adjacent_lanes)

    for adjacent_lane in adjacent_lanes:
        predecessors.extend(adjacent_lane.previous(resolution))

    return collect_neighbor_ids(predecessors, waypoint_lookup)


def build_successor_ids(waypoint, resolution, waypoint_lookup):
    successors = list(waypoint.next(resolution))
    adjacent_lanes = get_driving_adjacent_lanes(waypoint)

    successors.extend(adjacent_lanes)

    for adjacent_lane in adjacent_lanes:
        successors.extend(adjacent_lane.next(resolution))

    return collect_neighbor_ids(successors, waypoint_lookup)


def main():
    argparser = argparse.ArgumentParser(description='CARLA Map Generator / Waypoint Cacher')
    argparser.add_argument('--host', metavar='H', default='127.0.0.1', help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument('--port', metavar='P', default=2000, type=int, help='TCP port to listen to (default: 2000)')
    argparser.add_argument('--res', default=1.0, type=float, help='Resolution of waypoints (default: 1.0)')
    argparser.add_argument('--output', default='map_cache.pkl', help='Output pickle file (default: map_cache.pkl)')
    args = argparser.parse_args()

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(10.0)
        world = client.get_world()
        carla_map = world.get_map()

        print(f"Connected to CARLA: {carla_map.name}")
        print(f"Generating waypoints with resolution {args.res}m...")

        waypoints = carla_map.generate_waypoints(args.res)
        waypoint_lookup = {waypoint.id: waypoint for waypoint in waypoints}
        print(f"Generated {len(waypoint_lookup)} unique waypoints.")

        waypoint_graph = {}
        for waypoint_id, waypoint in waypoint_lookup.items():
            waypoint_graph[waypoint_id] = {
                'g': float('inf'),
                'rhs': float('inf'),
                'predecessors': build_predecessor_ids(waypoint, args.res, waypoint_lookup),
                'successors': build_successor_ids(waypoint, args.res, waypoint_lookup),
            }

        with open(args.output, 'wb') as output_file:
            pickle.dump(waypoint_graph, output_file)

        print(f"Successfully saved {len(waypoint_graph)} waypoint graph entries to '{args.output}'.")

    except Exception as exc:
        print(f"Error during map generation: {exc}")
        sys.exit(1)


if __name__ == '__main__':
    main()
