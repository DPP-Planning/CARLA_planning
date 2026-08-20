"""
Refactored version of moving_obs.py.

The original spawned two obstacle vehicles but never called
set_autopilot() on either one - despite the filename, nothing in the
scenario actually moved. This version keeps the first obstacle as a
genuine static LANE_BLOCKER (matching the original's actual behavior)
and makes the second obstacle a real MOVING_OBSTACLE with autopilot
enabled, so the scenario tests what its name says it tests: the agent
reacting to a moving hazard, not a second stationary one.

If a silently-static "moving" obstacle was actually intentional (e.g. to
isolate a specific static-obstacle bug), keep using the old
moving_obs.py instead of this file.
"""

import carla
from time import sleep
import os
import sys
from pathlib import Path

sys.path.append('../')
sys.path.append(str(Path(__file__).resolve().parents[1] / "grp planning"))
from agents.navigation.global_route_planner import GlobalRoutePlanner
from d_agent import DPP_Controller
from obstacle_spawner import ObstacleSpawner, ObstacleType
import random

client = carla.Client("localhost", 9000)
client.set_timeout(10)
world = client.get_world()
amap = world.get_map()

blueprint_library = world.get_blueprint_library()
vehicle_bp = random.choice(blueprint_library.filter('vehicle.audi.a2'))

sampling_resolution = 2
grp = GlobalRoutePlanner(amap, sampling_resolution)
spawn_points = world.get_map().get_spawn_points()

point_a_spawn = spawn_points[50]
point_b_spawn = spawn_points[100]
point_b = carla.Location(point_b_spawn.location)

vehicle = None
try:
    vehicle = world.spawn_actor(vehicle_bp, point_a_spawn)
    print("starting vehicle spawn:", point_a_spawn)

    with ObstacleSpawner(world, point_a_spawn) as obstacles:
        # Static lane blocker - same offset/behavior as vehicle_2 in the
        # original moving_obs.py.
        lane_blocker = obstacles.spawn(ObstacleType.LANE_BLOCKER, label="static lane blocker")

        # Genuinely moving obstacle - same starting offset as the original
        # vehicle_4 (-30, +4), but now actually drives via autopilot
        # instead of sitting still.
        moving_obstacle = obstacles.spawn(
            ObstacleType.MOVING_OBSTACLE,
            offset=carla.Location(-30.0, 4.0, 0.0),
            autopilot=True,
            label="moving obstacle",
        )

        controller = DPP_Controller(vehicle, point_b, spawn_points)

        os.makedirs("out", exist_ok=True)
        with open("out/basic_agent.log", "w") as f:
            f.write(f"Primary Agent ID: {vehicle.id}\n")
            f.write(f"Primary Agent Starting Location: {point_a_spawn}\n")
            f.write(f"Static Lane Blocker ID: {lane_blocker.id}\n")
            f.write(f"Moving Obstacle ID: {moving_obstacle.id}\n")
            f.write("\n")

        controller.start()

        while not controller.done():
            print(f"moving_obs: [UPDATE] vehicle at {vehicle.get_location()}")
            sleep(5)

finally:
    if vehicle is not None and vehicle.is_alive:
        vehicle.destroy()
