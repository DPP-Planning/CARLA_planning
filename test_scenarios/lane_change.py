"""
Refactored version of lane_change.py demonstrating the ObstacleSpawner API.

Compare against test_scenarios/lane_change.py: the offset math and manual
actor cleanup are gone, replaced by declared obstacle types and a context
manager.
"""

import carla
from time import sleep
import sys
from pathlib import Path

sys.path.append('../')
sys.path.append(str(Path(__file__).resolve().parents[1] / "grp planning"))
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.d_agent import DPP_Controller
from obstacle_spawner import ObstacleSpawner, ObstacleType
import random

client = carla.Client("localhost", 9000)  # matches original lane_change.py's port
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
        # Same scenario as lane_change.py: a lane blocker forces a lane change.
        obstacles.spawn(ObstacleType.LANE_BLOCKER, label="lane change blocker")

        controller = DPP_Controller(vehicle, point_b, spawn_points)
        controller.start()

        while not controller.done():
            print(f"main: [UPDATE] vehicle at {vehicle.get_location()}")
            sleep(5)
    # obstacle vehicle(s) destroyed automatically here

finally:
    if vehicle is not None and vehicle.is_alive:
        vehicle.destroy()
