import carla
from time import sleep
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
        # Same scenario as parallel_obstacles.py: a lane blocker plus a
        # second vehicle blocking the lane-change target lane.
        obstacles.spawn(ObstacleType.LANE_BLOCKER, label="lane blocker")
        obstacles.spawn(ObstacleType.ADJACENT_LANE_BLOCKER, label="lane change target blocker")

        controller = DPP_Controller(vehicle, point_b, spawn_points)
        controller.start()

        while not controller.done():
            print(f"parallel_obstacles: [UPDATE] vehicle at {vehicle.get_location()}")
            sleep(5)

finally:
    if vehicle is not None and vehicle.is_alive:
        vehicle.destroy()
