"""
Test proper behavior in junctions. Refactored to use ObstacleSpawner.
"""

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
vbp = random.choice(blueprint_library.filter('vehicle.audi.a2'))

sampling_resolution = 2
grp = GlobalRoutePlanner(amap, sampling_resolution)
spawn_points = world.get_map().get_spawn_points()

agent_spawn = spawn_points[11]
junction_reference = spawn_points[100]
destination = spawn_points[80]

agent_vehicle = None
try:
    agent_vehicle = world.spawn_actor(vbp, agent_spawn)
    controller = DPP_Controller(agent_vehicle, destination.location, spawn_points)

    with ObstacleSpawner(world, agent_spawn) as obstacles:
        # JUNCTION_CROSS_TRAFFIC's default offset (0, +20, 0) is applied
        # relative to spawn_points[100], not the agent, so pass it as the
        # per-call reference.
        obstacles.spawn(ObstacleType.JUNCTION_CROSS_TRAFFIC, reference=junction_reference,
                         label="junction cross traffic")

        controller.start()

        while not controller.done():
            print(f"junction_test: [UPDATE] vehicle at {agent_vehicle.get_location()}")
            sleep(5)

finally:
    if agent_vehicle is not None and agent_vehicle.is_alive:
        agent_vehicle.destroy()
