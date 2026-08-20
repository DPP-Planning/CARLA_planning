"""
Quick test to make sure lanes/obstacles on the other side of the yellow
line aren't considered for rerouting. Refactored to use ObstacleSpawner.
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

agent_spawn = spawn_points[13]
obstacle_spawn = spawn_points[11]
yellow_line_spawn = spawn_points[5]
destination = spawn_points[75]

agent_vehicle = None
try:
    agent_vehicle = world.spawn_actor(vbp, agent_spawn)

    with ObstacleSpawner(world, agent_spawn) as obstacles:
        # obstacle_spawn and yellow_line_spawn are independent spawn points,
        # not offsets from the agent, so use spawn_at() with an absolute
        # transform / a different reference rather than the agent-relative
        # offset math used for the other obstacle types.
        obstacles.spawn_at(obstacle_spawn, obstacle_type=ObstacleType.OPPOSING_LANE,
                            label="opposing lane obstacle")
        obstacles.spawn(ObstacleType.OPPOSING_LANE, offset=carla.Location(-3.5, 0.0, 0.0),
                         reference=obstacle_spawn, label="opposing lane obstacle 2")
        obstacles.spawn(ObstacleType.OPPOSING_LANE, offset=carla.Location(0.0, 7.0, 0.0),
                         reference=yellow_line_spawn, label="yellow line obstacle")

        controller = DPP_Controller(agent_vehicle, destination.location, spawn_points)
        controller.start()

        while not controller.done():
            print(f"yellow_line: [UPDATE] vehicle at {agent_vehicle.get_location()}")
            sleep(5)

finally:
    if agent_vehicle is not None and agent_vehicle.is_alive:
        agent_vehicle.destroy()
