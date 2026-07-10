"""
Quick test to make sure the lanes on the other side of the yellow line
aren't being considered for rerouting, and that obstacles on the other side
aren't being considered as obstacles.
"""

import carla
from time import sleep
import sys
from pathlib import Path

sys.path.append('../')
sys.path.append(str(Path(__file__).resolve().parents[1] / "grp planning"))
from agents.navigation.global_route_planner import GlobalRoutePlanner
import random
from d_agent import DPP_Controller

client = carla.Client("localhost", 9000)
client.set_timeout(10)
world = client.get_world()
amap = world.get_map()

blueprint_library = world.get_blueprint_library()
vbp = random.choice(blueprint_library.filter('vehicle.audi.a2')) #vehicle blueprint

sampling_resolution = 2
grp = GlobalRoutePlanner(amap, sampling_resolution)
spawn_points = world.get_map().get_spawn_points()

for i in range(len(spawn_points)): 
    sp = spawn_points[i]
    world.debug.draw_string(sp.location, str(i), color=carla.Color(255, 0, 0), life_time=10.0)
    print(sp)

agent_spawn = spawn_points[13]
obstacle_spawn = spawn_points[11]
yellow_line_spawn = spawn_points[5]
destination = spawn_points[75]

agent_vehicle = world.spawn_actor(vbp, agent_spawn)
controller = DPP_Controller(agent_vehicle, amap.get_waypoint(destination.location), spawn_points)

obstacle_vehicle = world.spawn_actor(vbp, obstacle_spawn)
obstacle_vehicle_2 = world.spawn_actor(vbp, carla.Transform(
    carla.Location(
        obstacle_spawn.location.x - 3.5, obstacle_spawn.location.y, obstacle_spawn.location.z 
    ),
    obstacle_spawn.rotation
))

line_vehicle = world.spawn_actor(vbp, carla.Transform(
    carla.Location(
        yellow_line_spawn.location.x, yellow_line_spawn.location.y + 7, yellow_line_spawn.location.z 
    ),
    yellow_line_spawn.rotation
))

try:
    controller.run()

finally:
    if (agent_vehicle.is_alive):
        destroyed_sucessfully = agent_vehicle.destroy()

    if (line_vehicle.is_alive):
        destroyed_sucessfully = line_vehicle.destroy()

    if (obstacle_vehicle.is_alive):
        destroyed_sucessfully = obstacle_vehicle.destroy()

    if (obstacle_vehicle_2.is_alive):
        destroyed_sucessfully = obstacle_vehicle_2.destroy()
