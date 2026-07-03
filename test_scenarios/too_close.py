"""
Quick test to make sure obstacles on the other side of the yellow line
aren't being marked as vehicles.
"""

import carla
from time import sleep
import sys

sys.path.append('../')
from agents.navigation.global_route_planner import GlobalRoutePlanner
import random
from agents.navigation.basic_agent import BasicAgent

client = carla.Client("localhost", 9000)
client.set_timeout(10)
world = client.get_world()
amap = world.get_map()

blueprint_library = world.get_blueprint_library()
vbp = random.choice(blueprint_library.filter('vehicle.audi.a2')) #vehicle blueprint

sampling_resolution = 2
grp = GlobalRoutePlanner(amap, sampling_resolution)
spawn_points = world.get_map().get_spawn_points()

agent_spawn = spawn_points[13]
destination = spawn_points[75]

agent_vehicle = world.spawn_actor(vbp, agent_spawn)
agent = BasicAgent(agent_vehicle)
agent.set_destination(destination.location)

obstacle_vehicle = world.spawn_actor(vbp, carla.Transform(carla.Location(
    agent_spawn.location.x, 
    agent_spawn.location.y + 5, 
    agent_spawn.location.z
), agent_spawn.rotation))

i = 0

try:
    while True:
        print(f"================== \n\n {i} \n\n==================")

        if agent.done():
            print("The target has been reached, stopping the simulation")
            break

        print("Agent:")
        agent_vehicle.apply_control(agent.run_step())

        i += 1

finally:
    if (agent_vehicle.is_alive):
        destroyed_sucessfully = agent_vehicle.destroy()

    if (obstacle_vehicle.is_alive):
        destroyed_sucessfully = obstacle_vehicle.destroy()