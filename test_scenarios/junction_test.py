"""
Test if we have proper behavior in junctions
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

agent_spawn = spawn_points[11]
point = spawn_points[100].location
junction_spawn = carla.Transform(carla.Location(point.x, point.y + 20, point.z))

destination = spawn_points[80]

agent_vehicle = world.spawn_actor(vbp, agent_spawn)
agent = BasicAgent(agent_vehicle)
agent.set_destination(destination.location)

junction_vehicle = world.spawn_actor(vbp, junction_spawn)

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

    if (junction_vehicle.is_alive):
        destroyed_sucessfully = junction_vehicle.destroy()