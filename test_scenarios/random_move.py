import carla
from time import sleep
import sys

sys.path.append('../')
from agents.navigation.global_route_planner import GlobalRoutePlanner
import random
from agents.navigation.basic_agent import BasicAgent
# from agents.navigation.global_route_planner_og import GlobalRoutePlanner # original route planner
# from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO

import random

client = carla.Client("localhost", 9000)
client.set_timeout(10)
world = client.get_world()
amap = world.get_map()

blueprint_library = world.get_blueprint_library()
vehicle_bp = random.choice(blueprint_library.filter('vehicle.audi.a2')) #vehicle blueprint
vehicle_bp_2 = random.choice(blueprint_library.filter('vehicle.audi.a2')) #vehicle blueprint

sampling_resolution = 2
# dao = GlobalRoutePlannerDAO(amap, sampling_resolution)
grp = GlobalRoutePlanner(amap, sampling_resolution)
# grp.setup()
spawn_points = world.get_map().get_spawn_points()

point_a_spawn = random.choice(spawn_points)
point_b_spawn = random.choice(spawn_points)

while point_a_spawn == point_b_spawn:
    point_b_spawn = random.choice(spawn_points)

point_b = carla.Location(point_b_spawn.location)

i = 0
try:
    vehicle = world.spawn_actor(vehicle_bp, point_a_spawn) #spawning a random vehicle
    print ("starting vehicle spawn: ", point_a_spawn)
    agent = BasicAgent(vehicle) # Creating a vehicle for agent
    agent.set_destination(point_b) #Set Location Destination

    agent._debug = True

    while True:
        print(f"================== \n\n {i} \n\n==================")
        if agent.done():
            print("The target has been reached, stopping the simulation")
            break

        print("Agent:")
        vehicle.apply_control(agent.run_step())

        i += 1
        # sleep(0.5)

finally:
    if (vehicle.is_alive):
        destroyed_sucessfully = vehicle.destroy()
