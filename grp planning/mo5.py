import carla
from time import sleep
import sys

sys.path.append('../')
from agents.navigation.global_route_planner import GlobalRoutePlanner
import random
from agents.navigation.basic_agent import BasicAgent
# from agents.navigation.global_route_planner_og import GlobalRoutePlanner # original route planner
# from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO

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


# for i in range(len(spawn_points)):
#     world.debug.draw_string(spawn_points[i].location, f'{i}', draw_shadow=False,
#     color=carla.Color(r=255, g=255, b=0), life_time=20.0,
#     persistent_lines=True)

# Obstacle Index: 34
# Start Index: 118

# assert(False)

# print(spawn_points)
point_a_spawn = spawn_points[118]
point_b_spawn = spawn_points[34]
point_a = carla.Location(point_a_spawn.location)
point_b = carla.Location(point_b_spawn.location)

dest = spawn_points[100].location

i = 0
try:
    vehicle = world.spawn_actor(vehicle_bp, point_a_spawn) #spawning a random vehicle
    print ("starting vehicle spawn: ", point_a_spawn)
    vehicle_2 = world.spawn_actor(vehicle_bp_2, point_b_spawn) #spawning a random vehicle
    agent = BasicAgent(vehicle) # Creating a vehicle for agent
    agent.set_destination(dest) #Set Location Destination
    agent._debug = True

    while True:
        # if i % 10 == 0: print(f"====================== \n\n {i} \n\n==================")
        print(f"================== \n\n {i} \n\n==================")

        if agent.done():
            print("The target has been reached, stopping the simulation")
            break

        print("Agent:")
        vehicle.apply_control(agent.run_step())

        i += 1

finally:
    destroyed_sucessfully = vehicle.destroy()
    destroyed_sucessfully = vehicle_2.destroy()
