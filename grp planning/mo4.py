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
# print(spawn_points)
point_a_spawn = spawn_points[50]
point_b_spawn = spawn_points[100]
point_a = carla.Location(point_a_spawn.location)
point_b = carla.Location(point_b_spawn.location)
# point_b = carla.Location(point_b.x - 10, point_b.y, point_b.z) # Overlay for purposful remapping
point_c_spawn = carla.Transform(carla.Location(point_a.x - 20, point_a.y, point_a.z), point_a_spawn.rotation) # Lane Blocker
point_d_spawn = carla.Transform(carla.Location(point_a.x - 65, point_a.y + 1.25, point_a.z), point_b_spawn.rotation) # Mid-Intersection block
point_e_spawn = carla.Transform(carla.Location(point_a.x - 20, point_a.y + 4, point_a.z), point_a_spawn.rotation)
point_f_spawn = carla.Transform(carla.Location(point_a.x - 30, point_a.y + 4, point_a.z), point_a_spawn.rotation)
# point_a_spawn = carla.Transform(carla.Location(point_a.x - 100, point_a.y, point_a.z), point_a_spawn.rotation) # forward spawn
# point_d_spawn = carla.Transform(carla.Location(point_a.x - 85, point_a.y + 1.25, point_a.z), point_b_spawn.rotation) # Full ahead road block
# point_d_spawn = carla.Transform(carla.Location(point_b.x + 1, point_b.y + 10, point_a.z), point_a_spawn.rotation) # Right Turn Partially blocked
# point_c_waypoint = amap.get_waypoint(carla.Location(point_a.x - 20, point_a.y, point_a.z))

print("Point c:", point_c_spawn)

# w1 = grp.trace_route(point_a, point_b, world) # there are other funcations can be used to generate a route in GlobalRoutePlanner.
    # print (spawn_points[50].location)
    # print (spawn_points[100].location)
    # print(spawn_points[100].location.x - spawn_points[50].location.x)

# i = 0
# for w in w1:
#     print(w[0].transform.location.x, ",",w[0].transform.location.y, w[1])
    # if i % 10 == 0:
    #     world.debug.draw_string(w[0].transform.location, 'O', draw_shadow=False,
    #     color=carla.Color(r=255, g=0, b=0), life_time=120.0,
    #     persistent_lines=True)
    # else:
    #     world.debug.draw_string(w[0].transform.location, 'O', draw_shadow=False,
    #     color = carla.Color(r=0, g=0, b=255), life_time=1000.0,
    #     persistent_lines=True)
    # i += 1

i = 0
try:
    vehicle = world.spawn_actor(vehicle_bp, point_a_spawn) #spawning a random vehicle
    print ("starting vehicle spawn: ", point_a_spawn)
    # vehicle_2 = world.spawn_actor(vehicle_bp_2, point_d_spawn) #spawning a random vehicle
    vehicle_2 = world.spawn_actor(vehicle_bp_2, point_c_spawn) #spawning a random vehicle
    # vehicle_3 = world.spawn_actor(vehicle_bp_2, point_e_spawn) #spawn vehicle in way of lane change
    vehicle_4 = world.spawn_actor(vehicle_bp_2, point_f_spawn)
    agent = BasicAgent(vehicle) # Creating a vehicle for agent
    agent.set_destination(point_b) #Set Location Destination

    # obs = BasicAgent(vehicle_3)
    # obs.set_destination(point_b)

    obs = BasicAgent(vehicle_4)
    obs.set_destination(point_b)

    agent._debug = True
    obs._debug =  True

    while True:
        # if i % 10 == 0: print(f"====================== \n\n {i} \n\n==================")
        print(f"================== \n\n {i} \n\n==================")
        # if (i % 1000 == 0):
        #     print(vehicle.get_location().x, ",", vehicle.get_location().y)
        if agent.done():
            print("The target has been reached, stopping the simulation")
            break

        if obs.done() and vehicle_4.is_alive:
            vehicle_4.destroy()

        print("Agent:")
        vehicle.apply_control(agent.run_step())

        print("Obs:")
        if vehicle_4.is_alive:
            if i > 6000:
                vehicle_4.apply_control(obs.run_step())

        i += 1

finally:
    if (vehicle.is_alive):
        destroyed_sucessfully = vehicle.destroy()

    if (vehicle_2.is_alive):
        destroyed_sucessfully = vehicle_2.destroy()

    # if (vehicle_3.is_alive):
    #     destroyed_sucessfully = vehicle_3.destroy()

    if (vehicle_4.is_alive):
        destroyed_sucessfully = vehicle_4.destroy()
