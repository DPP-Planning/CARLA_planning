# import carla
# import sys

# sys.path.append('../')
# from agents.navigation.global_route_planner import GlobalRoutePlanner
# import random
# from agents.navigation.basic_agent import BasicAgent
# import threading, time
# # from agents.navigation.global_route_planner_og import GlobalRoutePlanner # original route planner
# # from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO

# client = carla.Client("localhost", 4000)
# client.set_timeout(10)
# world = client.get_world()
# amap = world.get_map()

# blueprint_library = world.get_blueprint_library()
# vehicle_bp = random.choice(blueprint_library.filter('vehicle.audi.a2')) #vehicle blueprint
# vehicle_bp_2 = random.choice(blueprint_library.filter('vehicle.audi.a2')) #vehicle blueprint

# sampling_resolution = 2
# # dao = GlobalRoutePlannerDAO(amap, sampling_resolution)
# grp = GlobalRoutePlanner(amap, sampling_resolution)
# # grp.setup()
# spawn_points = world.get_map().get_spawn_points()
# # print(spawn_points)
# point_a_spawn = spawn_points[50]
# point_b_spawn = spawn_points[100]
# point_a = carla.Location(point_a_spawn.location)
# point_b = carla.Location(point_b_spawn.location)
# # point_b = carla.Location(point_b.x - 10, point_b.y, point_b.z) # Overlay for purposful remapping
# point_c_spawn = carla.Transform(carla.Location(point_a.x - 20, point_a.y, point_a.z), point_a_spawn.rotation) # Lane Blocker
# point_d_spawn = carla.Transform(carla.Location(point_a.x - 65, point_a.y + 1.25, point_a.z), point_b_spawn.rotation) # Mid-Intersection block
# # point_a_spawn = carla.Transform(carla.Location(point_a.x - 100, point_a.y, point_a.z), point_a_spawn.rotation) # forward spawn
# # point_d_spawn = carla.Transform(carla.Location(point_a.x - 85, point_a.y + 1.25, point_a.z), point_b_spawn.rotation) # Full ahead road block
# # point_d_spawn = carla.Transform(carla.Location(point_b.x + 1, point_b.y + 10, point_a.z), point_a_spawn.rotation) # Right Turn Partially blocked
# # point_c_waypoint = amap.get_waypoint(carla.Location(point_a.x - 20, point_a.y, point_a.z))

# print("Point c:", point_c_spawn)

# wp_start = amap.get_waypoint(point_a)
# wp_end   = amap.get_waypoint(point_b)

# def background_planner():
#     while True:
#         try:
#             grp.trace_route(wp_start.transform.location,
#                             wp_end.transform.location, world)
#         except Exception as e:
#             print("Planner error:", e)
#         time.sleep(0.5)

# planner_thread = threading.Thread(target=background_planner, daemon=True)
# planner_thread.start()

# # w1 = grp.trace_route(point_a, point_b, world) # there are other funcations can be used to generate a route in GlobalRoutePlanner.
#     # print (spawn_points[50].location)
#     # print (spawn_points[100].location)
#     # print(spawn_points[100].location.x - spawn_points[50].location.x)

# # i = 0
# # for w in w1:
# #     print(w[0].transform.location.x, ",",w[0].transform.location.y, w[1])
#     # if i % 10 == 0:
#     #     world.debug.draw_string(w[0].transform.location, 'O', draw_shadow=False,
#     #     color=carla.Color(r=255, g=0, b=0), life_time=120.0,
#     #     persistent_lines=True)
#     # else:
#     #     world.debug.draw_string(w[0].transform.location, 'O', draw_shadow=False,
#     #     color = carla.Color(r=0, g=0, b=255), life_time=1000.0,
#     #     persistent_lines=True)
#     # i += 1

# i = 0
# try:
#     vehicle = world.spawn_actor(vehicle_bp, point_a_spawn) #spawning a random vehicle
#     print ("starting vehicle spawn: ", point_a_spawn)
#     # vehicle_2 = world.spawn_actor(vehicle_bp_2, point_d_spawn) #spawning a random vehicle
#     vehicle_2 = world.spawn_actor(vehicle_bp_2, point_c_spawn) #spawning a random vehicle
#     agent = BasicAgent(vehicle) # Creating a vehicle for agent
#     agent.set_destination(point_b) #Set Location Destination

#     # print(vehicle.get_location().x, ",", vehicle.get_location().y, point_a_spawn)
#     # print(vehicle_2.get_location().x, ",", vehicle.get_location().y, point_c_spawn)

#     i = 0
#     while True:
#         # if (i % 1000 == 0):
#         #     print(vehicle.get_location().x, ",", vehicle.get_location().y)
#         if agent.done():
#             print("The target has been reached, stopping the simulation")
#             break
#         vehicle.apply_control(agent.run_step())
#         i += 1

# finally:
#     destroyed_sucessfully = vehicle.destroy()
#     destroyed_sucessfully = vehicle_2.destroy()
#=================================================================================

import carla
import sys
import threading
import time
import random
import os
from collections import defaultdict

sys.path.append('../')
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.basic_agent import BasicAgent

# Get CARLA connection details from environment variables (useful for Docker)
CARLA_HOST = os.getenv('CARLA_HOST', 'localhost')
CARLA_PORT = int(os.getenv('CARLA_PORT', '4000'))

print(f"Connecting to CARLA server at {CARLA_HOST}:{CARLA_PORT}")

client = carla.Client(CARLA_HOST, CARLA_PORT)
client.set_timeout(10)
world = client.get_world()
amap = world.get_map()

blueprint_library = world.get_blueprint_library()
vehicle_bp = random.choice(blueprint_library.filter('vehicle.audi.a2'))
vehicle_bp_2 = random.choice(blueprint_library.filter('vehicle.audi.a2'))

sampling_resolution = 2
grp = GlobalRoutePlanner(amap, sampling_resolution)

spawn_points = world.get_map().get_spawn_points()
point_a_spawn = spawn_points[50]
point_b_spawn = spawn_points[100]

point_a = carla.Location(point_a_spawn.location)
point_b = carla.Location(point_b_spawn.location)

point_c_spawn = carla.Transform(
    carla.Location(point_a.x - 20, point_a.y, point_a.z),
    point_a_spawn.rotation
)

print("Point c:", point_c_spawn)

planner_lock = threading.Lock()
new_path_event = threading.Event()

g   = defaultdict(lambda: float('inf'))
rhs = defaultdict(lambda: float('inf'))
current_path = []

# Initialize D* Lite state
all_wps = amap.generate_waypoints(2.0)
for wp in all_wps:
    g[wp.id] = float('inf')
    rhs[wp.id] = float('inf')

try:
    vehicle = world.spawn_actor(vehicle_bp, point_a_spawn)
    vehicle_2 = world.spawn_actor(vehicle_bp_2, point_c_spawn)

    agent = BasicAgent(vehicle)

    def agent_thread_fn():
        agent.set_destination(point_b)

    agent_thread = threading.Thread(target=agent_thread_fn)
    agent_thread.start()

    def planner_thread_fn():
        global current_path
        while not agent.done():
            with planner_lock:
                route = grp.trace_route(vehicle.get_location(), point_b, world)
                print ('route waypoints:', route)

                for wp_list in route:
                    for wp, _ in wp_list:
                        rhs[wp.id] = 0.0
                        g[wp.id] = min(g[wp.id], rhs[wp.id] + 1.0)

                current_path = route
                new_path_event.set()

            time.sleep(0.5)

    planner_thread = threading.Thread(
        target=planner_thread_fn, daemon=True
    )
    planner_thread.start()

    while True:
        if agent.done():
            print("The target has been reached, stopping the simulation")
            break
        
        if new_path_event.is_set():
            with planner_lock:
                agent.set_destination(current_path)
                new_path_event.clear()

    # def planner_thread_fn():
    #     global current_path
    #     while not agent.done():
    #         with planner_lock:
    #             route = grp.trace_route(amap.get_waypoint(vehicle.get_location()), amap.get_waypoint(point_b), world)
    #             for 
    #             for wp, _ in route:
    #                 rhs[wp.id] = 0.0
    #                 g[wp.id] = min(g[wp.id], rhs[wp.id] + 1.0)

    #             current_path = route
    #             new_path_event.set()

    #         time.sleep(0.5)

    # planner_thread = threading.Thread(
    #     target=planner_thread_fn, daemon=True
    # )
    # planner_thread.start()

    # while True:
    #     if agent.done():
    #         print("The target has been reached, stopping the simulation")
    #         break
        
    #     if new_path_event.is_set():
    #         with planner_lock:
    #             agent.set_global_plan(current_path)
    #             new_path_event.clear()

        control = agent.run_step()
        vehicle.apply_control(control)

finally:
    agent_thread.join()
    vehicle.destroy()
    vehicle_2.destroy()

# import carla
# import sys
# import threading
# import time
# import random

# sys.path.append('../')
# from agents.navigation.global_route_planner import GlobalRoutePlanner
# from agents.navigation.basic_agent import BasicAgent

# client = carla.Client("localhost", 4000)
# client.set_timeout(10)
# world = client.get_world()
# amap = world.get_map()

# blueprint_library = world.get_blueprint_library()
# vehicle_bp = random.choice(blueprint_library.filter('vehicle.audi.a2'))
# vehicle_bp_2 = random.choice(blueprint_library.filter('vehicle.audi.a2'))

# sampling_resolution = 2
# grp = GlobalRoutePlanner(amap, sampling_resolution)

# spawn_points = world.get_map().get_spawn_points()
# point_a_spawn = spawn_points[50]
# point_b_spawn = spawn_points[100]

# point_a = carla.Location(point_a_spawn.location)
# point_b = carla.Location(point_b_spawn.location)

# point_c_spawn = carla.Transform(
#     carla.Location(point_a.x - 20, point_a.y, point_a.z),
#     point_a_spawn.rotation
# )

# print("Point c:", point_c_spawn)

# try:
#     vehicle = world.spawn_actor(vehicle_bp, point_a_spawn)
#     vehicle_2 = world.spawn_actor(vehicle_bp_2, point_c_spawn)

#     agent = BasicAgent(vehicle)

#     def planner_thread_fn():
#         while not agent.done():
#             grp.trace_route(point_a, point_b, world)
#             time.sleep(0.5)

#     def agent_thread_fn():
#         agent.set_destination(point_b)

#     planner_thread = threading.Thread(
#         target=planner_thread_fn, daemon=True
#     )
#     agent_thread = threading.Thread(
#         target=agent_thread_fn
#     )

#     planner_thread.start()
#     agent_thread.start()

#     while True:
#         if agent.done():
#             print("The target has been reached, stopping the simulation")
#             break

#         control = agent.run_step()
#         vehicle.apply_control(control)

# finally:
#     agent_thread.join()
#     vehicle.destroy()
#     vehicle_2.destroy()
