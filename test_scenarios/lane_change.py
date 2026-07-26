import carla
from time import sleep
import sys
from pathlib import Path

# insert path to carla folder
sys.path.insert(0, "/home/ubuntu/persistent/CARLA_LATEST/PythonAPI/carla")

from agents.navigation.global_route_planner import GlobalRoutePlanner
import random
from agents.navigation.d_agent import DPP_Controller
from agents.navigation.Generate_map import gen_map_initial, save_waypoint_graph

client = carla.Client("localhost", 9000)
client.set_timeout(10)
world = client.get_world()
amap = world.get_map()

blueprint_library = world.get_blueprint_library()
vehicle_bp = random.choice(blueprint_library.filter('vehicle.audi.a2'))
vehicle_bp_2 = random.choice(blueprint_library.filter('vehicle.audi.a2'))

spawn_points = world.get_map().get_spawn_points()
point_a_spawn = spawn_points[50]
point_b_spawn = spawn_points[100]
point_a = carla.Location(point_a_spawn.location)
point_b = carla.Location(point_b_spawn.location)
point_c_spawn = carla.Transform(carla.Location(point_a.x - 20, point_a.y, point_a.z), point_a_spawn.rotation) # Lane Blocker
point_d_spawn = carla.Transform(carla.Location(point_a.x - 65, point_a.y + 1.25, point_a.z), point_b_spawn.rotation) # Mid-Intersection block
point_e_spawn = carla.Transform(carla.Location(point_a.x - 20, point_a.y + 4, point_a.z), point_a_spawn.rotation)
point_f_spawn = carla.Transform(carla.Location(point_a.x - 30, point_a.y + 4, point_a.z), point_a_spawn.rotation)

i = 0

try:
    vehicle = world.spawn_actor(vehicle_bp, point_a_spawn) #spawn ego
    vehicle_2 = world.spawn_actor(vehicle_bp_2, point_c_spawn) #spawn blocker

    controller = DPP_Controller(vehicle, point_b, spawn_points) # initalize agent
    controller.start() # start threads

    while vehicle.is_alive and not controller.done():
        current_loc = vehicle.get_location()
        print(f"UPDATE: current location: {current_loc}, distance from goal: {current_loc.distance(point_b)}")
        sleep(1)

finally:
    if (vehicle.is_alive):
        destroyed_sucessfully = vehicle.destroy()

    if (vehicle_2.is_alive):
        destroyed_sucessfully = vehicle_2.destroy()
