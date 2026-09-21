import carla
from time import sleep
import sys

#sys.path.append('../')
sys.path.insert(0, "/home/ubuntu/persistent/CARLA_LATEST/PythonAPI/carla")
import random
from agents.navigation.d_agent import DPP_Controller
from agents.navigation.basic_agent import BasicAgent

client = carla.Client("localhost", 9000)
client.set_timeout(10)
world = client.get_world()
amap = world.get_map()

blueprint_library = world.get_blueprint_library()
vehicle_bp = random.choice(blueprint_library.filter('vehicle.audi.a2')) #vehicle blueprint
vehicle_bp_2 = random.choice(blueprint_library.filter('vehicle.audi.a2')) #vehicle blueprint

sampling_resolution = 2
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

try:
    vehicle = world.spawn_actor(vehicle_bp, point_a_spawn) #spawning a random vehicle
    print ("starting vehicle spawn: ", point_a_spawn)
    vehicle_2 = world.spawn_actor(vehicle_bp_2, point_c_spawn) #spawning a random vehicle
    vehicle_3 = world.spawn_actor(vehicle_bp_2, point_e_spawn) #spawn vehicle in way of lane change
    vehicle_4 = world.spawn_actor(vehicle_bp_2, point_f_spawn)

    agent = DPP_Controller(vehicle, point_b, spawn_points)
    obs = DPP_Controller(vehicle_3, point_b, spawn_points)

    agent.start()
    agent._agent._debug = True

    flag = False
    i = 0

    while True:
        if agent.done():
            print("[MO2] The target has been reached, stopping the simulation")
            break

        if flag:
            if obs.done():
                vehicle_3.destroy()


        if vehicle_3.is_alive:
            if i > 7 and not flag:
                print("[MO2] Starting obstacle thread...")
                flag = True
                obs.start()

        i += 1
        sleep(1)

finally:
    if (vehicle.is_alive):
        destroyed_sucessfully = vehicle.destroy()

    if (vehicle_2.is_alive):
        destroyed_sucessfully = vehicle_2.destroy()

    if (vehicle_3.is_alive):
        destroyed_sucessfully = vehicle_3.destroy()

    if (vehicle_4.is_alive):
        destroyed_sucessfully = vehicle_4.destroy()
