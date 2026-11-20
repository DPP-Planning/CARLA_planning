from scenario_config import create_carla_client
import carla
from time import sleep
from agents.navigation.global_route_planner import GlobalRoutePlanner
import random
from d_agent import DPP_Controller
# from agents.navigation.global_route_planner_og import GlobalRoutePlanner # original route planner
# from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO

client = create_carla_client(carla, 9000)
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
    controller = DPP_Controller(vehicle, amap.get_waypoint(dest), spawn_points)

    controller.run()

finally:
    destroyed_sucessfully = vehicle.destroy()
    destroyed_sucessfully = vehicle_2.destroy()
