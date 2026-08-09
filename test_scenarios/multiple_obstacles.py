import carla
from time import sleep
import sys
from pathlib import Path

sys.path.append('../')
sys.path.append(str(Path(__file__).resolve().parents[1] / "grp planning"))
from agents.navigation.global_route_planner import GlobalRoutePlanner
import random
from d_agent import DPP_Controller
# from agents.navigation.global_route_planner_og import GlobalRoutePlanner # original route planner
# from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO

client = carla.Client("localhost", 4000)
client.set_timeout(10)
world = client.get_world()
amap = world.get_map()

blueprint_library = world.get_blueprint_library()
vehicle_bp = random.choice(blueprint_library.filter('vehicle.audi.a2')) #vehicle blueprint
obstacle_bp = random.choice(blueprint_library.filter('vehicle.audi.a2')) #vehicle blueprint

sampling_resolution = 2
# dao = GlobalRoutePlannerDAO(amap, sampling_resolution)
grp = GlobalRoutePlanner(amap, sampling_resolution)
# grp.setup()
spawn_points = world.get_map().get_spawn_points()

# print(spawn_points)
point_a_spawn = spawn_points[118]
dest = spawn_points[100].location

start_waypoint = amap.get_waypoint(point_a_spawn.location, project_to_road = True, lane_type = carla.LaneType.Driving)
obstacle_1_waypoint = start_waypoint.next(20.0)[0]
obstacle_2_base = obstacle_1_waypoint.next(25.0)[0]

adjacent_lane = obstacle_2_base.get_left_lane()
if (adjacent_lane is None or adjacent_lane.lane_type != carla.LaneType.Driving):
    adjacent_lane = obstacle_2_base.get_right_lane()
if (adjacent_lane is not None and adjacent_lane.lane_type == carla.LaneType.Driving):
    obstacle_2_waypoint = adjacent_lane
else:
    obstacle_2_waypoint = obstacle_2_base

obstacle_3_waypoint = obstacle_2_base.next(25.0)[0]

obstacle_1_spawn = obstacle_1_waypoint.transform
obstacle_2_spawn = obstacle_2_waypoint.transform
obstacle_3_spawn = obstacle_3_waypoint.transform

obstacle_1_spawn.location.z += 0.5
obstacle_2_spawn.location.z += 0.5
obstacle_3_spawn.location.z += 0.5

vehicle = None
obstacle_1 = None
obstacle_2 = None
obstacle_3 = None
try:
    vehicle = world.spawn_actor(vehicle_bp, point_a_spawn) #spawning a random vehicle
    print ("starting vehicle spawn: ", point_a_spawn)

    obstacle_1 = world.try_spawn_actor(obstacle_bp, obstacle_1_spawn)
    obstacle_2 = world.try_spawn_actor(obstacle_bp, obstacle_2_spawn)
    obstacle_3 = world.try_spawn_actor(obstacle_bp, obstacle_3_spawn)

    obstacles = [obstacle_1, obstacle_2, obstacle_3]

    for i, obstacle in enumerate(obstacles):
        if obstacle is not None:
            obstacle.apply_control(carla.VehicleControl(throttle = 0.0, brake = 1.0, hand_brake = True))

    controller = DPP_Controller(vehicle, dest, spawn_points)

    controller.start()
    while vehicle.is_alive:
        distance = vehicle.get_location().distance(dest)
        if distance < 5.0:
            vehicle.apply_control(carla.VehicleControl(throttle = 0.0, brake = 1.0))
            break
        sleep(1)

finally:
    if vehicle is not None and vehicle.is_alive:
        vehicle.destroy()
    if obstacle_1 is not None and obstacle_1.is_alive:
        obstacle_1.destroy()
    if obstacle_2 is not None and obstacle_2.is_alive:
        obstacle_2.destroy()
    if obstacle_3 is not None and obstacle_3.is_alive:
        obstacle_3.destroy()
