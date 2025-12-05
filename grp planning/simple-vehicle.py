# import carla
# import sys, os, time, math, threading, heapq, random

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
# point_c_spawn = carla.Transform(carla.Location(point_a.x - 20, point_a.y, point_a.z), point_a_spawn.rotation)

# print("Point c:", point_c_spawn)

# def euclid(a, b):
#     return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)

# gen_points = amap.generate_waypoints(2.0)

# all_waypoints = [amap.get_waypoint(g.transform.location, project_to_road=True) for g in gen_points]
# wp_index = {wp.id: i for i, wp in enumerate(all_waypoints)}

# def nearest_wp_to_location(loc):
#     best = None
#     bd = float('inf')
#     for wp in all_waypoints:
#         d = euclid(wp.transform.location, loc)
#         if d < bd:
#             bd = d; best = wp
#     return best

# succs = {}  
# preds = {}  
# for wp in all_waypoints:
#     sids = []

#     fwd = wp.next(2.0)
#     if fwd:
#         for f in fwd:
#             sids.append(nearest_wp_to_location(f.transform.location).id)

#     if wp.lane_change & carla.LaneChange.Left:
#         left = wp.get_left_lane()
#         if left and left.lane_type == carla.LaneType.Driving:
#             sids.append(nearest_wp_to_location(left.transform.location).id)
#             nxt = left.next(2.0)
#             if nxt: sids.append(nearest_wp_to_location(nxt[0].transform.location).id)
#     if wp.lane_change & carla.LaneChange.Right:
#         right = wp.get_right_lane()
#         if right and right.lane_type == carla.LaneType.Driving:
#             sids.append(nearest_wp_to_location(right.transform.location).id)
#             nxt = right.next(2.0)
#             if nxt: sids.append(nearest_wp_to_location(nxt[0].transform.location).id)

#     succs[wp.id] = list(dict.fromkeys([sid for sid in sids if sid != wp.id]))

# for wid in succs:
#     preds[wid] = []
# for wid, slist in succs.items():
#     for s in slist:
#         preds.setdefault(s, []).append(wid)

# planner_lock = threading.Lock()
# planner_path = []      
# blocked_wp_id = None   
# stop_planner = threading.Event()

# def planner_thread_fn(start_wp, goal_wp, obstacle_actor):
#     global planner_path, blocked_wp_id

#     while not stop_planner.is_set():

#         try:
#             obs_loc = obstacle_actor.get_location()
#             obs_wp = nearest_wp_to_location(obs_loc)
#             with planner_lock:
#                 blocked_wp_id = obs_wp.id if obs_wp else None
#         except Exception:
#             with planner_lock:
#                 blocked_wp_id = None

#         g = {wp.id: float('inf') for wp in all_waypoints}
#         visited = set()

#         hq = []
#         g[goal_wp.id] = 0.0
#         heapq.heappush(hq, (0.0, goal_wp.id))

#         while hq:
#             cost_u, uid = heapq.heappop(hq)
#             if cost_u != g[uid]:
#                 continue
#             with planner_lock:
#                 if blocked_wp_id is not None and uid == blocked_wp_id:
#                     continue
#             for pred_id in preds.get(uid, []):
#                 with planner_lock:
#                     if blocked_wp_id is not None and (pred_id == blocked_wp_id or uid == blocked_wp_id):
#                         continue
#                 loc_p = next((w.transform.location for w in all_waypoints if w.id == pred_id), None)
#                 loc_u = next((w.transform.location for w in all_waypoints if w.id == uid), None)
#                 if loc_p is None or loc_u is None:
#                     continue
#                 alt = euclid(loc_p, loc_u) + g[uid]
#                 if alt < g[pred_id]:
#                     g[pred_id] = alt
#                     heapq.heappush(hq, (alt, pred_id))
#         path_ids = []
#         cur = start_wp.id
#         safety = 0
#         while cur != goal_wp.id and safety < 1000:
#             safety += 1
#             path_ids.append(cur)
#             best = None; best_val = float('inf')
#             for s in succs.get(cur, []):
#                 with planner_lock:
#                     if blocked_wp_id is not None and (s == blocked_wp_id or cur == blocked_wp_id):
#                         continue
#                 loc_cur = next((w.transform.location for w in all_waypoints if w.id == cur), None)
#                 loc_s = next((w.transform.location for w in all_waypoints if w.id == s), None)
#                 if loc_cur is None or loc_s is None: continue
#                 val = euclid(loc_cur, loc_s) + g.get(s, float('inf'))
#                 if val < best_val:
#                     best_val = val; best = s
#             if best is None:
#                 break
#             cur = best
#         if cur == goal_wp.id:
#             path_ids.append(goal_wp.id)
#         with planner_lock:
#             planner_path = path_ids
#         time.sleep(0.4)

# i = 0
# try:
#     vehicle = world.spawn_actor(vehicle_bp, point_a_spawn)
#     print ("starting vehicle spawn: ", point_a_spawn)
#     vehicle_2 = world.spawn_actor(vehicle_bp_2, point_c_spawn)

#     agent = BasicAgent(vehicle)
#     agent.set_destination(point_b)

#     wp_start = nearest_wp_to_location(point_a)
#     wp_goal  = nearest_wp_to_location(point_b)

#     planner_t = threading.Thread(target=planner_thread_fn, args=(wp_start, wp_goal, vehicle_2), daemon=True)
#     planner_t.start()

#     i = 0
#     while True:
#         if agent.done():
#             print("The target has been reached, stopping the simulation")
#             break
#         with planner_lock:
#             path_copy = list(planner_path)
#         if path_copy:
#             veh_loc = vehicle.get_location()
#             nearest_idx = 0
#             nd = float('inf')
#             for idx, wid in enumerate(path_copy):
#                 wp_obj = next((w for w in all_waypoints if w.id == wid), None)
#                 if wp_obj is None: continue
#                 d = euclid(wp_obj.transform.location, veh_loc)
#                 if d < nd:
#                     nd = d; nearest_idx = idx
#             target_idx = min(nearest_idx + 3, len(path_copy)-1)
#             target_wp = next((w for w in all_waypoints if w.id == path_copy[target_idx]), None)
#             if target_wp:
#                 agent.set_destination(target_wp.transform.location)
#         control = agent.run_step()
#         vehicle.apply_control(control)

#         i += 1
#         time.sleep(0.05)

# finally:
#     stop_planner.set()
#     try:
#         planner_t.join(timeout=1.0)
#     except Exception:
#         pass
#     try:
#         vehicle.destroy()
#         vehicle_2.destroy()
#     except Exception:
#         pass
#     print('Cleaned up and exiting.')


import carla
import sys

sys.path.append('../')
from agents.navigation.global_route_planner import GlobalRoutePlanner
import random
from agents.navigation.basic_agent import BasicAgent
import threading, time
# from agents.navigation.global_route_planner_og import GlobalRoutePlanner # original route planner
# from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO

client = carla.Client("localhost", 4000)
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
# point_a_spawn = carla.Transform(carla.Location(point_a.x - 100, point_a.y, point_a.z), point_a_spawn.rotation) # forward spawn
# point_d_spawn = carla.Transform(carla.Location(point_a.x - 85, point_a.y + 1.25, point_a.z), point_b_spawn.rotation) # Full ahead road block
# point_d_spawn = carla.Transform(carla.Location(point_b.x + 1, point_b.y + 10, point_a.z), point_a_spawn.rotation) # Right Turn Partially blocked
# point_c_waypoint = amap.get_waypoint(carla.Location(point_a.x - 20, point_a.y, point_a.z))

print("Point c:", point_c_spawn)

wp_start = amap.get_waypoint(point_a)
wp_end   = amap.get_waypoint(point_b)

def background_planner():
    while True:
        try:
            grp.trace_route(wp_start.transform.location,
                            wp_end.transform.location, world)
        except Exception as e:
            print("Planner error:", e)
        time.sleep(0.5)

planner_thread = threading.Thread(target=background_planner, daemon=True)
planner_thread.start()

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
    agent = BasicAgent(vehicle) # Creating a vehicle for agent
    agent.set_destination(point_b) #Set Location Destination

    # print(vehicle.get_location().x, ",", vehicle.get_location().y, point_a_spawn)
    # print(vehicle_2.get_location().x, ",", vehicle.get_location().y, point_c_spawn)

    i = 0
    while True:
        # if (i % 1000 == 0):
        #     print(vehicle.get_location().x, ",", vehicle.get_location().y)
        if agent.done():
            print("The target has been reached, stopping the simulation")
            break
        vehicle.apply_control(agent.run_step())
        i += 1

finally:
    destroyed_sucessfully = vehicle.destroy()
    destroyed_sucessfully = vehicle_2.destroy()