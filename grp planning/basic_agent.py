# Copyright (c) # Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module implements an agent that roams around a track following random
waypoints and avoiding other vehicles. The agent also responds to traffic lights.
It can also make use of the global route planner to follow a specifed route
"""

import carla
from shapely.geometry import Polygon

'''
import os, sys
HERE      = os.path.dirname(__file__)                   
AGENTS    = os.path.abspath(os.path.join(HERE, os.pardir))    
CARLA_ROOT= os.path.abspath(os.path.join(HERE, os.pardir, os.pardir)) 

sys.path.insert(0, AGENTS)
sys.path.insert(0, CARLA_ROOT)
'''
from agents.navigation.local_planner import LocalPlanner, RoadOption
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.tools.misc import (
        get_speed, is_within_distance,
        get_trafficlight_trigger_location,
        compute_distance
    )
from agents.navigation.collision import car_mesh


import numpy as np
from itertools import tee
import threading

def loc_to_vec(loc: carla.Location) -> np.ndarray:
        return np.array([loc.x, loc.y, loc.z], dtype=float)

def vec_to_loc(v: np.ndarray) -> carla.Location:
    return carla.Location(float(v[0]), float(v[1]), float(v[2]))

def bz_curve(p0, p1, p2, t):
    u = 1.0 - t
    return (u*u) * p0 + 2*u*t * p1 + (t*t) * p2

def cubic_bz(p0, p1, p2, p3, t):
    u = 1.0 - t
    return (u**3) * p0 \
         + 3 * (u**2) * t * p1 \
         + 3 * u * (t**2) * p2 \
         + (t**3) * p3

def bz_velocity(p0: np.ndarray, p1: np.ndarray, p2: np.ndarray, p3: np.ndarray, t: float) -> np.ndarray:
    #First derivative of a cubic bezier 
    u = 1.0 - t
    return 3 * ((u**2) * (p1 - p0) + 2 * u * t * (p2 - p1) + (t**2) * (p3 - p2))

def bz_acc(p0: np.ndarray, p1: np.ndarray, p2: np.ndarray, p3: np.ndarray, t: float) -> np.ndarray:
    #Second derivative of a cubic bezier 
    u = 1.0 - t
    return 6 * (u * (p2 - 2*p1 + p0) + t * (p3 - 2*p2 + p1))

def bz_curvature(p0: np.ndarray, p1: np.ndarray, p2: np.ndarray, p3: np.ndarray, t: float) -> float:
    d1 = bz_velocity(p0, p1, p2, p3, t)
    d2 = bz_acc(p0, p1, p2, p3, t)
    cross = np.cross(d1, d2)
    num   = np.linalg.norm(cross)
    den   = np.linalg.norm(d1)**3
    return float(num / den)

def jaggedness(route): 
    wp_xy = []
    for item in route: 
        wp = item[0] if isinstance(item, (tuple, list)) and len(item) >= 1 else item
        if hasattr(wp, "location"):
            wp_loc = wp.location
        elif hasattr(wp, "transform") and hasattr(wp.transform, "location"):
                wp_loc = wp.transform.location
        else: 
            continue 
        wp_xy.append([wp_loc.x, wp_loc.y])
    if(len(wp_xy) < 3):
        return "zero"
    stacked_list = np.stack(wp_xy, axis = 0)
    head_angle = np.unwrap(np.arctan2(np.diff(stacked_list[:,1]), np.diff(stacked_list[:,0])))
    diff_angle = np.diff(head_angle)
    abs_angle = np.abs(diff_angle)
    sum_of_angles = np.sum(abs_angle)
    return sum_of_angles

def pairwise(iterable):
    a, b = tee(iterable)
    next(b, None)
    return zip(a, b)


class BasicAgent(object):
    """
    BasicAgent implements an agent that navigates the scene.
    This agent respects traffic lights and other vehicles, but ignores stop signs.
    It has several functions available to specify the route that the agent must follow,
    as well as to change its parameters in case a different driving mode is desired.
    """

    def __init__(self, vehicle, target_speed=20, opt_dict={}, map_inst=None, grp_inst=None):
        """
        Initialization the agent paramters, the local and the global planner.

            :param vehicle: actor to apply to agent logic onto
            :param target_speed: speed (in Km/h) at which the vehicle will move
            :param opt_dict: dictionary in case some of its parameters want to be changed.
                This also applies to parameters related to the LocalPlanner.
            :param map_inst: carla.Map instance to avoid the expensive call of getting it.
            :param grp_inst: GlobalRoutePlanner instance to avoid the expensive call of getting it.

        """
        self._vehicle = vehicle
        self._world = self._vehicle.get_world()
        if map_inst:
            if isinstance(map_inst, carla.Map):
                self._map = map_inst
            else:
                print("Warning: Ignoring the given map as it is not a 'carla.Map'")
                self._map = self._world.get_map()
        else:
            self._map = self._world.get_map()
        self._last_traffic_light = None

        # Base parameters
        self._ignore_traffic_lights = False
        self._ignore_stop_signs = False
        self._ignore_vehicles = False
        self._use_bbs_detection = False
        self._target_speed = target_speed
        self._sampling_resolution = 2.0
        self._base_tlight_threshold = 5.0  # meters
        self._base_vehicle_threshold = 5.0  # meters
        self._speed_ratio = 1
        self._max_brake = 0.5
        self._offset = 0
        self._destination = None
        self._previous_obstacle = None
        self._prev_obs = False
        self._seen_obstacles = {}
        self._seen_obstacles_lock = threading.Lock()
        # self._obstacle_ttl_ticks = 8
        self._obstacle_ttl_ticks = 200
        self._position_change_threshold = 0.05
        self._obstacle_prediction_seconds = 1.0

        # Lane Change
        self._lc_attempts = 0
        self._prev_lane = -1
        self._debug = False

        # Sensor attachment
        blueprint = self._world.get_blueprint_library().find('sensor.other.obstacle')
        bb_extent = self._vehicle.bounding_box.extent
        sensor_z = bb_extent.z + 0.7

        def _spawn_obstacle_sensor(x, y, yaw, distance_m):
            blueprint.set_attribute('distance', str(distance_m))
            blueprint.set_attribute('hit_radius', '1.5')
            blueprint.set_attribute('only_dynamics', 'False')
            blueprint.set_attribute('debug_linetrace', 'True')
            blueprint.set_attribute('sensor_tick', '0.0')
            transform = carla.Transform(
                carla.Location(x=x, y=y, z=sensor_z),
                carla.Rotation(yaw=yaw)
            )
            return self._world.spawn_actor(blueprint, transform, attach_to=self._vehicle)

        self._front_obstacle_sensor = _spawn_obstacle_sensor(bb_extent.x, 0.0, 0.0, 20.0)
        self._rear_obstacle_sensor = _spawn_obstacle_sensor(-bb_extent.x, 0.0, 180.0, 6.0)
        self._left_obstacle_sensor = _spawn_obstacle_sensor(0.0, -bb_extent.y, -90.0, 6.0)
        self._right_obstacle_sensor = _spawn_obstacle_sensor(0.0, bb_extent.y, 90.0, 6.0)
        self._front_left_obstacle_sensor = _spawn_obstacle_sensor(bb_extent.x, -bb_extent.y, -45.0, 8.0)
        self._front_right_obstacle_sensor = _spawn_obstacle_sensor(bb_extent.x, bb_extent.y, 45.0, 8.0)
        self._rear_left_obstacle_sensor = _spawn_obstacle_sensor(-bb_extent.x, -bb_extent.y, -135.0, 8.0)
        self._rear_right_obstacle_sensor = _spawn_obstacle_sensor(-bb_extent.x, bb_extent.y, 135.0, 8.0)

        self._obstacle_sensors = [
            self._front_obstacle_sensor,
            self._rear_obstacle_sensor,
            self._left_obstacle_sensor,
            self._right_obstacle_sensor,
            self._front_left_obstacle_sensor,
            self._front_right_obstacle_sensor,
            self._rear_left_obstacle_sensor,
            self._rear_right_obstacle_sensor,
        ]

        def obstacle_callback(event: carla.ObstacleDetectionEvent):
            obstacle = event.other_actor
            if obstacle and obstacle.type_id.startswith("vehicle"):
                self._update_seen_obstacle(obstacle)

        for sensor in self._obstacle_sensors:
            sensor.listen(obstacle_callback)

        # Change parameters according to the dictionary
        opt_dict['target_speed'] = target_speed
        if 'ignore_traffic_lights' in opt_dict:
            self._ignore_traffic_lights = opt_dict['ignore_traffic_lights']
        if 'ignore_stop_signs' in opt_dict:
            self._ignore_stop_signs = opt_dict['ignore_stop_signs']
        if 'ignore_vehicles' in opt_dict:
            self._ignore_vehicles = opt_dict['ignore_vehicles']
        if 'use_bbs_detection' in opt_dict:
            self._use_bbs_detection = opt_dict['use_bbs_detection']
        if 'sampling_resolution' in opt_dict:
            self._sampling_resolution = opt_dict['sampling_resolution']
        if 'base_tlight_threshold' in opt_dict:
            self._base_tlight_threshold = opt_dict['base_tlight_threshold']
        if 'base_vehicle_threshold' in opt_dict:
            self._base_vehicle_threshold = opt_dict['base_vehicle_threshold']
        if 'detection_speed_ratio' in opt_dict:
            self._speed_ratio = opt_dict['detection_speed_ratio']
        if 'max_brake' in opt_dict:
            self._max_brake = opt_dict['max_brake']
        if 'offset' in opt_dict:
            self._offset = opt_dict['offset']
        if 'obstacle_prediction_seconds' in opt_dict:
            self._obstacle_prediction_seconds = float(opt_dict['obstacle_prediction_seconds'])

        # Initialize the planners
        # if isinstance(grp_inst, GlobalRoutePlanner):
            self._global_planner = grp_inst
        else:
            print("Warning: Ignoring the given map as it is not a 'carla.Map'")
            self._global_planner = GlobalRoutePlanner(self._map, self._sampling_resolution)


        self._local_planner = LocalPlanner(self._vehicle, opt_dict=opt_dict, map_inst=self._map)

        # Get the static elements of the scene
        self._lights_list = self._world.get_actors().filter("*traffic_light*")
        self._lights_map = {}  # Dictionary mapping a traffic light to a wp corrspoing to its trigger volume location

    def add_emergency_stop(self, control):
        """
        Overwrites the throttle a brake values of a control to perform an emergency stop.
        The steering is kept the same to avoid going out of the lane when stopping during turns

            :param speed (carl.VehicleControl): control to be modified
        """
        control.throttle = 0.0
        control.brake = self._max_brake
        control.hand_brake = False
        return control

    def set_target_speed(self, speed):
        """
        Changes the target speed of the agent
            :param speed (float): target speed in Km/h
        """
        self._target_speed = speed
        self._local_planner.set_speed(speed)

    def follow_speed_limits(self, value=True):
        """
        If active, the agent will dynamically change the target speed according to the speed limits

            :param value (bool): whether or not to activate this behavior
        """
        self._local_planner.follow_speed_limits(value)

    def get_local_planner(self):
        """Get method for protected member local planner"""
        return self._local_planner

    def get_global_planner(self):
        """Get method for protected member local planner"""
        return self._global_planner

    def set_destination(self, end_location, start_location=None):
        """
        This method creates a list of waypoints between a starting and ending location,
        based on the route returned by the global router, and adds it to the local planner.
        If no starting location is passed, the vehicle local planner's target location is chosen,
        which corresponds (by default), to a location about 5 meters in front of the vehicle.

            :param end_location (carla.Location): final location of the route
            :param start_location (carla.Location): starting location of the route
        """
        # New parameter: new_obstacle. The current plan is to call set_destination in the event
        # of a detected obstacle, so thus we can feed that obstacle towards the algorithm
        # and appropriately replan. 
        # In normal usage, its value being None ensures that normal operation of the function
        # is sustained. Further, there was a key decision that the agent class is not holding
        # the obstacles as information but just passes it on to the algorthim, such that it can
        # be simulated that the algorithm is working with a new iteration of the map its developing
        # an algorithm on.
        #
        # if not start_location:
        #     start_location = self._local_planner.target_waypoint.transform.location
        #     # start_location = self._vehicle.get_location()
        #     clean_queue = True
        # else:
        #     start_location = self._vehicle.get_location()
        #     clean_queue = False
        #
        # start_waypoint = self._map.get_waypoint(start_location)
        # print("basic_agent start location: " , start_location)
        # end_waypoint = self._map.get_waypoint(end_location)
        # self._destination = end_location
        #
        # if self._debug:
        #     self._world.debug.draw_string(start_waypoint.transform.location, 'START', draw_shadow=False,
        #     color=carla.Color(r=255, g=255, b=0), life_time=5.0,
        #     persistent_lines=True)
        #
        # route_trace = self.trace_route(start_waypoint, end_waypoint, new_obstacle)

        if type(end_location) is not list:
            if not start_location:
                if not isinstance(self._local_planner.target_waypoint, carla.Waypoint):
                    start_location = self._local_planner.target_waypoint
                else:
                    start_location = self._local_planner.target_waypoint.transform.location
                # start_location = self._vehicle.get_location()
                clean_queue = True
            else:
                start_location = self._vehicle.get_location()
                clean_queue = False

            start_waypoint = self._map.get_waypoint(start_location)
            print("basic_agent start location: " , start_location)
            print("basic_agent end location: " , end_location)
            end_waypoint = self._map.get_waypoint(end_location)
            self._destination = end_location

            route_trace = self.trace_route(start_waypoint, end_waypoint)
        else:
            route_trace = end_location
            clean_queue = True


        # route trace is a list of routes
        # these now have to be connected via lane change links.

        # i = 0
        # for route in route_trace:
        #     for w in route:s
        #         # print(w[0].transform.location.x, ",",w[0].transform.location.y, w[1])
        #         if i % 10 == 0:
        #             self._world.debug.draw_string(w[0].transform.location, f'{i}', draw_shadow=False,
        #             color=carla.Color(r=255, g=0, b=0), life_time=120.0,
        #             persistent_lines=True)
        #         else:
        #             self._world.debug.draw_string(w[0].transform.location, f'{i}', draw_shadow=False,
        #             color = carla.Color(r=0, g=0, b=255), life_time=60.0,
        #             persistent_lines=True)
        #         i += 1

        # depending on the car's rotation, we can figure out what lane
        # they want to change to.

        # Applied bezier curve to route
        use_bezier = True

        for i in range(len(route_trace)):
            if (i != len(route_trace) - 1):
                yaw = route_trace[i][-1][0].transform.rotation.yaw

                x_1, y_1 = route_trace[i][-1][0].transform.location.x, route_trace[i][-1][0].transform.location.y
                # extended end for bezier curves

                wp_min_distance = 1
                wp_starting_distance = 3
                # wp_distance = wp_starting_distance
                wp_distance = min(wp_starting_distance, len(route_trace[i+1]) - 1)

                print(" - checking for collisions")
                collisions = self._seen_obstacle_collisions_at_waypoint(route_trace[i + 1][wp_distance][0])
                print(" - {} collisions found".format(len(collisions)))

                j = 0

                while len(collisions) > 0:
                    print(" - detected collision at lane change head, moving route head...")

                    wp_distance -= 1


                    if wp_distance == wp_min_distance:

                        print(" - minimum head distance reached, waiting for obstacle to clear...")
                        wp_distance = wp_starting_distance
                        break

                    collisions = self._seen_obstacle_collisions_at_waypoint(route_trace[i + 1][wp_distance][0])
                    print(" - {} collisions found".format(len(collisions)))

                    j += 1

                x_2, y_2 = route_trace[i + 1][wp_distance][0].transform.location.x, route_trace[i + 1][0][0].transform.location.y

                # Bezier cruves
                p0 = loc_to_vec(route_trace[i][-1][0].transform.location) 
                p1 = np.array([x_1 + (x_2 - x_1) * 0.75, y_1 + (y_2 - y_1) * 0.0, 0.0]) # control point
                p2 = np.array([x_2 - (x_2 - x_1) * 0.75, y_2 - (y_2 - y_1) * 0.0, 0.0]) # control point
                # p3 = loc_to_vec(route_trace[i + 1][3][0].transform.location)
                p3 = loc_to_vec(route_trace[i + 1][wp_distance][0].transform.location)

                # Regular end for regular curves
                x_2, y_2 = route_trace[i + 1][0][0].transform.location.x, route_trace[i + 1][0][0].transform.location.y   

                # print("p0: ", p0, "p1: ", p1, "p2: ", p2, "p3: ", p3)     

                # print("bezier curves: ", p0, p2, p1)    
                #right_vec = current_node.waypoint.transform.get_right_vector()
                #cp_up    = p0 + np.array([0.0, 0.0, 3.0])              
                #cp_right = p0 + np.array([right_vec.x, right_vec.y, right_vec.z]) * 3.0
                #p1 = cp_up 

                alternate_lane_path = []
                for t in np.linspace(0.0, 1.0, 30, endpoint=True):         
                    pt = cubic_bz(p0, p1, p2, p3, t)
                    #print("1")
                    curve = bz_curvature(p0, p1, p2, p3, t)
                    # print(f"t={t:.3}, curvature={curve:.6f}")
                    loc = vec_to_loc(pt)
                    rot = (route_trace[i + 1][0][0]).transform.rotation 
                    #wpt = map.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
                    #print("Waypoint test: ", wpt)

                    # Print points
                    # self._world.debug.draw_point(
                    #     vec_to_loc(pt),
                    #     size=0.08,
                    #     color=carla.Color(0, 0, 255),
                    #     life_time=15.0)
                     
                    alternate_lane_path.append((loc, RoadOption.LANEFOLLOW))    

                    '''
                    else: 
                        new_wp = carla.Transform(loc, rot)
                        alternate_lane_path.append((new_wp, RoadOption.LANEFOLLOW))
                    '''
                    
                    #alternate_lane_path.append((wpt), RoadOption.LANEFOLLOW)
                    
                #insert the lin space curvature calc here

                ###############

                lane_path = None

                if yaw < 185 and yaw > 175:
                    if y_2 < y_1:
                        lane_path = self._generate_lane_change_path(route_trace[i][-1][0], 'right', 0,0,1)
                    else:
                        lane_path = self._generate_lane_change_path(route_trace[i][-1][0], 'left', 0,0,1)

                    # print("lane change vertical: ", lane_path)
                    if not use_bezier:
                        print("NO bz")
                        route_trace[i] = route_trace[i] + lane_path
                    
                if yaw < -85 and yaw > -95:
                    if x_2 < x_1:
                        lane_path = self._generate_lane_change_path(route_trace[i][-1][0], 'right', 0,0,1)
                    else:
                        lane_path = self._generate_lane_change_path(route_trace[i][-1][0], 'left', 0,0,1)

                    if not use_bezier:
                        print("No bz")
                        route_trace[i] = route_trace[i] + lane_path

                if use_bezier:
                    print("YES bz")
                    route_trace[i] = route_trace[i] + alternate_lane_path

                # print("Lane_path: ")
                
                if lane_path:
                    for pt in lane_path:
                        waypt, road_opt = pt
                        # print(waypt, road_opt)
                    
        final_route = []

        # for route in route_trace:
        #     final_route = final_route + route

        for r in range(len(route_trace)):
            final_route = final_route + route_trace[r]

            final_route_end = final_route[-1][0].transform.location if isinstance(final_route[-1][0], carla.Waypoint) else final_route[-1][0]

            if r != len(route_trace) - 1:
                entry_idx = 0

                for w in range(len(route_trace[r+1])):
                    pt = route_trace[r+1][w][0]

                    next_route_head = pt.transform.location if isinstance(pt, carla.Waypoint) else pt

                    if final_route_end.distance(next_route_head) < 0.1:
                        entry_idx = w
                        break

                route_trace[r+1] = route_trace[r+1][entry_idx:]

        # Drawn Line with carla path
        for i in range(len(final_route) - 1):
            curr_fr = final_route[i][0]
            next_fr = final_route[i+1][0]

            if isinstance(curr_fr, carla.Waypoint): 
                curr_loc = curr_fr.transform.location
            else: 
                if isinstance(curr_fr, carla.Location): 
                    curr_loc = curr_fr

            if isinstance(next_fr, carla.Waypoint): 
                next_loc = next_fr.transform.location
            else: 
                if isinstance(next_fr, carla.Location): 
                    next_loc = next_fr

            self._world.debug.draw_line(
                curr_loc, next_loc,
                thickness=0.25,
                color=carla.Color(r=0, g=0, b=50, a=150),
                life_time=10.0
            )

        #print("set_destination: ", final_route)
        print(f"Jaggedness:", jaggedness(final_route))

        self._local_planner.set_global_plan(final_route, clean_queue=clean_queue)

        self._route_cache = final_route

        return

    def set_global_plan(self, plan, stop_waypoint_creation=True, clean_queue=True):
        """
        Adds a specific plan to the agent.

            :param plan: list of [carla.Waypoint, RoadOption] representing the route to be followed
            :param stop_waypoint_creation: stops the automatic random creation of waypoints
            :param clean_queue: resets the current agent's plan
        """
        self._local_planner.set_global_plan(
            plan,
            stop_waypoint_creation=stop_waypoint_creation,
            clean_queue=clean_queue
        )

    def trace_route(self, start_waypoint, end_waypoint):
        """
        Calculates the shortest route between a starting and ending waypoint.

            :param start_waypoint (carla.Waypoint): initial waypoint
            :param end_waypoint (carla.Waypoint): final waypoint
        """
        # New parameter: new_obstacle. This feeds the obstacle into the grp.

        start_location = start_waypoint.transform.location
        end_location = end_waypoint.transform.location

        seen_obstacles_snapshot = self._get_seen_obstacles_snapshot()

        return self._global_planner.trace_route(
            start_location,
            end_location,
            self._world,
            seen_obstacles_snapshot
        )

    def _seen_obstacle_collisions_at_waypoint(self, waypoint_or_location):
        """Return seen obstacle actors whose mesh footprint contains the given waypoint/location."""
        collisions = []

        for obs_data in self._get_seen_obstacles_snapshot():
            actor = obs_data['actor']
            if not actor.is_alive:
                continue

            obs_mesh = obs_data['mesh']
            if obs_mesh.contains_waypoint(waypoint_or_location):
                collisions.append(actor)

        return collisions

    def _at_junction(self, wp_lookahead=2):
        if self._map.get_waypoint(self._vehicle.get_location()).is_junction:
            return True
        elif wp_lookahead <= 0:
            return False
        else:
            lookahead = wp_lookahead
            while len(self._local_planner._waypoints_queue) <= lookahead:
                lookahead = lookahead - 1
                if lookahead == 0: return False

            for i in range(lookahead):
                target_wpt = self._local_planner._waypoints_queue[i][0]
                if target_wpt.is_junction:
                    return True

            return False

    def _get_goal_distance(self):
        return self._vehicle.get_location().distance(self._destination)

    def run_step(self):
        """Execute one step of navigation."""
        self._tick_seen_obstacles()

        # Two hazard detection booleans to avoid replanning just for the sake of
        # avoiding a light. Though problem is the consideration if the vehicle
        # classified as obstacle is part of light.
        # For now, since vehicles are obstacles, I think its fine to have them be
        # assigned as hazards, until we have proper obstaclrd.

        hazard_obstacle = False
        hazard_light = False

        # Retrieve all relevant actors
        vehicle_list = self._world.get_actors().filter("*vehicle*")

        # vehicle_speed = get_speed(self._vehicle) / 3.6
        vehicle_speed = get_speed(self._vehicle) / 5

        # Sensor-based blocking check over current plan
        min_vehicle_distance = 25
        ego_location = self._vehicle.get_location()
        plan_queue = list(self._local_planner.get_plan())
        plan_waypoints = [wp for wp, _ in plan_queue]
        blocking_candidates = []


        for obs_data in self._get_seen_obstacles_snapshot():
            actor = obs_data['actor']
            if not actor.is_alive:
                continue

            obs_mesh = obs_data['mesh']
            for plan_wp in plan_waypoints:
                if isinstance(plan_wp, carla.libcarla.Waypoint):
                    plan_wp_location = plan_wp.transform.location
                else:
                    plan_wp_location = plan_wp

                if ego_location.distance(plan_wp_location) > min_vehicle_distance:
                    continue

                if obs_mesh.contains_waypoint(plan_wp, radius_m=0.2):
                    obstacle_velocity = obs_data.get('velocity')
                    if obstacle_velocity is None:
                        obstacle_velocity = actor.get_velocity()

                    obstacle_speed = obstacle_velocity.length() if obstacle_velocity is not None else 0.0

                    if obstacle_speed < 100.0:
                        hazard_obstacle = True
                        blocking_candidates.append(actor.id)
                    break

        # Deduplicate while preserving detection order.
        blocking_candidates = list(dict.fromkeys(blocking_candidates))

        # if blocking_candidates:
        #     print("Blocking candidates detected: ")
        #     for candidate in blocking_candidates:
        #         candidate_loc = self.get_detected_agent_attribute(candidate, 'location')
        #         if candidate_loc is None:
        #             candidate_actor = self._world.get_actor(candidate)
        #             candidate_loc = candidate_actor.get_location() if candidate_actor else None
        #         if candidate_loc is None:
        #             continue
        #         print(candidate_loc)
        #         self._world.debug.draw_string(candidate_loc, 'Blocking Candidate', draw_shadow=False,
        #             color=carla.Color(r=255, g=0, b=0), life_time=15.0,
        #             persistent_lines=True)

        # Check if the vehicle is affected by a red traffic light
        max_tlight_distance = self._base_tlight_threshold + self._speed_ratio * vehicle_speed
        affected_by_tlight, _ = self._affected_by_traffic_light(self._lights_list, max_tlight_distance)
        if affected_by_tlight:
            hazard_light = True

        control = self._local_planner.run_step()

        if hazard_obstacle and hazard_light:
            control = self.add_emergency_stop(control)
        elif hazard_obstacle:

            obstacle_adjacent_lane_blocked = False
            for candidate_id in blocking_candidates:
                obstacle_loc = self.get_detected_agent_attribute(
                    candidate_id,
                    'location',
                    default=None
                )

                if obstacle_loc is None:
                    obstacle_actor = self._world.get_actor(candidate_id)
                    if obstacle_actor:
                        obstacle_loc = obstacle_actor.get_location()

                if obstacle_loc is None:
                    continue

                obstacle_wpt = self._map.get_waypoint(obstacle_loc, lane_type=carla.LaneType.Any)
                if self._adjacent_lane_waypoints_blocked(obstacle_wpt):
                    obstacle_adjacent_lane_blocked = True
                    break

            # if obstacle_adjacent_lane_blocked:
            #     print("adjacent lane blocked: ", obstacle_wpt)

            agent_adjacent_lane_blocked = self._adjacent_lane_waypoints_blocked(self._map.get_waypoint(self._vehicle.get_location()))

            current_blocking_ids = set(blocking_candidates)

            # if obstacle_wpt and (adjacent_lane_blocked or (left_blocked and right_blocked)):
            if blocking_candidates and obstacle_adjacent_lane_blocked:
                print("Adjacent lane waypoint(s) blocked near obstacle. Emergency stopping.")
                control = self.add_emergency_stop(control)

            # Currently not considering if any of the obstacles are moving,
            # just if it is the same vehicles that have been detected before, 
            # then we can assume that they are not moving and thus we should stop.
            # Though later we should take these agents movement into consideration
            elif blocking_candidates and (self._previous_obstacle is None or current_blocking_ids != self._previous_obstacle):
                print("Entered obstacle resolution")
                print("Replanning around obstacle: ", end="")
                for candidate in blocking_candidates:
                    candidate_loc = self.get_detected_agent_attribute(candidate, 'location')
                    if candidate_loc is None:
                        candidate_actor = self._world.get_actor(candidate)
                        candidate_loc = candidate_actor.get_location() if candidate_actor else None
                    print(candidate_loc, end="; ")
                print()

                self._previous_obstacle = current_blocking_ids
                self.set_destination(self._destination, None)

                for candidate in blocking_candidates:
                    candidate_loc = self.get_detected_agent_attribute(candidate, 'location')
                    if candidate_loc is None:
                        candidate_actor = self._world.get_actor(candidate)
                        candidate_loc = candidate_actor.get_location() if candidate_actor else None
                    if candidate_loc is None:
                        continue
                    self._world.debug.draw_string(candidate_loc, 'Obstacle', draw_shadow=False,
                    color=carla.Color(r=255, g=0, b=0), life_time=15.0,
                    persistent_lines=True)

            elif blocking_candidates:
                # Hazard is still present but neither a new replan set nor an adjacent-lane
                # blocked condition fired this tick. Keep braking to avoid rolling forward
                # through a transient geometry/condition flip.
                control = self.add_emergency_stop(control)

        self._prev_obs = hazard_obstacle
        self._prev_lane = self._map.get_waypoint(self._vehicle.get_location()).lane_id

        # print(control)

        return control
    
    def _draw_string(self, string, location, color=carla.Color(r=255,g=0,b=0), shadow=False, life_time=0.0):
        """
        Use self._world to draw string at location
        """
        if life_time > 0.0:
            self._world.debug.draw_string(
                location,
                string,
                draw_shadow=shadow,
                color=color,
                life_time=life_time
            )
        else:
            self._world.debug.draw_string(
                location,
                string,
                draw_shadow=shadow,
                color=color
            )

    def _update_seen_obstacle(self, obstacle_actor):
        """Add/update an obstacle in the rotating obstacle set."""
        try:
            obstacle_mesh = car_mesh(obstacle_actor)
            current_location = obstacle_actor.get_location()
            current_velocity = obstacle_actor.get_velocity()
            with self._seen_obstacles_lock:
                previous_entry = self._seen_obstacles.get(obstacle_actor.id)
                is_new_obstacle = previous_entry is None
                previous_location = previous_entry['location'] if previous_entry else None
                position_changed = True
                position_vector_xy = None
                if previous_location:
                    position_changed = (
                        current_location.distance(previous_location) > self._position_change_threshold
                    )
                    raw_position_vector_xy = np.array([
                        current_location.x - previous_location.x,
                        current_location.y - previous_location.y
                    ], dtype=float)
                    vector_norm = np.linalg.norm(raw_position_vector_xy)
                    if vector_norm > 0.0:
                        position_vector_xy = raw_position_vector_xy / vector_norm

                self._seen_obstacles[obstacle_actor.id] = {
                    'actor': obstacle_actor,
                    'mesh': obstacle_mesh,
                    'ttl': self._obstacle_ttl_ticks,
                    'location': current_location,
                    'velocity': current_velocity,
                    'position_changed': position_changed,
                    'position_vector_xy': position_vector_xy
                }

                if is_new_obstacle:
                    print(
                        f"[Obstacle Detected] id={obstacle_actor.id}, "
                        f"type={obstacle_actor.type_id}, "
                        f"location={current_location}"
                    )
        except RuntimeError:
            return

    def _get_seen_obstacles_snapshot(self):
        """Thread-safe snapshot copy of currently tracked obstacles."""
        with self._seen_obstacles_lock:
            return [
                {
                    'actor': obs_data['actor'],
                    'mesh': obs_data['mesh'],
                    'ttl': obs_data['ttl'],
                    'location': obs_data.get('location'),
                    'velocity': obs_data.get('velocity'),
                    'position_changed': obs_data.get('position_changed', False),
                    'position_vector_xy': obs_data.get('position_vector_xy')
                }
                for obs_data in self._seen_obstacles.values()
            ]

    def get_detected_agent(self, actor_id):
        """Return a copy of the stored data for a detected agent by actor id, or None."""
        with self._seen_obstacles_lock:
            obs_data = self._seen_obstacles.get(actor_id)
            if obs_data is None:
                return None

            return {
                'actor': obs_data['actor'],
                'mesh': obs_data['mesh'],
                'ttl': obs_data['ttl'],
                'location': obs_data.get('location'),
                'velocity': obs_data.get('velocity'),
                'position_changed': obs_data.get('position_changed', False),
                'position_vector_xy': obs_data.get('position_vector_xy')
            }

    def get_detected_agent_attribute(self, actor_id, attribute_name, default=None):
        """Return one stored attribute for a detected agent by actor id, else default."""
        agent_data = self.get_detected_agent(actor_id)
        if agent_data is None:
            return default
        return agent_data.get(attribute_name, default)

    def _tick_seen_obstacles(self):
        """Refresh obstacle TTL each tick and remove stale observations."""
        with self._seen_obstacles_lock:
            obstacle_items = list(self._seen_obstacles.items())

        expired_ids = set()
        refreshed_data = {}

        for obs_id, obs_data in obstacle_items:
            actor = obs_data['actor']

            if not actor.is_alive:
                expired_ids.add(obs_id)
                continue

            try:
                refreshed_mesh = car_mesh(actor)
                refreshed_location = actor.get_location()
                refreshed_velocity = actor.get_velocity()
            except RuntimeError:
                expired_ids.add(obs_id)
                continue

            refreshed_ttl = obs_data['ttl'] - 1
            previous_location = obs_data.get('location')
            position_changed = True
            position_vector_xy = None
            if previous_location:
                position_changed = (
                    refreshed_location.distance(previous_location) > self._position_change_threshold
                )
                raw_position_vector_xy = np.array([
                    refreshed_location.x - previous_location.x,
                    refreshed_location.y - previous_location.y
                ], dtype=float)
                vector_norm = np.linalg.norm(raw_position_vector_xy)
                if vector_norm > 0.0:
                    position_vector_xy = raw_position_vector_xy / vector_norm

            refreshed_data[obs_id] = {
                'mesh': refreshed_mesh,
                'ttl': refreshed_ttl,
                'location': refreshed_location,
                'velocity': refreshed_velocity,
                'position_changed': position_changed,
                'position_vector_xy': position_vector_xy
            }

            self._draw_obstacle_bbox(refreshed_mesh)
            self._draw_predicted_obstacle_bbox(
                refreshed_mesh,
                refreshed_velocity,
                self._obstacle_prediction_seconds
            )

            if refreshed_ttl <= 0:
                expired_ids.add(obs_id)

        with self._seen_obstacles_lock:
            for obs_id in expired_ids:
                self._seen_obstacles.pop(obs_id, None)

            for obs_id, refreshed in refreshed_data.items():
                if obs_id not in self._seen_obstacles:
                    continue
                if obs_id in expired_ids:
                    continue
                self._seen_obstacles[obs_id]['mesh'] = refreshed['mesh']
                self._seen_obstacles[obs_id]['ttl'] = refreshed['ttl']
                self._seen_obstacles[obs_id]['location'] = refreshed['location']
                self._seen_obstacles[obs_id]['velocity'] = refreshed['velocity']
                self._seen_obstacles[obs_id]['position_changed'] = refreshed['position_changed']
                self._seen_obstacles[obs_id]['position_vector_xy'] = refreshed['position_vector_xy']

    def _draw_obstacle_bbox(self, mesh_obj):
        """Draw the obstacle footprint by connecting all bounding-box corners with lines."""
        corners = mesh_obj.corners
        if len(corners) < 2:
            return

        cx = sum(p.x for p in corners) / len(corners)
        cy = sum(p.y for p in corners) / len(corners)
        ordered_corners = sorted(corners, key=lambda p: np.arctan2(p.y - cy, p.x - cx))

        for i in range(len(ordered_corners)):
            start = ordered_corners[i]
            end = ordered_corners[(i + 1) % len(ordered_corners)]
            self._world.debug.draw_line(
                start,
                end,
                thickness=0.08,
                color=carla.Color(r=255, g=140, b=0),
                life_time=0.2,
                persistent_lines=False
            )

    def _draw_predicted_obstacle_bbox(self, mesh_obj, velocity, prediction_seconds):
        """Draw predicted obstacle footprint in blue using velocity over prediction_seconds."""
        corners = mesh_obj.corners
        if len(corners) < 2:
            return
        if velocity is None:
            return
        if prediction_seconds <= 0.0:
            return

        velocity_xy = np.array([velocity.x, velocity.y], dtype=float)
        if np.linalg.norm(velocity_xy) <= 0.0:
            return

        displacement_xy = velocity_xy * float(prediction_seconds)
        dx = float(displacement_xy[0])
        dy = float(displacement_xy[1])

        predicted_corners = [
            carla.Location(x=float(c.x + dx), y=float(c.y + dy), z=float(c.z))
            for c in corners
        ]

        cx = sum(p.x for p in predicted_corners) / len(predicted_corners)
        cy = sum(p.y for p in predicted_corners) / len(predicted_corners)
        ordered_corners = sorted(predicted_corners, key=lambda p: np.arctan2(p.y - cy, p.x - cx))

        for i in range(len(ordered_corners)):
            start = ordered_corners[i]
            end = ordered_corners[(i + 1) % len(ordered_corners)]
            self._world.debug.draw_line(
                start,
                end,
                thickness=0.08,
                color=carla.Color(r=0, g=100, b=255),
                life_time=0.2,
                persistent_lines=False
            )

    def _adjacent_lane_waypoints_blocked(self, obstacle_wpt):
        """Return True if left/right adjacent lane waypoint near obstacle is occupied by a seen obstacle mesh."""
        if not obstacle_wpt:
            return False
        
        print("Checking adjacent lanes for obstacle waypoint: ", obstacle_wpt)

        left_wp = obstacle_wpt.get_left_lane()
        right_wp = obstacle_wpt.get_right_lane()
        obs_data = self._get_seen_obstacles_snapshot()

        print("Left WP: ", left_wp, "Right WP: ", right_wp)

        obstacle_fwd = obstacle_wpt.transform.get_forward_vector()

        def is_direction_compatible(candidate_wp):
            if candidate_wp is None:
                return False
            if candidate_wp.lane_type != carla.LaneType.Driving:
                return False
            if candidate_wp.road_id != obstacle_wpt.road_id:
                return False
            # CARLA lane_id sign encodes direction; opposite sign means opposing flow.
            if obstacle_wpt.lane_id * candidate_wp.lane_id <= 0:
                return False

            cand_fwd = candidate_wp.transform.get_forward_vector()
            dot = (
                obstacle_fwd.x * cand_fwd.x +
                obstacle_fwd.y * cand_fwd.y +
                obstacle_fwd.z * cand_fwd.z
            )
            return dot > 0.0

        def waypoint_blocked_by_seen_obstacle(candidate_wp):
            if not is_direction_compatible(candidate_wp):
                return True

            candidate_loc = candidate_wp.transform.location
            for obs in obs_data:
                obs_mesh = obs['mesh']
                if obs_mesh.contains_waypoint(candidate_loc, radius_m=0.5):
                    return True
            return False

        left_blocked = waypoint_blocked_by_seen_obstacle(left_wp)
        right_blocked = waypoint_blocked_by_seen_obstacle(right_wp)

        print("left_blocked:", left_blocked, "right_blocked:", right_blocked)

        if left_blocked and right_blocked:
            return True
        else:
            return False


    def _get_seen_obstacle_waypoints(self):
        """Convert current obstacle set into waypoints for planning."""
        obstacle_wpts = []
        for obs_data in self._get_seen_obstacles_snapshot():
            actor = obs_data['actor']
            if actor.is_alive:
                obstacle_wpts.append(self._map.get_waypoint(actor.get_location(), lane_type=carla.LaneType.Any))
        return obstacle_wpts

    def done(self):
        """Check whether the agent has reached its destination."""
        return self._local_planner.done()

    def ignore_traffic_lights(self, active=True):
        """(De)activates the checks for traffic lights"""
        self._ignore_traffic_lights = active

    def ignore_stop_signs(self, active=True):
        """(De)activates the checks for stop signs"""
        self._ignore_stop_signs = active

    def ignore_vehicles(self, active=True):
        """(De)activates the checks for stop signs"""
        self._ignore_vehicles = active

    def set_offset(self, offset):
        """Sets an offset for the vehicle"""
        self._local_planner.set_offset(offset)

    def lane_change(self, direction, same_lane_time=0, other_lane_time=0, lane_change_time=2):
        """
        Changes the path so that the vehicle performs a lane change.
        Use 'direction' to specify either a 'left' or 'right' lane change,
        and the other 3 fine tune the maneuver
        """
        speed = self._vehicle.get_velocity().length()
        path = self._generate_lane_change_path(
            self._map.get_waypoint(self._vehicle.get_location()),
            direction,
            same_lane_time * speed,
            other_lane_time * speed,
            lane_change_time * speed,
            False,
            1,
            self._sampling_resolution
        )
        if not path:
            print("WARNING: Ignoring the lane change as no path was found")

        self.set_global_plan(path)

    def _affected_by_traffic_light(self, lights_list=None, max_distance=None):
        """
        Method to check if there is a red light affecting the vehicle.

            :param lights_list (list of carla.TrafficLight): list containing TrafficLight objects.
                If None, all traffic lights in the scene are used
            :param max_distance (float): max distance for traffic lights to be considered relevant.
                If None, the base threshold value is used
        """
        if self._ignore_traffic_lights:
            return (False, None)

        if not lights_list:
            lights_list = self._world.get_actors().filter("*traffic_light*")

        if not max_distance:
            max_distance = self._base_tlight_threshold

        if self._last_traffic_light:
            if self._last_traffic_light.state != carla.TrafficLightState.Red:
                self._last_traffic_light = None
            else:
                return (True, self._last_traffic_light)

        ego_vehicle_location = self._vehicle.get_location()
        ego_vehicle_waypoint = self._map.get_waypoint(ego_vehicle_location)

        for traffic_light in lights_list:
            if traffic_light.id in self._lights_map:
                trigger_wp = self._lights_map[traffic_light.id]
            else:
                trigger_location = get_trafficlight_trigger_location(traffic_light)
                trigger_wp = self._map.get_waypoint(trigger_location)
                self._lights_map[traffic_light.id] = trigger_wp

            if trigger_wp.transform.location.distance(ego_vehicle_location) > max_distance:
                continue

            if trigger_wp.road_id != ego_vehicle_waypoint.road_id:
                continue

            ve_dir = ego_vehicle_waypoint.transform.get_forward_vector()
            wp_dir = trigger_wp.transform.get_forward_vector()
            dot_ve_wp = ve_dir.x * wp_dir.x + ve_dir.y * wp_dir.y + ve_dir.z * wp_dir.z

            if dot_ve_wp < 0:
                continue

            if traffic_light.state != carla.TrafficLightState.Red:
                continue

            if is_within_distance(trigger_wp.transform, self._vehicle.get_transform(), max_distance, [0, 90]):
                self._last_traffic_light = traffic_light
                return (True, traffic_light)

        return (False, None)

    def _generate_lane_change_path(self, waypoint, direction='left', distance_same_lane=10,
                                distance_other_lane=25, lane_change_distance=25,
                                check=True, lane_changes=1, step_distance=2):
        """
        This methods generates a path that results in a lane change.
        Use the different distances to fine-tune the maneuver.
        If the lane change is impossible, the returned path will be empty.
        """

        print("in make lane change: ", waypoint.transform.location, direction)
        distance_same_lane = max(distance_same_lane, 0.1)
        distance_other_lane = max(distance_other_lane, 0.1)
        lane_change_distance = max(lane_change_distance, 0.1)

        plan = []
        plan.append((waypoint, RoadOption.LANEFOLLOW))  # start position

        option = RoadOption.LANEFOLLOW

        # Same lane
        distance = 0
        while distance < distance_same_lane:
            next_wps = plan[-1][0].next(step_distance)
            if not next_wps:
                return []
            next_wp = next_wps[0]
            distance += next_wp.transform.location.distance(plan[-1][0].transform.location)
            plan.append((next_wp, RoadOption.LANEFOLLOW))

        if direction == 'left':
            option = RoadOption.CHANGELANELEFT
        elif direction == 'right':
            option = RoadOption.CHANGELANERIGHT
        else:
            # ERROR, input value for change must be 'left' or 'right'
            return []

        lane_changes_done = 0
        lane_change_distance = lane_change_distance / lane_changes

        # Lane change
        while lane_changes_done < lane_changes:

            # Move forward
            next_wps = plan[-1][0].next(lane_change_distance)
            if not next_wps:
                return []
            next_wp = next_wps[0]

            # Get the side lane
            if direction == 'left':
                if check and str(next_wp.lane_change) not in ['Left', 'Both']:
                    return []
                side_wp = next_wp.get_left_lane()
            else:
                if check and str(next_wp.lane_change) not in ['Right', 'Both']:
                    return []
                side_wp = next_wp.get_right_lane()

            if not side_wp or side_wp.lane_type != carla.LaneType.Driving:
                return []

            # Update the plan
            plan.append((side_wp, option))
            lane_changes_done += 1

        # Other lane
        distance = 0
        while distance < distance_other_lane:
            next_wps = plan[-1][0].next(step_distance)
            if not next_wps:
                return []
            next_wp = next_wps[0]
            distance += next_wp.transform.location.distance(plan[-1][0].transform.location)
            plan.append((next_wp, RoadOption.LANEFOLLOW))

        return plan


