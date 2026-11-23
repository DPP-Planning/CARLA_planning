import sys

sys.path.insert(0, "/home/ubuntu/persistent/CARLA_LATEST/PythonAPI/carla")

from agents.navigation.basic_agent import BasicAgent
from agents.navigation.controller import VehiclePIDController
from agents.navigation.Generate_map import gen_map_initial
from agents.tools.misc import get_speed

from queue import Queue
from collections import deque
import carla
import random
import threading
import time

from dLite.dlite import ThreadedDStarLite, DStarLite
from astar import ThreadedAStar

position_update_frequency = 5

class DPP_Controller():
    def __init__(self, vehicle: carla.Vehicle, destination: carla.Location, waypoints, search_module="dlite", agent_module="basic"):
        self._vehicle = vehicle
        self._destination = destination
        self._world = self._vehicle.get_world()
        self._map = self._world.get_map()

        map_data = gen_map_initial(
            self._map,
            start_waypoint=self._map.get_waypoint(self._vehicle.get_location()),
            goal_waypoint=self._map.get_waypoint(self._destination),
        )

        self._all_waypoints = map_data.all_waypoints
        self._wp_pts = map_data.wp_pts

        self._search = None
        self._agent = None

        if search_module == "dlite":
            self._search = ThreadedDStarLite(DStarLite(
                self._world,
                self._map.get_waypoint(self._vehicle.get_location()),
                self._map.get_waypoint(self._destination),
                self._all_waypoints,
                self._wp_pts,
                self._vehicle,
                waypoint_graph=map_data.waypoint_graph,
                waypoint_lookup=map_data.waypoint_lookup
            ))
        elif search_module == "astar":
            self._search = ThreadedAStar(
                self._destination,
                self._vehicle,
                self._world         
            )
        else:
            self._search = None

        if agent_module == "basic":
            self._agent = BasicAgentD(
                self._vehicle, 
                self._destination, 
                self._search
            )
        else:
            self._agent = None

    def done(self):
        return self._agent.done() or self._search.done()

    def start(self):
        """
        Initalize threads and run agent.

        Agent and Search modules must implement:
            start()
                - start the agent thread
            done()
                - check if thread is finished 
        
        Search must implement:
            get_best_successor(waypoint: carla.Waypoint) -> successor: carla.Waypoint
                - Return the next waypoint the agent should go to.
            set_obstacles(obstacle_snapshot: [obstacle_data])
                - Update obstacle environment for search.
        """

        if self._search: self._search.start()
        if self._agent: self._agent.start()

class BasicAgentD(BasicAgent):
    def __init__(self, vehicle, destination, search, target_speed=20, opt_dict={}, map_inst=None, grp_inst=None, debug=False):
        super().__init__(vehicle, target_speed, opt_dict, map_inst, grp_inst)

        self.last_wp = None

        self._agent_thread = None
        self._stop_event = threading.Event()

        self._dt = 1.0 / 20.0
        self._target_speed = 20.0
        self._sampling_radius = 2.0
        self._args_lateral_dict = {'K_P': 1.95, 'K_I': 0.05, 'K_D': 0.2, 'dt': self._dt}
        self._args_longitudinal_dict = {'K_P': 1.0, 'K_I': 0.05, 'K_D': 0, 'dt': self._dt}
        self._max_throt = 0.75
        self._max_brake = 1.0
        self._max_steer = 0.8
        self._offset = 0
        self._base_min_distance = 3.0
        self._distance_ratio = 0.5
        self._follow_speed_limits = False

        self._vehicle_controller = None
        self._q_max = 25
        self._q_min = 5
        self._queue = deque()

        self._debug = debug

        self._blockers = dict()

        self._set_route = False
        self._destination = destination
        self._search = search
        self._last_snap = None

    def start(self, start_thread=True):
        self._init_controller()
        self._stop_event.clear()
        if start_thread:
            self._agent_thread = threading.Thread(target=self._agent_loop, name="MotionPlannerThread", daemon=True)
            self._agent_thread.start()

    def done(self):
        return self._stop_event.is_set()

    def run_step(self):
        """
        Modified run_step from Basic Agent which feeds waypoints from D*
        directly into Vehicle Controller.

        Executes one step of navigation.
        """
        self._debug = False

        ext_padding = 2.0
        min_obs_distance = max(self._vehicle.bounding_box.extent.x + ext_padding, self._vehicle.bounding_box.extent.y + ext_padding)
        consumption_range = self._sampling_radius + 1.0

        if self._debug: print("entered run_step")
        if self._vehicle.get_location().distance(self._destination) < 3.5: self._stop_event.set()

        in_junction = self._map.get_waypoint(self._vehicle.get_location()).is_junction
        hazard_obstacle = False
        hazard_light = False
        vehicle_speed = get_speed(self._vehicle) / 5

        self._tick_seen_obstacles()

        if self.last_wp is None:
            self.last_wp = self._map.get_waypoint(self._vehicle.get_location())

        if len(self._queue) <= self._q_max // 2:
            self._get_next_waypoints()

        if len(self._queue) == 0:
            if self._debug: print(" - could not get waypoints from GNW, returning empty control")
            control = carla.VehicleControl()
            self.add_emergency_stop(control)
            return control

        target_wp = self._queue[0]
        target_distance = self._vehicle.get_location().distance(target_wp.transform.location)

        control = self._vehicle_controller.run_step(
            self._target_speed,
            target_wp
        )

        if target_distance < consumption_range:
            self.last_wp = target_wp
            self._queue.popleft()

        #min_vehicle_distance = 25
        #ego_location = self._vehicle.get_location()
        #path_blocked_by_bbox = False
        #blocking_candidates = []

        plan_waypoints = [wp for wp in self._queue]
        snapshot = self._get_seen_obstacles_snapshot()

        if self._last_snap is not None:
            previous_actors = [x['actor'].id for x in self._last_snap]
            new_actors = [x['actor'].id for x in snapshot]
            changed_positions = any([x['position_changed'] for x in snapshot])
            if previous_actors != new_actors or changed_positions:
                if not in_junction: self._search.set_obstacles(snapshot)
            self._last_snap = snapshot
        else:
            self._search.set_obstacles(snapshot)
            self._last_snap = snapshot

        if len(plan_waypoints) > 0:
            for obs_data in snapshot:
                velocity = obs_data['velocity']

                if abs(velocity.x) >= vehicle_speed  or abs(velocity.y) >= vehicle_speed:
                    continue

                actor = obs_data['actor']
                if not actor.is_alive:
                    continue

                obs_mesh = obs_data['mesh']
                obs_wpt = self._map.get_waypoint(obs_mesh.center.location)

                plan_waypoints.sort(key=lambda x: x.transform.location.distance(obs_wpt.transform.location))

                # if plan_waypoints[0].transform.location.distance(obs_wpt.transform.location) <= max(self._vehicle.bounding_box.extent.x, self._vehicle.bounding_box.extent.y):
                #     hazard_obstacle = True
                #     blocked_plan_wp = plan_waypoints[0]
                #     self._blockers[obs_wpt.id] = []
                #     for p in plan_waypoints: 
                #         if p.transform.location.distance(blocked_plan_wp.transform.location) <= 10:
                #             blocking_candidates.append(p)
                #             self._blockers[obs_wpt.id].append(p)

                same_lane = plan_waypoints[0].lane_id == obs_wpt.lane_id
                if plan_waypoints[0].transform.location.distance(obs_wpt.transform.location) < min_obs_distance and same_lane:
                    hazard_obstacle = True
            
            # if hazard_obstacle:
            #     self._search.set_obstacles(self._get_seen_obstacles_snapshot())

        # Check if the vehicle is affected by a red traffic light
        # max_tlight_distance = self._base_tlight_threshold + self._speed_ratio * vehicle_speed
        # affected_by_tlight, _ = self._affected_by_traffic_light(self._lights_list, max_tlight_distance)
        # if affected_by_tlight:
        #     hazard_light = True

        if hazard_obstacle or hazard_light:
            control = self.add_emergency_stop(control)

        if hazard_obstacle and not in_junction:
            self._queue.clear()

        # if hazard_obstacle and hazard_light:
        #     control = self.add_emergency_stop(control)
        # elif hazard_obstacle:
        #     if self._debug: print(" - hazard detected, correcting...")

        #     adjacent_lane_blocked = False
        #     for b in blocking_candidates:
        #         if self._adjacent_lane_waypoints_blocked(b):
        #             adjacent_lane_blocked = True
        #             break

        #     if blocking_candidates and adjacent_lane_blocked:

        #         # If obstacle cannot be navigated around, stop.

        #         if self._debug: print(" - adjacent lane waypoint(s) blocked near obstacle. Emergency stopping.")
        #         control = self.add_emergency_stop(control)

        #     elif blocking_candidates and (self._previous_obstacle is None or set([w.id for w in blocking_candidates]) != self._previous_obstacle):
        #         if self._debug: print(" - sending obstacles to dlite")

        #         # If obstacle can be navigated around
        #         # 1. Send obstacles and a replan request to D*
        #         # 2. Clear queued waypoints.

        #         for candidate in blocking_candidates:
        #             self._search.signal_obstacle(candidate.transform.location)

        #         self._previous_obstacle = set([w.id for w in blocking_candidates])
        #         self._queue.clear()

        #         for candidate in blocking_candidates:
        #             self._world.debug.draw_string(candidate.transform.location, 'Obstacle', draw_shadow=False,
        #             color=carla.Color(r=255, g=0, b=0), life_time=5.0,
        #             persistent_lines=True)

        #         control = self.add_emergency_stop(control)
        #     elif blocking_candidates:
        #         control = self.add_emergency_stop(control)

        return control

    def stop_vehicle(self):
        control = carla.VehicleControl()
        control.throttle = 0.0
        control.steer = 0.0
        control.brake = 1.0
        self._vehicle.apply_control(control)

    def get_lane_predecessors(self, waypoint, r=1):
        candidates = []
        last_pred = waypoint

        for i in range(r):
            preds = self._search._dstar.predecessors(last_pred)
            candidate = list(filter(lambda x : x.lane_id == waypoint.lane_id, preds))[0]
            candidates.append(candidate)
            last_pred = candidate

        return candidates

    # --- Private Methods --- #

    def _init_controller(self):
        """
        Initalizes the vehicle controller
        """

        self._vehicle_controller = VehiclePIDController(self._vehicle,
                                                        args_lateral=self._args_lateral_dict,
                                                        args_longitudinal=self._args_longitudinal_dict,
                                                        offset=self._offset,
                                                        max_throttle=self._max_throt,
                                                        max_brake=self._max_brake,
                                                        max_steering=self._max_steer)

    def _agent_loop(self):
        """
        Thread function for the motion planner agent.
        """

        print("[BasicAgentD] motion planner started")

        distance_from_goal = self._vehicle.get_location().distance(self._destination)

        # try:
        while not self._stop_event.is_set():

            distance_from_goal = self._vehicle.get_location().distance(self._destination)

            if not self._vehicle.is_alive:
                self._stop_event.set()
                print("[BasicAgentD] agent vehicle died unexpectedly")

            else:
                control_signal = self.run_step()
                self._vehicle.apply_control(control_signal)
            
        # except Exception as e:
        #         self._stop_event.set()
        #         print(e)
        #         print("[BasicAgentD] run_step() error:", e)

        print("[BasicAgentD] motion planner terminated")

    def _get_next_waypoints(self):
        """
        Keep getting current best route successors until the queue is full.
        """

        if self._search._needs_replan.is_set():
            return

        current_waypoint = self._queue[-1] if len(self._queue) > 0 else self.last_wp

        visited = {wp.id for wp in self._queue}
        visited.add(current_waypoint.id)

        i = 0

        while len(self._queue) < self._q_max:
            succ = self._search.get_best_successor(current_waypoint)

            if succ is None:
                break

            if succ.id in visited:
                break

            self._queue.append(succ)

            self._world.debug.draw_string(succ.transform.location, "*", color=carla.Color(r=0, g=255, b=0), life_time=1.0)

            visited.add(succ.id)
            current_waypoint = succ
            i += 1

    def _update_obstacles(self):
        """
        Wraps around tick_seen_obstacles, adds extra logic for signalling obstacles
        which left perception for D*
        """

        old_snap = self._get_seen_obstacles_snapshot()
        self._tick_seen_obstacles()
        new_snap = self._get_seen_obstacles_snapshot()

        old_obs_wps = [self._search._dstar._closest_generated_waypoint(self._map.get_waypoint(x['location'])) for x in old_snap]
        new_obs_wps = [self._search._dstar._closest_generated_waypoint(self._map.get_waypoint(x['location'])) for x in new_snap]

        cleared = [x for x in old_obs_wps if x not in new_obs_wps]

        if self._debug: print(f" - {len(cleared)} obstacles have left perception")

        for c in cleared:
            local_wpt = c

            if self._blockers.get(local_wpt.id):
                for x in self._blockers[local_wpt.id]:
                    self._search.signal_cleared(x.transform.location)

                del self._blockers[local_wpt.id]

        if len(cleared) > 0:
            self._queue.clear()

if __name__ == "__main__":
    print("Entering main.")

    client = carla.Client("localhost", 9000)
    client.set_timeout(10)
    world = client.get_world()

    print("Connected to CARLA.")

    blueprint_library = world.get_blueprint_library()
    vehicle_bp = random.choice(blueprint_library.filter('vehicle.audi.a2')) #vehicle blueprint
    spawn_points = world.get_map().generate_waypoints(1.0)

    point_a = spawn_points[50]
    point_b = spawn_points[100]

    vehicle = None

    try:
        vehicle = world.spawn_actor(vehicle_bp, point_a.transform)
        destination = carla.Location(point_b.transform.location)

        print("main: building Controller")
        controller = DPP_Controller(vehicle, destination, spawn_points)

        print("main: starting agent")
        controller.start()

        while not controller.done():
            current_loc = vehicle.get_location()
            time.sleep(position_update_frequency)

    finally:
        if vehicle is not None and vehicle.is_alive:
            print("main: destroying vehicle")
            destroyed_successfully = vehicle.destroy()
