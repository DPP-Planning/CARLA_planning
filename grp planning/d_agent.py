import sys

sys.path.insert(0, "/home/ubuntu/persistent/CARLA_LATEST/PythonAPI/carla")

# Use old basic agent
from agents.navigation.basic_agent import BasicAgent
from agents.navigation.controller import VehiclePIDController
from agents.tools.misc import get_speed
from queue import Queue
from collections import deque
import carla
import random
import threading
import time
from dLite.dlite import ThreadedDStarLite, DStarLite
from agents.navigation.Generate_map import gen_map_initial

position_update_frequency = 5

class DPP_Controller():
    def __init__(self, vehicle: carla.Vehicle, destination: carla.Location, waypoints, search_algorithm="dlite"):
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

        if search_algorithm == "dlite":
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

            self._agent = BasicAgentD(self._vehicle, self._destination, self._search)

        elif search_algorithm == "astar":
            self._search = None
            self._agent = BasicAgent(self._vehicle)
            self._agent.set_destination(self._destination, None)

        elif search_algorithm == "dstar":
            self._search = None
            self._agent = None

        else:
            self._search = None
            self._agent = None


    def done(self):
        return self._agent.done()

    def start(self):
        """
        Initalize threads and run agent.
        """
        print("controller: starting threads")

        if self._search: self._search.start()
        if self._agent: self._agent.start()

class BasicAgentD(BasicAgent):
    def __init__(self, vehicle, destination, search, target_speed=20, opt_dict={}, map_inst=None, grp_inst=None, debug=False):
        super().__init__(vehicle, target_speed, opt_dict, map_inst, grp_inst)

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
        self._q_max = 10
        self._queue = deque()

        self._debug = debug

        self._blockers = dict()

        self._destination = destination
        self._search = search

    def start(self, start_thread=False):
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

        if self._debug: print("entered run_step")

        old_snap = self._get_seen_obstacles_snapshot()
        self._tick_seen_obstacles()
        new_snap = self._get_seen_obstacles_snapshot()

        old_obs_wps = [self._search._dstar._closest_generated_waypoint(self._map.get_waypoint(x['location'])) for x in old_snap]
        new_obs_wps = [self._search._dstar._closest_generated_waypoint(self._map.get_waypoint(x['location'])) for x in new_snap]

        cleared = [x for x in old_obs_wps if x not in new_obs_wps]

        if self._debug: print(f" - {len(cleared)} obstacles have left perception")

        for c in cleared:
            local_wpt = self.localize(c)

            if self._blockers.get(local_wpt.id):
                for x in self._blockers[local_wpt.id]:
                    self._search.signal_cleared(x.transform.location)

                del self._blockers[local_wpt.id]

        if len(cleared) > 0:
            self._queue.clear()

        hazard_obstacle = False
        hazard_light = False

        vehicle_list = self._world.get_actors().filter("*vehicle*")

        vehicle_speed = get_speed(self._vehicle) / 5

        if len(self._queue) <= self._q_max // 2:
        #if len(self._queue) < self._q_max:
            self._get_next_waypoints()

        if len(self._queue) == 0:
            if self._debug: print(" - could not get waypoints from GNW, returning empty control")
            control = carla.VehicleControl()
            self.add_emergency_stop(control)
            return control

        control = self._vehicle_controller.run_step(
            self._target_speed,
            self._queue.popleft()
        )

        min_vehicle_distance = 25
        ego_location = self._vehicle.get_location()
        plan_waypoints = [wp for wp in self._queue]
        path_blocked_by_bbox = False
        blocking_candidates = []

        snapshot = self._get_seen_obstacles_snapshot()

        for obs_data in snapshot:
            velocity = obs_data['velocity']

            if abs(velocity.x) >= vehicle_speed  or abs(velocity.y) >= vehicle_speed:
                continue

            actor = obs_data['actor']
            if not actor.is_alive:
                continue

            obs_mesh = obs_data['mesh']
            obs_wpt = self.localize(self._map.get_waypoint(obs_mesh.center.location))
            obs_preds = self.get_lane_predecessors(self.localize(obs_wpt), r=4)

            for plan_wp in plan_waypoints:
                if isinstance(plan_wp, carla.libcarla.Waypoint):
                    plan_wp_location = plan_wp.transform.location
                else:
                    plan_wp_location = plan_wp

                if ego_location.distance(plan_wp_location) > min_vehicle_distance:
                    continue

                if obs_mesh.contains_waypoint(plan_wp) or self.localize(plan_wp) in obs_preds:
                    if self._debug: print(f" - obstacle detected on plan waypoint {plan_wp.id}")
                    path_blocked_by_bbox = True
                    hazard_obstacle = True

                    # Append Obstacle Waypoint
                    blocking_candidates.append(obs_wpt)
                    blocking_candidates.append(self.localize(plan_wp))

                    # Append Adjacent Lane Waypoints
                    if obs_wpt.id not in self._blockers.keys():
                        self._blockers[obs_wpt.id] = [obs_wpt]

                    if not any([x.id == self.localize(plan_wp).id for x in self._blockers[obs_wpt.id]]):
                        self._blockers[obs_wpt.id].append(self.localize(plan_wp))

            if path_blocked_by_bbox:
                break

        # Check if the vehicle is affected by a red traffic light
        max_tlight_distance = self._base_tlight_threshold + self._speed_ratio * vehicle_speed
        affected_by_tlight, _ = self._affected_by_traffic_light(self._lights_list, max_tlight_distance)
        if affected_by_tlight:
            hazard_light = True

        if hazard_obstacle and hazard_light:
            control = self.add_emergency_stop(control)
        elif hazard_obstacle:
            if self._debug: print(" - hazard detected, correcting...")

            adjacent_lane_blocked = False
            for b in blocking_candidates:
                if self._adjacent_lane_waypoints_blocked(b):
                    adjacent_lane_blocked = True
                    break

            if blocking_candidates and adjacent_lane_blocked:

                # If obstacle cannot be navigated around, stop.

                if self._debug: print(" - adjacent lane waypoint(s) blocked near obstacle. Emergency stopping.")
                control = self.add_emergency_stop(control)

            elif blocking_candidates and (self._previous_obstacle is None or set([w.id for w in blocking_candidates]) != self._previous_obstacle):
                if self._debug: print(" - sending obstacles to dlite")

                # If obstacle can be navigated around
                # 1. Send obstacles and a replan request to D*
                # 2. Clear queued waypoints.

                for candidate in blocking_candidates:
                    self._search.signal_obstacle(candidate.transform.location)

                self._previous_obstacle = set([w.id for w in blocking_candidates])
                self._queue.clear()

                for candidate in blocking_candidates:
                    self._world.debug.draw_string(candidate.transform.location, 'Obstacle', draw_shadow=False,
                    color=carla.Color(r=255, g=0, b=0), life_time=5.0,
                    persistent_lines=True)

                control = self.add_emergency_stop(control)
            elif blocking_candidates:
                control = self.add_emergency_stop(control)

        return control

    def stop_vehicle(self):
        control = carla.VehicleControl()
        control.throttle = 0.0
        control.steer = 0.0
        control.brake = 1.0
        self._vehicle.apply_control(control)

    def localize(self, waypoint):
        return self._search._dstar._closest_generated_waypoint(waypoint)

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
        self._vehicle_controller = VehiclePIDController(self._vehicle,
                                                        args_lateral=self._args_lateral_dict,
                                                        args_longitudinal=self._args_longitudinal_dict,
                                                        offset=self._offset,
                                                        max_throttle=self._max_throt,
                                                        max_brake=self._max_brake,
                                                        max_steering=self._max_steer)


    def _agent_loop(self):
        print("mp: motion planner started")

        while self._vehicle.get_location().distance(self._destination) > 3.5:

            if not self._vehicle.is_alive:
                break
            else:
                control_signal = self.run_step()
                self._vehicle.apply_control(control_signal)

        self.stop_vehicle()
        self._stop_event.set()
        print("mp: motion planner terminated")

    def _get_next_waypoints(self):
        """
        Keep getting current best route successors until the queue is full.
        """

        #self._queue.clear()

        if self._search._needs_replan.is_set():
            if self._debug: print(" - exiting GWP: search is still planning.")
            return

        current_waypoint = self._map.get_waypoint(self._vehicle.get_location())
        start = current_waypoint
        visited = {current_waypoint.id}

        i = 0

        if self._debug: print(" - GWP filling queue...")
        while len(self._queue) < self._q_max:
            succ = self._search.get_best_successor(current_waypoint)

            if succ is None:
                break

            if succ.id in visited:
                break

            self._queue.append(succ)

            self._world.debug.draw_string(succ.transform.location, "*", color=carla.Color(r=0, g=255, b=0), life_time=0.0)

            visited.add(succ.id)
            current_waypoint = succ
            i += 1
        if self._debug: print(f" - GWP finished after queueing {i} waypoints")

    def _get_lane_neighbors(self, target, r=5):
        last_pred = target
        last_succ = target

        candidates = []

        for i in range(r):
            succs = self._search._dstar.successors(last_succ)
            preds = self._search._dstar.predecessors(last_pred)

            s = [x for x in succs if target.lane_id == x.lane_id][0]
            p = [x for x in preds if target.lane_id == x.lane_id][0]
            candidates.append(s)
            #candidates.append(p)

            #self._world.debug.draw_string(p.transform.location, "{ }", color=carla.Color(r=100, g=255, b=0), life_time=0.0)

            last_succ = s
            last_pred = p

        return candidates

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
