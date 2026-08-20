import sys

sys.path.insert(0, "/home/ubuntu/persistent/CARLA_LATEST/PythonAPI/carla")

# Use old basic agent
from agents.navigation.basic_agent import BasicAgent
from agents.navigation.controller import VehiclePIDController
from agents.tools.misc import get_speed
# from dlite import DStarLite
from queue import Queue
from collections import deque
import carla
import random
import threading
import time
from dLite.dlite import ThreadedDStarLite, DStarLite
from agents.navigation.Generate_map import gen_map_initial

mp_debug = False
regular_updates = False
position_update_frequency = 5

class DPP_Controller():
    def __init__(self, vehicle: carla.Vehicle, destination: carla.Location, waypoints):
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

        self._agent = BasicAgentD(self._vehicle)

        self._done = False

    def done(self):
        return self._done

    def start(self, log_status=True):
        """
        Initalize threads and run agent.
        """
        self._agent.init_controller()
        self._agent._search = self._search

        print("controller: initalizing threads")

        mp_thread = threading.Thread(target=self.motion_planner)

        print("controller: starting threads")

        mp_thread.start()
        self.path_search()

    def motion_planner(self):
        i = 0

        print("mp: motion planner started")

        while True:

            if not self._vehicle.is_alive:
                break
            elif self._agent.done():
                self._done = True
                break
            else:
                control_signal = self._agent.run_step()
                self._vehicle.apply_control(control_signal)

            i += 1

        print("mp: motion planner terminated")

    def path_search(self):
        self._search.start()

class BasicAgentD(BasicAgent):
    def __init__(self, vehicle, target_speed=20, opt_dict={}, map_inst=None, grp_inst=None):
        super().__init__(vehicle, target_speed, opt_dict, map_inst, grp_inst)

        self._dt = 1.0 / 20.0
        self._target_speed = 20.0
        self._sampling_radius = 2.0
        self._args_lateral_dict = {'K_P': 1.95, 'K_I': 0.05, 'K_D': 0.2, 'dt': self._dt}
        self._args_longitudinal_dict = {'K_P': 1.0, 'K_I': 0.05, 'K_D': 0, 'dt': self._dt}
        self._max_throt = 0.75
        self._max_brake = 0.3
        self._max_steer = 0.8
        self._offset = 0
        self._base_min_distance = 3.0
        self._distance_ratio = 0.5
        self._follow_speed_limits = False

        self._vehicle_controller = None
        self._q_max = 10
        self._queue = deque()
        self._search = None

    def init_controller(self):
        self._vehicle_controller = VehiclePIDController(self._vehicle,
                                                        args_lateral=self._args_lateral_dict,
                                                        args_longitudinal=self._args_longitudinal_dict,
                                                        offset=self._offset,
                                                        max_throttle=self._max_throt,
                                                        max_brake=self._max_brake,
                                                        max_steering=self._max_steer)

    def run_step(self):
        """
        Modified run_step from Basic Agent which feeds waypoints from D* 
        directly into Vehicle Controller.

        Executes one step of navigation.
        """
        self._tick_seen_obstacles()

        hazard_obstacle = False
        hazard_light = False

        vehicle_list = self._world.get_actors().filter("*vehicle*")

        vehicle_speed = get_speed(self._vehicle) / 5

        #if len(self._queue) == 0:
        if len(self._queue) < self._q_max:
            self._get_next_waypoints()
            if len(self._queue) == 0:
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
        blocked_wps = set()
        obstacle_min_berth = self._vehicle.bounding_box.extent.x * 4

        snapshot = self._get_seen_obstacles_snapshot()

        for obs_data in snapshot:
            actor = obs_data['actor']
            if not actor.is_alive:
                continue

            obs_mesh = obs_data['mesh']
            obs_wpt = self._map.get_waypoint(obs_mesh.center.location)
            for plan_wp in plan_waypoints:
                if isinstance(plan_wp, carla.libcarla.Waypoint):
                    plan_wp_location = plan_wp.transform.location
                else:
                    plan_wp_location = plan_wp

                if ego_location.distance(plan_wp_location) > min_vehicle_distance:
                    continue

                #if obs_mesh.contains_waypoint(plan_wp):
                if obs_mesh.contains_waypoint(plan_wp) or (obs_mesh.center.location.distance(plan_wp.transform.location) < obstacle_min_berth and plan_wp.lane_id == obs_wpt.lane_id):
                    if mp_debug: print(f"mp (basic_agent): obstacle detected on plan waypoint {plan_wp.id}")
                    path_blocked_by_bbox = True
                    hazard_obstacle = True
                    blocking_candidates.append(self._map.get_waypoint(actor.get_location()))
                    blocking_candidates.append(plan_wp)
                    #break

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
            if mp_debug: print("mp (basic_agent): hazard detected")
            adjacent_lane_blocked = self._adjacent_lane_waypoints_blocked(blocking_candidates[0])

            if blocking_candidates and adjacent_lane_blocked:

                # If obstacle cannot be navigated around, stop.

                if mp_debug: print("mp (basic_agent): adjacent lane waypoint(s) blocked near obstacle. Emergency stopping.")
                control = self.add_emergency_stop(control)

            elif blocking_candidates and (self._previous_obstacle is None or set([w.id for w in blocking_candidates]) != self._previous_obstacle):
                if mp_debug: print("mp:(basic_agent): entered obstacle resolution")

                # If obstacle can be navigated around
                # 1. Send obstacles and a replan request to D*
                # 2. Clear queued waypoints.

                for candidate in blocking_candidates:
                    if mp_debug: print(f"mp (basic_agent): signalling obstacle {candidate.id}")
                    self._search.signal_obstacle(candidate.transform.location)

                self._previous_obstacle = set([w.id for w in blocking_candidates])

                self._queue.clear()

                for candidate in blocking_candidates:
                    self._world.debug.draw_string(candidate.transform.location, '*', draw_shadow=False,
                    color=carla.Color(r=255, g=0, b=0), life_time=15.0,
                    persistent_lines=True)

                control = self.add_emergency_stop(control)
            elif blocking_candidates:
                control = self.add_emergency_stop(control)

        return control

    def done(self):
         # TODO: establish termination conditions for mp
         return False

    # --- Private Methods --- #

    def _get_next_waypoints(self):
        """
        Keep getting current best route successors until the queue is full.
        """

        if self._search._needs_replan.is_set():
            return

        self._queue.clear()

        current_waypoint = self._map.get_waypoint(self._vehicle.get_location())
        start = current_waypoint
        visited = {current_waypoint.id}

        i = 0

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

if __name__ == "__main__":
    print("Entering main.")

    client = carla.Client("localhost", 9000)
    client.set_timeout(10)
    world = client.get_world()

    print("Connected to CARLA.")

    blueprint_library = world.get_blueprint_library()
    vehicle_bp = random.choice(blueprint_library.filter('vehicle.audi.a2')) #vehicle blueprint
    # spawn_points = world.get_map().get_spawn_points()
    spawn_points = world.get_map().generate_waypoints(2.0)

    point_a = spawn_points[50]
    point_b = spawn_points[100]

    vehicle = None
    # vehicle = world.spawn_actor(vehicle_bp, point_a.transform)

    try:
        vehicle = world.spawn_actor(vehicle_bp, point_a.transform)
        destination = carla.Location(point_b.transform.location)

        print("main: building Controller")
        controller = DPP_Controller(vehicle, destination, spawn_points)

        print("main: starting agent")
        controller.start()

        while vehicle.is_alive:
            current_loc = vehicle.get_location()
            print(f"main: [UPDATE] vehicle at {current_loc}, distance form goal {current_loc.distance(destination)}")
            time.sleep(position_update_frequency)

    finally:
        if vehicle is not None and vehicle.is_alive:
            print("main: destroying vehicle")
            destroyed_successfully = vehicle.destroy()
