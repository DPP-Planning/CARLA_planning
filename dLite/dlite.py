import carla
import json
import math
import random
import time

# from queue import PriorityQueue
from PriorityQueueDLite import PriorityQueue, Priority
import sys
import os
from pathlib import Path
# print(sys.getrecursionlimit())
sys.setrecursionlimit(50000)

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(PROJECT_ROOT / 'grp planning'))
sys.path.append(str(PROJECT_ROOT))

# Now import from global_route_planning.py
from global_route_planner import _localize
from Generate_map import gen_map_initial, save_waypoint_graph

import threading
from copy import deepcopy


class PerceptionState:
    """Thread-safe state shared by perception, D* Lite, and vehicle execution."""

    def __init__(self):
        self._lock = threading.RLock()
        self.seen_obstacles = {}
        self.obstacle_blocking = False
        self.path_updated = False
        self.path = {}

    def mark_obstacle(self, waypoint):
        with self._lock:
            self.seen_obstacles[waypoint.id] = waypoint
            self.obstacle_blocking = True

    def clear_obstacle_blocking(self):
        with self._lock:
            self.obstacle_blocking = False

    def set_path(self, route_waypoints):
        with self._lock:
            self.path = {
                index: waypoint.id
                for index, waypoint in enumerate(route_waypoints or [])
                if waypoint is not None
            }
            self.path_updated = True

    def consume_path_update(self):
        with self._lock:
            was_updated = self.path_updated
            self.path_updated = False
            return was_updated, dict(self.path)

    def snapshot(self):
        with self._lock:
            return {
                "seen_obstacles": sorted(self.seen_obstacles.keys()),
                "obstacle_blocking": self.obstacle_blocking,
                "path_updated": self.path_updated,
                "path": dict(self.path),
            }

class DStarLite:
    def __init__(
        self,
        world,
        start_waypoint,
        end_waypoint,
        all_waypoints,
        wp_pts,
        vehicle,
        waypoint_graph=None,
        waypoint_lookup=None,
        debug_draw_costs=False,
    ):
        self.world = world
        self.map = world.get_map()
        self.start = start_waypoint
        self.goal = end_waypoint
        self.U = PriorityQueue()
        self.km = 0
        self.g = {}
        self.rhs = {}
        self.s_last = None
        self.s_current = self.start
        self.all_waypoints = all_waypoints
        self.wp_pos = wp_pts
        self.part1 = []
        self.resolution = 1.0
        self.vehicle = vehicle
        self.debug_draw_costs = debug_draw_costs
        # self.og_rhs = {}
        # self.crnt_rhs = {}
        self.all_obst_wps = {} # dict of all obstacle waypoints ever encountered
        self.new_obst_wps = {} # dict of obstacle waypoints just found, resets after each scan
        self.new_obst = 0
        print('init successfully')
        self.new_edges_and_old_costs = None
        self.path = []
        self.actual_route = []
        self.policy_routes = []
        self.reroute_events = []
        self.obstacle_events = []
        self.waypoint_graph = waypoint_graph or {}
        self.waypoint_lookup = waypoint_lookup or {waypoint.id: waypoint for waypoint in all_waypoints}
        self.perception_state = PerceptionState()
        self._waypoint_spatial_index = self._build_waypoint_spatial_index()

    def _location_key(self, location, bucket_size=None):
        bucket_size = bucket_size or self.resolution
        return (
            math.floor(location.x / bucket_size),
            math.floor(location.y / bucket_size),
            math.floor(location.z / bucket_size),
        )

    def _build_waypoint_spatial_index(self):
        spatial_index = {}
        for waypoint in self.waypoint_lookup.values():
            location_key = self._location_key(waypoint.transform.location)
            spatial_index.setdefault(location_key, []).append(waypoint)
        return spatial_index

    def _closest_generated_waypoint(self, target_waypoint):
        if target_waypoint is None:
            return None
        if target_waypoint.id in self.waypoint_lookup:
            return self.waypoint_lookup[target_waypoint.id]

        target_location = target_waypoint.transform.location
        base_key = self._location_key(target_location)
        closest_waypoint = None
        closest_distance = float("inf")

        for search_radius in range(3):
            for dx in range(-search_radius, search_radius + 1):
                for dy in range(-search_radius, search_radius + 1):
                    for dz in range(-search_radius, search_radius + 1):
                        candidate_key = (base_key[0] + dx, base_key[1] + dy, base_key[2] + dz)
                        for candidate in self._waypoint_spatial_index.get(candidate_key, []):
                            candidate_distance = target_location.distance(candidate.transform.location)
                            if candidate_distance < closest_distance:
                                closest_waypoint = candidate
                                closest_distance = candidate_distance
            if closest_waypoint is not None:
                return closest_waypoint

        for candidate in self.waypoint_lookup.values():
            candidate_distance = target_location.distance(candidate.transform.location)
            if candidate_distance < closest_distance:
                closest_waypoint = candidate
                closest_distance = candidate_distance

        return closest_waypoint

    def _graph_neighbors(self, waypoint, direction):
        resolved_waypoint = self._closest_generated_waypoint(waypoint)
        if resolved_waypoint is None:
            return []

        graph_entry = self.waypoint_graph.get(resolved_waypoint.id)
        if graph_entry is None:
            return []

        neighbors = []
        for neighbor_id in graph_entry.get(direction, []):
            neighbor = self.waypoint_lookup.get(neighbor_id)
            if neighbor is not None:
                neighbors.append(neighbor)
        return neighbors

    def _waypoint_ids(self, waypoints, unique=False):
        waypoint_ids = []
        seen_ids = set()
        for waypoint in waypoints or []:
            if waypoint is None:
                continue
            waypoint_id = waypoint.id
            if unique and waypoint_id in seen_ids:
                continue
            seen_ids.add(waypoint_id)
            waypoint_ids.append(waypoint_id)
        return waypoint_ids

    def extract_policy_route(self, start_waypoint=None, max_steps=None):
        current = start_waypoint or self.s_current or self.start
        if current is None:
            return []

        max_steps = max_steps or max(len(self.all_waypoints) * 2, 1)
        route = [current]
        visited_ids = {current.id}

        while current.transform.location.distance(self.goal.transform.location) >= 3.5 and len(route) < max_steps:
            try:
                successor = self.successors(current)
            except Exception:
                break
            if not successor:
                break

            best = None
            best_cost = float("inf")
            for candidate in successor:
                candidate_g = self.g.get(candidate.id, float("inf"))
                candidate_cost = self.heuristic_c(current, candidate) + candidate_g
                if candidate_cost < best_cost:
                    best = candidate
                    best_cost = candidate_cost

            if best is None or best_cost == float("inf") or best.id in visited_ids:
                break

            route.append(best)
            visited_ids.add(best.id)
            current = best

        return route

    def record_policy_route(self, label, start_waypoint=None, obstacles=None, store=True):
        route = self.extract_policy_route(start_waypoint=start_waypoint)
        route_ids = self._waypoint_ids(route)
        route_record = {
            "label": label,
            "start": route_ids[0] if route_ids else None,
            "goal": self.goal.id if self.goal else None,
            "route": route_ids,
            "obstacles": sorted(obstacles or self.all_obst_wps.keys()),
        }
        if store:
            self.policy_routes.append(route_record)
        self.perception_state.set_path(route)
        return route_record

    def export_visualization_data(self, output_path="dlite_route.json", route=None):
        output_path = Path(output_path)
        if not output_path.is_absolute():
            output_path = Path(__file__).resolve().parents[1] / output_path

        actual_route = route or self.actual_route

        payload = {
            "start": self.start.id if self.start else None,
            "goal": self.goal.id if self.goal else None,
            "route": self._waypoint_ids(actual_route),
            "actual_route": self._waypoint_ids(actual_route),
            "planned_routes": self.policy_routes,
            "reroutes": self.reroute_events,
            "explored": self._waypoint_ids(self.path, unique=True),
            "obstacles": sorted(self.all_obst_wps.keys()),
            "obstacle_events": self.obstacle_events,
            "perception": self.perception_state.snapshot(),
        }

        output_path.parent.mkdir(parents=True, exist_ok=True)
        with open(output_path, "w", encoding="utf-8") as output_file:
            json.dump(payload, output_file, indent=2)

        print(f"Saved D* Lite visualization data to {output_path}")
        return output_path
        
    def successors(self,waypoint):
        neighbors = self._graph_neighbors(waypoint, "successors")
        if waypoint.transform.location.distance(self.goal.transform.location) < self.resolution + .5:
            neighbors.append(self.goal)
        return neighbors
    
    def predecessors(self, waypoint):
        neighbors = self._graph_neighbors(waypoint, "predecessors")
        if waypoint.transform.location.distance(self.start.transform.location) < self.resolution + 0.5:
            neighbors.append(self.start)
        return neighbors
    
    def add_obst_loc_to_obst_wp(self, location):
        wp = self.map.get_waypoint(location)
        gen_wp = self._closest_generated_waypoint(wp)
        if gen_wp is None:
            return None
        self.all_obst_wps[gen_wp.id] = gen_wp
        self.new_obst_wps[gen_wp.id] = gen_wp
        self.perception_state.mark_obstacle(gen_wp)
        return gen_wp

    
    def heuristic(self, waypoint1, waypoint2):
        return waypoint1.transform.location.distance(waypoint2.transform.location)
    
    def heuristic_c(self, waypoint1, waypoint2):
        if waypoint1.id in self.all_obst_wps or waypoint2.id in self.all_obst_wps:
            return float('inf')
        else:
            return waypoint1.transform.location.distance(waypoint2.transform.location)        
        # if waypoint1.id == 1412984381793799066:
        #     print("\n\n\nthe target of id", waypoint1)

        # # if waypoint1.id == 1412984381793799066 or waypoint2.id == 1412984381793799066:
        # #     return float('inf')
        # # else:
        # #     return waypoint1.transform.location.distance(waypoint2.transform.location)
        
    def contain(self, u):
        return any(item == u for item in self.U.vertices_in_heap)
    
    def wp_key(self, waypoint):
        return (waypoint.transform.location.x, waypoint.transform.location.y, waypoint.transform.location.z)

    def calculate_key(self, s):
        return Priority(
            min(self.g[s.id], self.rhs[s.id]) + self.heuristic(s, self.s_current) + self.km,
            min(self.g[s.id], self.rhs[s.id])
            )

    def initialize(self):
        self.U = PriorityQueue()
        self.km = 0
        for s in self.all_waypoints:
            self.rhs[s.id] = float('inf')
            self.g[s.id] = float('inf')
        self.g[self.goal.id]=float('inf')
        self.rhs[self.goal.id] = 0
        self.g[self.start.id]=float('inf')
        self.rhs[self.start.id] = float('inf')

        self.U.insert(self.goal,  Priority(self.heuristic(self.start, self.goal), 0))
        
        print(f'self.goal {self.goal}') # wp
        print(f'self.U {self.U.top_key()}') # Priority(g, rhs)
        print(f'self.U {self.U.heap}') # [priorityNode]
        print(f'self.U {self.U.vertices_in_heap}') # [wp]
        print(f'goal calculate_key {self.calculate_key(self.goal).k1}') 

        self.world.debug.draw_string(self.goal.transform.location, 'goal', draw_shadow=False, color=carla.Color(r=110, g=0, b=220), life_time=60.0, persistent_lines=True)
        pred_list = self.predecessors(self.goal)

        print(f'pred_list {pred_list}')
        print(f'pred 1 {pred_list[0]}')

    def update_vertex(self, u):
        if self.g[u.id] != self.rhs[u.id] and self.contain(u):
            self.U.update(u, self.calculate_key(u))
        elif self.g[u.id] != self.rhs[u.id] and not self.contain(u):
            self.U.insert(u, self.calculate_key(u))
        elif self.g[u.id] == self.rhs[u.id] and self.contain(u):
            self.U.remove(u)

    def compute_shortest_path(self):
        while (self.U.top_key() < self.calculate_key(self.start)) or (self.rhs[self.start.id] > self.g[self.start.id]):
            u = self.U.top()

            k_old = self.U.top_key()
            k_new = self.calculate_key(u)

            if k_old < k_new:
                self.U.update(u, k_new)
            elif self.g[u.id] > self.rhs[u.id]:
                self.g[u.id] = self.rhs[u.id]
                self.U.remove(u)
                for s in self.predecessors(u):
                    self.path.append(s)
                    if s != self.goal:  
                        self.rhs[s.id] = min(self.rhs[s.id], self.heuristic_c(s, u) + self.g[u.id])
                    self.update_vertex(s)
            else:
                self.g_old = self.g[u.id]
                self.g[u.id] = float('inf')
                pred = self.predecessors(u)
                pred.append(u)
                for s in pred:   
                    self.path.append(s)
                    if self.rhs[s.id] == self.heuristic_c(s, u) + self.g_old:
                        print('locally consistent!')
                        if s != self.goal:
                            min_s = float('inf')
                            print('pred:', pred)
                            succ = self.successors(s)
                            for s_ in succ:
                                temp = self.heuristic_c(s, s_) + self.g[s_.id]
                                if min_s > temp:
                                    min_s = temp
                            self.rhs[s.id] = min_s
                    self.update_vertex(s)

    def obs_signal(self,location):
        self.new_obst+=1
        self.add_obst_loc_to_obst_wp(location)

    def rescan(self):
        if self.new_obst==0:
            return False
        else:
            self.new_obst=0
            return True

    def main(self):
        self.s_last = self.start
        self.s_current = self.start
        path = [self.s_current]
        self.actual_route = path
        self.perception_state.set_path(path)
        flag=0
        demo_obstacle_indices = [2812, 2816, 2820, 2824]
        has_demo_obstacles = len(self.all_waypoints) > max(demo_obstacle_indices)
        self.compute_shortest_path()
        self.record_policy_route("initial", start_waypoint=self.s_current, obstacles=[])
        
        print(f'self.s_current before while{self.s_current}')
        while self.s_current.transform.location.distance(self.goal.transform.location) >= 3.5:
            if self.rhs[self.start.id] == float('inf'):
                print("There is no known path to the goal.")
                return None

            # Move to the best successor
            successor = self.successors(self.s_current)
            if not successor:
                print("No valid successor found.")
                return None
            min_s = float('inf')
            arg_min = None
            for s_ in successor:
                temp = self.heuristic_c(self.s_current, s_) + self.g[s_.id]
                print(temp)
                if temp<= min_s:
                    min_s = temp
                    arg_min = s_

            self.s_current = arg_min
            path.append(self.s_current)
            self.actual_route = path
            self.perception_state.set_path(path)
            self.vehicle.set_transform(self.s_current.transform)

            if has_demo_obstacles and self.s_current.transform.location.distance(self.all_waypoints[2824].transform.location)<25 and flag==0:
                flag=1
                for obstacle_index in demo_obstacle_indices:
                    self.obs_signal(self.all_waypoints[obstacle_index].transform.location)

                print("\n\n\nadded obstacle")

            if has_demo_obstacles and self.debug_draw_costs and self.s_current.transform.location.distance(self.all_waypoints[2824].transform.location)<15 and flag==1:
                flag=2
                for i in self.all_waypoints:
                    self.world.debug.draw_string(i.transform.location, f'{self.g[i.id]}', draw_shadow=False, color=carla.Color(r=0, g=220, b=220), life_time=20.0, persistent_lines=True)

            time.sleep(.01)
            if has_demo_obstacles and self.s_current.transform.location.distance(self.all_waypoints[2824].transform.location)<5:
                self.world.debug.draw_string(self.s_current.transform.location, f'{self.rhs[self.s_current.id]}', draw_shadow=False, color=carla.Color(r=220, g=200, b=200), life_time=30.0, persistent_lines=True)
                time.sleep(2)

            if self.s_current.transform.location.distance(self.goal.transform.location) < 3.5: # see if i can make it work with 2.0
                print('ARRIVED')
                return path

            if self.rescan():
                new_obstacle_ids = sorted(self.new_obst_wps.keys())
                self.obstacle_events.append(
                    {
                        "step": len(path) - 1,
                        "current": self.s_current.id,
                        "obstacles": new_obstacle_ids,
                    }
                )
                self.km += self.heuristic_c(self.s_last, self.start)
                self.s_last = self.start

                for vertex in self.new_obst_wps.values():
                    for u in self.predecessors(vertex):
                        c_old = self.heuristic(u,vertex)
                        self.rhs[vertex.id] = float('inf')

                        if(c_old>self.heuristic_c(u,vertex)):
                            if(not (u.transform.location.distance(self.goal.transform.location) < 3.5)):# u!=self.goal
                                self.rhs[u.id] = min(self.rhs[u.id], self.heuristic_c(u, vertex) + self.g[vertex.id])
                                print('\n\n\n\n\n\n\nworked1')

                        elif(self.rhs[u.id]==c_old+self.g[vertex.id]):
                            if(u.transform.location.distance(self.goal.transform.location) >= 3.5):
                                min_s = float('inf')
                                arg_min = None
                                for s_ in self.successors(u):
                                    temp = self.heuristic_c(u, s_) + self.g[s_.id]
                                    if temp < min_s:
                                        min_s = temp
                                        arg_min = s_
                                self.rhs[u.id] = min_s
                                print('\n\n\n\n\n\n\nworked2')
                        self.update_vertex(u)
                self.new_obst_wps = {}
                self.compute_shortest_path()
                self.perception_state.clear_obstacle_blocking()
                reroute_record = self.record_policy_route(
                    f"reroute {len(self.reroute_events) + 1}",
                    start_waypoint=self.s_current,
                    obstacles=new_obstacle_ids,
                    store=False,
                )
                reroute_record["step"] = len(path) - 1
                reroute_record["current"] = self.s_current.id
                reroute_record["trigger_obstacles"] = new_obstacle_ids
                self.reroute_events.append(reroute_record)
        print(f"{self.all_obst_wps}")
        debug_waypoint_id = 1412984381793799066
        if debug_waypoint_id in self.g:
            print(f'{self.g[debug_waypoint_id]}')
        print('Done!')
        return path

class ThreadedDStarLite:
    def __init__(self, dstar: DStarLite, debug_draw_in_thread=False, replan_interval=0.2):
        self._dstar = dstar
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._planner_thread = None
        self._planner_idle = threading.Event()
        self._planner_idle.set()  # initially idle until run
        self._needs_replan = threading.Event()
        self._waypoint_index = {}   # mapping: wp_id -> {'g', 'rhs', 'predecessors', 'successors', 'waypoint'}
        self.replan_interval = replan_interval
        self.debug_draw_in_thread = debug_draw_in_thread
        # self._avoid_main_runner_warning = (
        #     "Threaded usage will NOT call DStarLite.main() automatically. "
        #     "Use get_best_successor() from main thread to move the vehicle."
        # )

    # --- Public API -----------------------------------------------------

    def start(self):
        with self._lock:
            if not self._dstar.g or not self._dstar.rhs:
                try:
                    self._dstar.initialize()
                except Exception as e:
                    print("[ThreadedDStarLite] dstar.initialize() raised:", e)

            self._build_waypoint_index()  # populate initial snapshot

        self._stop_event.clear()
        self._needs_replan.set()
        self._planner_thread = threading.Thread(target=self._planner_loop, name="DStarPlannerThread", daemon=True)
        self._planner_thread.start()

    def stop(self, join_timeout=1.0):
        self._stop_event.set()
        if self._planner_thread:
            self._planner_thread.join(timeout=join_timeout)

    def signal_obstacle(self, location):
        with self._lock:
            try:
                obstacle_waypoint = self._dstar.add_obst_loc_to_obst_wp(location)
                if obstacle_waypoint is not None:
                    self._needs_replan.set()
            except Exception as e:
                print("[ThreadedDStarLite] signal_obstacle() error:", e)

    def request_replan(self):
        self._needs_replan.set()

    def get_waypoint_snapshot(self):
        with self._lock:
            return deepcopy(self._waypoint_index)

    def get_best_successor(self, current_waypoint, prefer_lowest_g=True):
        if current_waypoint is None:
            return None

        with self._lock:
            try:
                succs = self._dstar.successors(current_waypoint)
            except Exception as e:
                succs = []
            best = None
            best_cost = float('inf')
            for s in succs:
                sid = s.id
                entry = self._waypoint_index.get(sid)
                if entry is None:
                    continue
                g_s = entry['g']
                try:
                    h = self._dstar.heuristic_c(current_waypoint, s)
                except Exception:
                    h = self._dstar.heuristic(current_waypoint, s)
                cost = h + g_s
                if cost < best_cost:
                    best_cost = cost
                    best = s
            return best

    # --- Internal planner thread ---------------------------------------

    def _planner_loop(self):
        """Background loop that recomputes compute_shortest_path when needed and updates snapshot."""
        print("[ThreadedDStarLite] Planner thread started.")
        while not self._stop_event.is_set():
            if not self._needs_replan.is_set():
                time.sleep(self.replan_interval)
                continue
            if self._needs_replan.is_set():
                self._needs_replan.clear()
                self._planner_idle.clear()
                try:
                    self._dstar.compute_shortest_path()
                    trigger_obstacles = sorted(self._dstar.new_obst_wps.keys())
                    if trigger_obstacles:
                        self._dstar.obstacle_events.append(
                            {
                                "step": len(self._dstar.actual_route),
                                "current": self._dstar.s_current.id if self._dstar.s_current else None,
                                "obstacles": trigger_obstacles,
                            }
                        )
                        route_record = self._dstar.record_policy_route(
                            f"threaded reroute {len(self._dstar.reroute_events) + 1}",
                            start_waypoint=self._dstar.s_current,
                            obstacles=trigger_obstacles,
                            store=False,
                        )
                        route_record["current"] = self._dstar.s_current.id if self._dstar.s_current else None
                        route_record["trigger_obstacles"] = trigger_obstacles
                        self._dstar.reroute_events.append(route_record)
                    elif not self._dstar.policy_routes:
                        self._dstar.record_policy_route("threaded initial", start_waypoint=self._dstar.s_current)
                    self._dstar.new_obst_wps = {}
                    self._dstar.new_obst = 0
                    self._dstar.perception_state.clear_obstacle_blocking()
                except Exception as e:
                    print("[ThreadedDStarLite] compute_shortest_path() error:", e)
                with self._lock:
                    try:
                        self._build_waypoint_index()
                    except Exception as e:
                        print("[ThreadedDStarLite] _build_waypoint_index() error:", e)
                self._planner_idle.set()
        print("[ThreadedDStarLite] Planner thread exiting.")

    def wait_until_idle(self, timeout=None):
        return self._planner_idle.wait(timeout=timeout)

    # --- Snapshot builder ---------------------------------------------

    def _build_waypoint_index(self):
        wp_index = {}
        all_wps = getattr(self._dstar, "all_waypoints", None)
        if not all_wps:
            all_wps = getattr(self._dstar, "all_waypoints", [])
        for wp in all_wps:
            wid = wp.id
            g_val = self._dstar.g.get(wid, float('inf'))
            rhs_val = self._dstar.rhs.get(wid, float('inf'))
            try:
                preds = self._dstar.predecessors(wp)
            except Exception:
                preds = []
            try:
                succs = self._dstar.successors(wp)
            except Exception:
                succs = []
            pred_ids = list({p.id for p in preds if p is not None})
            succ_ids = list({s.id for s in succs if s is not None})
            wp_index[wid] = {
                'g': float(g_val),
                'rhs': float(rhs_val),
                'predecessors': pred_ids,
                'successors': succ_ids,
                'waypoint': wp
            }
        self._waypoint_index = wp_index

    # --- Convenience debug helpers ------------------------------------

    def dump_topk(self, k=10):
        """Return top-k nodes by smallest g value (for quick debugging)."""
        with self._lock:
            items = sorted(self._waypoint_index.items(), key=lambda kv: kv[1].get('g', float('inf')))
            return items[:k]


class PlanningCoordinator:
    """Owns Basic Agent, D* Lite, and perception-style shared state.

    The image notes call for a superior class that initializes both runtime
    pieces and protects shared variables with mutexes. This class provides that
    handoff point without forcing the Basic Agent implementation to live here.
    """

    def __init__(self, dstar_lite, basic_agent=None, replan_interval=0.2):
        self.dstar_lite = dstar_lite
        self.basic_agent = basic_agent
        self.shared_state = dstar_lite.perception_state
        self.threaded_dstar = ThreadedDStarLite(
            dstar_lite,
            replan_interval=replan_interval,
        )
        self._basic_agent_thread = None
        self._stop_event = threading.Event()

    def start(self, run_basic_agent=None):
        self._stop_event.clear()
        self.threaded_dstar.start()
        if run_basic_agent is not None:
            self._basic_agent_thread = threading.Thread(
                target=run_basic_agent,
                name="BasicAgentThread",
                daemon=True,
            )
            self._basic_agent_thread.start()

    def stop(self):
        self._stop_event.set()
        self.threaded_dstar.stop()
        if self._basic_agent_thread:
            self._basic_agent_thread.join(timeout=1.0)

    def report_obstacle(self, location):
        self.threaded_dstar.signal_obstacle(location)

    def get_path_update(self):
        return self.shared_state.consume_path_update()

    def get_shared_state_snapshot(self):
        return self.shared_state.snapshot()

def main():
    # Connect to the CARLA server
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    # Get the world and map
    world = client.get_world()
    carla_map = world.get_map()

    # Spawn a firetruck at a random location (point A)
    blueprint_library = world.get_blueprint_library()
    # firetruck_bp = blueprint_library.filter('vehicle.carlamotors.firetruck')[0]
    firetruck_bp = blueprint_library.filter('vehicle.harley-davidson.low_rider')[0]

    spawn_points = carla_map.get_spawn_points()

    point_a = spawn_points[50]
    print("point_a:", point_a)
    firetruck = world.spawn_actor(firetruck_bp, point_a)
    point_b = spawn_points[5]

    start_waypoint = carla_map.get_waypoint(point_a.location)
    end_waypoint = carla_map.get_waypoint(point_b.location)
    world.debug.draw_string(start_waypoint.transform.location, 'START', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
    world.debug.draw_string(end_waypoint.transform.location, 'END', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
    print("Firetruck starting at", point_a.location)
    print(f"Destination: {point_b.location}")

    map_data = gen_map_initial(
        carla_map,
        start_waypoint=start_waypoint,
        goal_waypoint=end_waypoint,
    )
    save_waypoint_graph(map_data.waypoint_graph, "map_cache.txt")

    all_waypoints = map_data.all_waypoints
    wp_pts = map_data.wp_pts
    get_start = map_data.start_waypoint
    get_end = map_data.goal_waypoint

    print(f'gen_points {all_waypoints[0]}')
    print(f'all_waypoints {all_waypoints[0]}')
    # world.debug.draw_string(all_waypoints[0].transform.location, '^', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
    print(f'get_start {get_start}')
    print(f'get_end {get_end}')
    world.debug.draw_string(get_start.transform.location, 'S', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
    world.debug.draw_string(get_end.transform.location, 'E', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
    print('============================================================')
    try:
        dstar_lite = DStarLite(
            world,
            get_start,
            get_end,
            all_waypoints,
            wp_pts,
            firetruck,
            waypoint_graph=map_data.waypoint_graph,
            waypoint_lookup=map_data.waypoint_lookup,
        )
        dstar_lite.initialize()
        route = dstar_lite.main()
        dstar_lite.export_visualization_data(route=route)

    finally:
         # Clean up
        firetruck.destroy()
        print('Firetruck destroyed successfully')


if __name__ == "__main__":
    main()
