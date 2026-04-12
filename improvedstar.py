import math
import heapq
import random
#from runpy import run_path
import numpy as np
import carla
import logging
from collections import deque

"""
logging.basicConfig(
    filename='dstar.log',
    level=logging.INFO,
    format='%(levelname)s - %(funcName)s - %(message)s - %(lifneno)d'
)
"""

class DStar:
    def __init__(self, start_wp, goal_wp, vehicle, world, resolution: float = 1.0):
        self.start          = start_wp
        self.goal           = goal_wp
        self.vehicle        = vehicle
        self.world          = world
        self.resolution     = resolution
        self.map            = world.get_map()
        self.all_obst_wps   = {}
        self.new_obst_wps   = {}
        self.edge_cost      = {}
        self.h              = {}
        self.parent         = {}
        self.tag            = {}
        self.OPEN           = []
        self.all_waypoints = {}
        self.blocked_nodes = set()   
        self.xt             = self.goal
        self.s_current      = self.start
        self.initialized = False 
        #self.obstacle = 0
        self.dynamic_obstacles = []
        self.add_obs = {}
        self.all_obs = {}

    def waypoint_location(self, waypoint):
        location = waypoint.transform.location
        return ((location.x), (location.y), (location.z), waypoint.lane_id)

    def cost(self, wp1, wp2):
        forward_key =  (self.waypoint_location(wp1), self.waypoint_location(wp2))
        reverse_key = (self.waypoint_location(wp2), self.waypoint_location(wp1))
        
        if forward_key in self.edge_cost:
            return self.edge_cost[forward_key]
        if reverse_key in self.edge_cost: 
            return self.edge_cost[reverse_key]
        return wp1.transform.location.distance(wp2.transform.location)

    def get_neighbors(self, waypoint):
        neighbors = []
        prev_wps = waypoint.previous(self.resolution)
        if prev_wps:
            neighbors.extend(prev_wps)
        
        next_wps = waypoint.next(self.resolution)  
        if next_wps:
            neighbors.extend(next_wps)
        
        # Lane changes
        if waypoint.lane_change & carla.LaneChange.Left:
            left_wp = waypoint.get_left_lane()
            if left_wp and left_wp.lane_type == carla.LaneType.Driving:
                neighbors.append(left_wp)
        
        if waypoint.lane_change & carla.LaneChange.Right:
            right_wp = waypoint.get_right_lane()
            if right_wp and right_wp.lane_type == carla.LaneType.Driving:
                neighbors.append(right_wp)
        
        if waypoint.transform.location.distance(self.start.transform.location) < 1.5:
            neighbors.append(self.start)
        
        return neighbors

    def get_kmin(self):
        return self.OPEN[0][0] if self.OPEN else float("inf")
    
    def process_state(self):
        if not self.OPEN:
            return float("inf")
        
        kOld, _, currState = heapq.heappop(self.OPEN)
        currKey = self.waypoint_location(currState)
        current_h = self.h.get(currKey, float("inf"))
        
        if kOld > current_h and self.tag.get(currKey) != "Open":
            return self.get_kmin()
        
        self.tag[currKey] = "Closed"
        
        if kOld < current_h:
            # check if any neighbor has lower raised h
            for neighbor in self.get_neighbors(currState):
                newKey = self.waypoint_location(neighbor)
                neighbor_h = self.h.get(newKey, float("inf"))
                newCost = self.cost(currState, neighbor)
                neighbor_tag = self.tag.get(newKey, "New")
                
                if (neighbor_tag != "New" and 
                    neighbor_h <= kOld and 
                    current_h > neighbor_h + newCost):
                    self.parent[currKey] = neighbor
                    self.h[currKey] = neighbor_h + newCost
                    current_h = self.h[currKey]
            
            # propagate raise to children that depend on current children
            for neighbor in self.get_neighbors(currState):
                newKey = self.waypoint_location(neighbor)
                neighbor_h = self.h.get(newKey, float("inf"))
                newCost = self.cost(currState, neighbor)
                neighbor_tag = self.tag.get(newKey, "New")
                parent_key = self.waypoint_location(self.parent[newKey]) if newKey in self.parent else None
                
                if neighbor_tag == "New":
                    self.parent[newKey] = currState
                    self.h[newKey] = current_h + newCost
                    heapq.heappush(self.OPEN, (current_h + newCost, id(neighbor), neighbor))
                    self.tag[newKey] = "Open"
                
                elif parent_key == currKey and neighbor_h != current_h + newCost:
                    # Child's cost is stale, re-open it so it raises
                    self.h[newKey] = current_h + newCost
                    heapq.heappush(self.OPEN, (neighbor_h, id(neighbor), neighbor))
                    self.tag[newKey] = "Open"
                
                elif parent_key != currKey and neighbor_h > current_h + newCost:
                    # better path found through current iter, repoen
                    heapq.heappush(self.OPEN, (current_h, id(currState), currState))
                    self.tag[currKey] = "Open"
                
                elif (parent_key != currKey and 
                      current_h > neighbor_h + self.cost(currState, neighbor) and
                      neighbor_tag == "Closed"):
                    # Neighbor could lowers current, re-open neighbor
                    heapq.heappush(self.OPEN, (neighbor_h, id(neighbor), neighbor))
                    self.tag[newKey] = "Open"
                    
        elif kOld == current_h:
            for neighbor in self.get_neighbors(currState):
                newKey = self.waypoint_location(neighbor)
                neighbor_h = self.h.get(newKey, float("inf"))
                newCost = current_h + self.cost(currState, neighbor)
                neighbor_tag = self.tag.get(newKey, "New")
                
                # Compare location keys
                parent_key = self.waypoint_location(self.parent[newKey]) if newKey in self.parent else None
                
                if (neighbor_tag == "New" or (parent_key == currKey and neighbor_h != newCost) or 
                    (parent_key != currKey and neighbor_h > newCost)):
                    self.parent[newKey] = currState
                    self.h[newKey] = newCost
                    heapq.heappush(self.OPEN, (newCost, id(neighbor), neighbor))
                    self.tag[newKey] = "Open"
                    
        else:  # kOld > current_h
            for neighbor in self.get_neighbors(currState):
                newKey = self.waypoint_location(neighbor)
                neighbor_h = self.h.get(newKey, float("inf"))
                newCost = current_h + self.cost(currState, neighbor)
                neighbor_tag = self.tag.get(newKey, "New")
                
                parent_key = self.waypoint_location(self.parent[newKey]) if newKey in self.parent else None
                
                if (neighbor_tag == "New" or 
                    (parent_key == currKey and neighbor_h != newCost)):
                    self.parent[newKey] = currState
                    self.h[newKey] = newCost
                    heapq.heappush(self.OPEN, (newCost, id(neighbor), neighbor))
                    self.tag[newKey] = "Open"
                    
                elif (parent_key != currKey and neighbor_h > newCost and neighbor_tag == "Closed"):
                    heapq.heappush(self.OPEN, (current_h, id(currState), currState))
                    self.tag[currKey] = "Open"
                    
                elif (parent_key != currKey and current_h > neighbor_h + self.cost(currState, neighbor) 
                    and neighbor_tag == "Closed"):
                    heapq.heappush(self.OPEN, (neighbor_h, id(neighbor), neighbor))
                    self.tag[newKey] = "Open"
        
        return self.get_kmin()
    
    def modify_cost(self, wp1, wp2, cost):
        edge_key = (self.waypoint_location(wp1), self.waypoint_location(wp2))
        self.edge_cost[edge_key] = cost
        
        key1 = self.waypoint_location(wp1)
        key2 = self.waypoint_location(wp2)
        
        if self.tag.get(key1) != "New":
            parent_key = self.waypoint_location(self.parent[key1]) if key1 in self.parent else None
            old_h = self.h.get(key1, float("inf"))
          
            print(f"modify_cost: key1 h={old_h:.2f}, parent_key==key2? {parent_key == key2}")
            print(f"parent_key={parent_key}")
            print(f"key2={key2}")
            
            if parent_key == key2:
                self.h[key1] = self.h.get(key2, float("inf")) + cost
                heapq.heappush(self.OPEN, (old_h, id(wp1), wp1))
                self.tag[key1] = "Open"
            else:
                heapq.heappush(self.OPEN, (old_h, id(wp1), wp1))
                self.tag[key1] = "Open"
        
        return self.get_kmin()
   
    def reconstruct_path(self, from_waypoint=None):
        if from_waypoint is None:
            from_waypoint = self.start
        
        path = []
        current = from_waypoint
        goal_key = self.waypoint_location(self.goal)
        visited = set()
        
        while self.waypoint_location(current) != goal_key:
            current_key = self.waypoint_location(current)
            
            if current_key in visited:
                print(f"cycle at {current_key}")
                return None
            visited.add(current_key)
            
            path.append(current)
            
            if current_key not in self.parent:
                print(f"no parent for {current_key}")
                return None
            
            next_wp = self.parent[current_key]
            '''
            edge_cost = self.cost(current, next_wp)
            
            if edge_cost > 10000:
                print(f"path went through obs, cost={edge_cost}")
            '''
            current = next_wp
        path.append(self.goal)
        for i in range(len(path) - 1):
            c = self.cost(path[i], path[i+1])
            if c > 10000:
                print(f"PATH GOES THROUGH OBSTACLE at index {i}, cost={c}")
        return path
    
    #because of this function, once the path gets updated with the obstacles from back to start
    #its traverses again because of the parents so it starts from the latest built obstacle
    def get_current_path(self):
        path = []
        current = self.s_current
        goal_key = self.waypoint_location(self.goal)
        visited = set() 
        while self.waypoint_location(current) != goal_key:
            current_key = self.waypoint_location(current)
            if current_key in visited:
                print("Key previously visited")
                return None
            visited.add(current_key)
            path.append(current)
            if current_key not in self.parent:
                return None
            current = self.parent[current_key]
        path.append(self.goal)
        return path
    
    def manage_obstacles(self, path, num_obstacles=None):
        if len(path) < 3:
            print("Path too short to add obstacles")
            return False
        
        num_obstacles = random.randint(1, min(3, len(path) - 2))
        obstacle_indices = random.sample(range(1, len(path) - 2), num_obstacles)
        
        print(f"Add {num_obstacles} obstacles to path")
        current_key = self.waypoint_location(self.s_current)
        
        for index in obstacle_indices:
            wp1 = path[index]
            wp2 = path[index + 1]
            wp1_key = self.waypoint_location(wp1)
            wp2_key = self.waypoint_location(wp2)
            print(f"Obstacle at index {index}: wp1 h={self.h.get(wp1_key)}, wp2 h={self.h.get(wp2_key)}")
            
            self.dynamic_obstacles.append((wp1, wp2, 999999))
            self.world.debug.draw_string(wp1.transform.location, "OBS", draw_shadow=False, 
                                        color=carla.Color(255, 0, 0), life_time=30.0)
            
            # Modify cost in both directions
            self.modify_cost(wp1, wp2, 999999)
            self.modify_cost(wp2, wp1, 999999)

            iteration = 0
            max_iterations = 50000
            h_before = self.h.get(current_key, float("inf"))
            
            while self.OPEN and iteration < max_iterations:
                kmin = self.process_state()
                iteration += 1
                
                if kmin == float("inf"):
                    print("no path exists around obstacles")
                    return False
                
                if self.tag.get(current_key) == "Closed" and iteration > 2:
                    h_after = self.h.get(current_key, float("inf"))
                    if h_after != h_before and h_after < h_before * 10:
                        print(f"Obstacle at index {index} resolved after {iteration} iterations (h: {h_before:.2f} : {h_after:.2f})")
                        h_before = h_after
                        break
            
            if iteration >= max_iterations:
                print("max iters for obstacle at index", index)
                return False
            
            # Reconstruct path with updated costs for next obstacle check
            path = self.reconstruct_path(from_waypoint=self.s_current)
            if path is None:
                print("No path after obstacle at index", index)
                return False
        
        return True
   
    def initialize_search(self):
        self.h.clear()
        self.parent.clear()
        self.tag.clear()
        self.OPEN.clear()
        self.s_current = self.start

        goal_key  = self.waypoint_location(self.goal)
        start_key = self.waypoint_location(self.start)

        self.h[goal_key]   = 0.0
        self.tag[goal_key] = "Open"
        heapq.heappush(self.OPEN, (0.0, id(self.goal), self.goal))
        val = 0.0
        while self.tag.get(start_key) != "Closed" and val != float("inf"):
            #rint(f"val before: {val}")
            val = self.process_state()
           #print(f"val after: {val}")
            #self.handle_obstacles()
        if val == float("inf"):
            print("val is inf")
            return None
        path = self.reconstruct_path()
        if path:
            self.visualize_path(path)
            return path
        else:
            print("No path found")
            return None
        
    def run(self):
        # Initial search from goal to start
        initial_path = self.initialize_search()
        if initial_path is None:
            print("No initial path found")
            return None
        
        print(f"Initial path: {len(initial_path)} waypoints")
        self.visualize_path(initial_path)
        self.s_current = self.start
        success = self.manage_obstacles(initial_path)
        if not success:
            print("Cannot replan around obstacles")
            return None
        final_path = self.reconstruct_path(from_waypoint=self.s_current)
        
        if final_path is None:
            print("No path after obstacles")
            return None
        
        print(f"Final path wps: {len(final_path)}")
        self.visualize_path(final_path)
        self.print_path(final_path)
        
        return final_path
        
    def visualize_path(self, path):
        if not path:
            return
        for i in range(len(path) - 1):
            p1 = path[i].transform.location
            p2 = path[i + 1].transform.location
            self.world.debug.draw_string(p1, "^", draw_shadow=False,
                                         color=carla.Color(0, 255, 0),
                                         life_time=15.0)
            self.world.debug.draw_string(p2, "^", draw_shadow=False,
                                         color=carla.Color(0, 255, 0),
                                         life_time=15.0)
        self.world.debug.draw_string(path[0].transform.location, "START",
                                     draw_shadow=False,
                                     color=carla.Color(0, 255, 0),
                                     life_time=15.0)
        self.world.debug.draw_string(path[-1].transform.location, "GOAL",
                                     draw_shadow=False,
                                     color=carla.Color(0, 0, 255),
                                     life_time=15.0)
        

    def print_path(self, path, tag="path"):
        if not path:
            return
        coords = [(round(wp.transform.location.x, 2), round(wp.transform.location.y, 2), round(wp.transform.location.z, 2)) for wp in path]
        print(f"({len(coords)} waypoints):\n{coords}\n")

    def move_vehicle(self, path):
        if not path:
            return
        for wp in path:
            self.vehicle.set_transform(wp.transform)
            self.world.wait_for_tick()
  
if __name__ == "__main__":
    try:
        client = carla.Client("localhost", 4000)
        client.set_timeout(10.0)
        world = client.get_world()
        carla_map = world.get_map()
        blueprint_library = world.get_blueprint_library()
        '''
        #traffic manager
        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        traffic_manager.set_synchronous_mode(True)
        
        #synchronous mode 
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)
        '''
        firetruck_bp = blueprint_library.filter('vehicle.carlamotors.firetruck')[0]
        spawn_points = carla_map.get_spawn_points()

        start_ind = 25
        end_ind = 35
        point_a = spawn_points[start_ind]
        firetruck = world.spawn_actor(firetruck_bp, point_a)
        point_b = spawn_points[end_ind]
        
        while point_b.location == point_a.location:
            point_b = random.choice(spawn_points)

        start_wp = carla_map.get_waypoint(point_a.location, project_to_road=True)
        goal_wp = carla_map.get_waypoint(point_b.location, project_to_road=True)

        print(f"Start: {start_wp.transform.location}")
        print(f"Goal: {goal_wp.transform.location}")

        world.debug.draw_string(start_wp.transform.location, "START",
                                draw_shadow=False, color=carla.Color(255, 0, 0),
                                life_time=30.0, persistent_lines=True)
        world.debug.draw_string(goal_wp.transform.location, "GOAL",
                                draw_shadow=False, color=carla.Color(255, 0, 0),
                                life_time=30.0, persistent_lines=True)

        planner = DStar(start_wp, goal_wp, firetruck, world, resolution=1.0)
        #path = planner.initialize_search()
        path = planner.run()

        if path is None:
            print("Initial planning failed")
            #sys.exit()
        print("All waypoints in initial path:")
        for index, wp in enumerate(path):
            loc = wp.transform.location
            print(f"{index:03d}: (x={loc.x}, y={loc.y}, z={loc.z})")

        if path:
            #print("Moving vehicle along the planned path")
            planner.print_path(path, "final")
            planner.visualize_path(path)
        
        if path: 
            planner.move_vehicle(path)
        firetruck.destroy()
        #planner.print_path(path)

    except Exception as exc:
        print("Error during D* planning or execution:", exc)
        if 'firetruck' in locals():
            firetruck.destroy()
