import carla
import random
import time
# from queue import PriorityQueue
from PriorityQueueDLite import PriorityQueue, Priority
import sys
# print(sys.getrecursionlimit())
sys.setrecursionlimit(50000)

# sys.path.append('../')

class DStarLite:
    def __init__(self, world, start_waypoint, end_waypoint,all_waypoints,wp_pts):
        self.world = world
        self.map = world.get_map()
        self.start = start_waypoint
        self.goal = end_waypoint
        self.U = PriorityQueue()
        self.km = 0
        self.g = {}
        self.rhs = {}
        self.s_last = None
        self.s_current = None
        self.all_waypoints = all_waypoints
        self.wp_pos = wp_pts
        self.part1 = []
        print('init successfully')
        self.new_edges_and_old_costs = None
        
    def successors(self,waypoint):
        neighbors = []
        # Forward neighbor
        forward = waypoint.next(1)

        if forward:
            neighbors.extend(forward)
        
        # Legal left lane change
        if waypoint.lane_change & carla.LaneChange.Left:
            left_lane = waypoint.get_left_lane()
            if left_lane and left_lane.lane_type == carla.LaneType.Driving:
                neighbors.append(left_lane)

                # # Diagonal forward-left neighbor
                # left_forward = left_lane.next(1)
                # if left_forward:
                #     neighbors.extend(left_forward)

        # Legal right lane change
        if waypoint.lane_change & carla.LaneChange.Right:
            right_lane = waypoint.get_right_lane()
            if right_lane and right_lane.lane_type == carla.LaneType.Driving:
                neighbors.append(right_lane)

                # # Diagonal forward-right neighbor
                # right_forward = right_lane.next(1)
                # if right_forward:
                #     neighbors.extend(right_forward)
        # if distance to self.start is less than x, add it to neighbors
        if waypoint.transform.location.distance(self.goal.transform.location) < 1:
            neighbors.append(self.goal)
        # ret = []
        # for i in self.rhs.keys():
        #     print(i)
        #     # closest rhs waypoint to neighbor
        #     dist = float('inf')
        #     for neighbor in neighbors:
        #         dist = min(dist, self.rhs[i])
        #         # if neighbor is closer than rhs, add it to neighbors
        #         if self.rhs[i] < dist:
        #             ret.append(neighbor)

        # print("shdjfl",self.rhs.keys())
        
        return neighbors
    def predecessors(self, waypoint):

        neighbors = []
        # Backward neighbor
        Backward = waypoint.previous(1)
        if Backward:
            neighbors.extend(Backward)
        
        # Legal left lane change
        if waypoint.lane_change & carla.LaneChange.Left:
            left_lane = waypoint.get_left_lane()
            if left_lane and left_lane.lane_type == carla.LaneType.Driving:
                neighbors.append(left_lane)

                # # Diagonal forward-left neighbor
                # left_forward = left_lane.previous(1)
                # if left_forward:
                #     neighbors.extend(left_forward)

        # Legal right lane change
        if waypoint.lane_change & carla.LaneChange.Right:
            right_lane = waypoint.get_right_lane()
            if right_lane and right_lane.lane_type == carla.LaneType.Driving:
                neighbors.append(right_lane)

                # # Diagonal forward-right neighbor
                # right_forward = right_lane.previous(1)
                # if right_forward:
                #     neighbors.extend(right_forward)
        # if distance to self.start is less than x, add it to neighbors
        if waypoint.transform.location.distance(self.start.transform.location) < 1:
            neighbors.append(self.start)
        # draw string all neighbors
        # for neighbor in neighbors:
        #     self.world.debug.draw_string(neighbor.transform.location, 'N', draw_shadow=False, color=carla.Color(r=220, g=0, b=220), life_time=30.0, persistent_lines=True)
        return neighbors
    
    def heuristic(self, waypoint1, waypoint2):
        return waypoint1.transform.location.distance(waypoint2.transform.location)
    def heuristic_c(self, waypoint1, waypoint2):
        # will eventually change this
        return waypoint1.transform.location.distance(waypoint2.transform.location)

    def contain(self, u):
        # print(f'u {self.U.heap}')
        # print(f'u {self.U.vertices_in_heap[0]}')
        return any(item == u for item in self.U.vertices_in_heap)
    
    def wp_key(self, waypoint):
        return (waypoint.transform.location.x, waypoint.transform.location.y, waypoint.transform.location.z)

    def calculate_key(self, s):
        return Priority(
            min(self.g[s.id], self.rhs[s.id]) + self.heuristic(s, self.start) + self.km,
            min(self.g[s.id], self.rhs[s.id])
            )


    def initialize(self):
        self.U = PriorityQueue()
        self.km = 0
        # check = 0
        # for s in self.all_waypoints:
        #     self.rhs[s.id] = float('inf')
        #     self.g[s.id] = float('inf')
        self.g[self.goal.id]=float('inf')
        self.rhs[self.goal.id] = 0
        self.g[self.start.id]=float('inf')
        self.rhs[self.start.id] = float('inf')
        # print(f'self.rhs {self.rhs}')
        # self.U.put((self.calculate_key(self.goal), self.goal))
        # self.U.insert(self.goal, [self.heuristic(self.start, self.goal), 0])
        self.U.insert(self.goal,  Priority(self.heuristic(self.start, self.goal), 0))
        
        print(f'self.goal {self.goal}') # wp
        print(f'self.U {self.U.top_key()}') # Priority(g, rhs)
        print(f'self.U {self.U.heap}') # [priorityNode]
        print(f'self.U {self.U.vertices_in_heap}') # [wp]
        print(f'goal calculate_key {self.calculate_key(self.goal).k1}') 

        self.world.debug.draw_string(self.goal.transform.location, 'goal', draw_shadow=False, color=carla.Color(r=110, g=0, b=220), life_time=60.0, persistent_lines=True)
        # pred_list = self.predecessors(self.goal)
        # for i in range(len(pred_list)):
        #     self.world.debug.draw_string(pred_list[i].transform.location, f'init:{i}', draw_shadow=False, color=carla.Color(r=110, g=0, b=220), life_time=60.0, persistent_lines=True)

        # print(f'pred_list {pred_list}')
        # print(f'pred 1 {pred_list[0]}')
        # print(f'self.U {self.U.pop()}') # wp
        # print(f'self.U {self.U.top_key()}') # empty
        # print(f'self.g {self.g}') # {..., wp.id: g, ...}



    def update_vertex(self, u):
        # if self.g[u.id] != self.rhs[u.id] and not self.contain(u):
        #     self.U.insert(u, self.calculate_key(u))
        #     print('2')
        #     print(f'self.U {self.U.top_key()}') # (g, rhs)
        # elif self.g[u.id] == self.rhs[u.id] and self.contain(u):
        #     self.U.remove(u)
        #     print('3')
        #     print(f'self.U {self.U.top_key()}') # (g, rhs)

        # elif self.g[u.id] != self.rhs[u.id] and self.contain(u):
        #     self.U.update(u, self.calculate_key(u))
        #     print('1')
        #     print(f'self.U {self.U.top_key()}') # (g, rhs)

        if self.g[u.id] != self.rhs[u.id] and self.contain(u):
            # a shorter path has been found
            self.U.update(u, self.calculate_key(u))
            # print('1')
            # print(f'self.U {self.U.top_key()}') # (g, rhs)

        elif self.g[u.id] != self.rhs[u.id] and not self.contain(u):
            # if node hasn't been processed to its optimal cost yet
            self.U.insert(u, self.calculate_key(u))
            # print('2')
            # print(f'self.U {self.U.top_key()}') # (g, rhs)
            # self.world.debug.draw_string(u.transform.location, 'U', draw_shadow=False, color=carla.Color(r=110, g=0, b=220), life_time=200.0, persistent_lines=True)

        elif self.g[u.id] == self.rhs[u.id] and self.contain(u):
            # have already optimized this node but still in the open set(lookup)
            self.U.remove(u)
            # print('3')
            # print(f'self.U {self.U.top_key()}') # (g, rhs)

    def compute_shortest_path(self):
        while (self.U.top_key() < self.calculate_key(self.start)) or (self.rhs[self.start.id] > self.g[self.start.id]):
            u = self.U.top() # waypoint
            # print(f'self.U {self.U.top_key()}')
            # print(f'u: {u}')
            # self.world.debug.draw_string(u.transform.location, 'U', draw_shadow=False, color=carla.Color(r=110, g=0, b=220), life_time=200.0, persistent_lines=True)
            k_old = self.U.top_key() # (g, rhs) which is (heuristic, min(g, rhs))
            k_new = self.calculate_key(u)

            if k_old < k_new: # if the waypoint is not up to date in the open set
                # print('compute_shortest_path 1')
                self.U.update(u, k_new)
            elif self.g[u.id] > self.rhs[u.id]: # if a more optimal path is found
                # print('compute_shortest_path 2')
                # print(f'self.g {self.g[u.id]}')
                # print(f'self.rhs {self.rhs[u.id]}')
                self.g[u.id] = self.rhs[u.id]
                # print(f'self.g {self.g[u.id]}')

                self.U.remove(u)
                for s in self.predecessors(u):
                    # check if s.id is in self.rhs
                    if s.id not in self.rhs:
                        # print(f'rhs2 s.id {s.id}')
                        self.rhs[s.id] = float('inf')
                    if s.id not in self.g:
                        # print(f'g2 s.id {s.id}')
                        self.g[s.id] = float('inf')
                    if s != self.goal:
                        self.rhs[s.id] = min(self.rhs[s.id], self.heuristic_c(s, u) + self.g[u.id])
                        self.world.debug.draw_string(s.transform.location, f'{self.rhs[s.id]}', draw_shadow=False, color=carla.Color(r=220, g=0, b=00), life_time=30.0, persistent_lines=True)

                    self.update_vertex(s)
                    # self.world.debug.draw_string(s.transform.location, f'{self.g[s.id]}', draw_shadow=False, color=carla.Color(r=220, g=0, b=00), life_time=30.0, persistent_lines=True)

                # self.world.debug.draw_string(u.transform.location, 'R', draw_shadow=False, color=carla.Color(r=0, g=0, b=220), life_time=60.0, persistent_lines=True)
            else:
                # print('compute_shortest_path 3')
                self.g_old = self.g[u.id]
                self.g[u.id] = float('inf')
                pred = self.predecessors(u)
                pred.append(u)
                for s in pred:   
                    if s.id not in self.rhs:
                        print(f'rhs3 s.id {s.id}')
                        self.rhs[s.id] = float('inf')
                    if s.id not in self.g:
                        print(f'g3 s.id {s.id}')
                        self.g[s.id] = float('inf')
                    # self.world.debug.draw_string(s.transform.location, 'SSS', draw_shadow=False, color=carla.Color(r=220, g=0, b=00), life_time=30.0, persistent_lines=True)
                    # self.world.debug.draw_string(s.transform.location, str(self.g[s.id]), draw_shadow=False, color=carla.Color(r=220, g=0, b=00), life_time=30.0, persistent_lines=True)
                    if self.rhs[s.id] == self.heuristic_c(s, u) + self.g_old:
                        # print('locally consistent!')
                        if s != self.goal:#?????
                            # self.rhs[s.id] = (self.heuristic_c(s, u) + self.g_old)
                            min_s = float('inf')
                            # succ = self.sensed_map.successors(vertex=s)
                            # print('pred:', pred)
                            succ = self.successors(s)
                            for s_ in succ:
                                if s_.id not in self.rhs:
                                    self.rhs[s_.id] = float('inf')
                                if s_.id not in self.g:
                                    self.g[s_.id] = float('inf')

                                temp = self.heuristic_c(s, s_) + self.g[s_.id]
                                if min_s > temp:
                                    min_s = temp
                                # temp = min( self.heuristic_c(s, s_) + self.g[s_.id], min_s)
                            self.rhs[s.id] = min_s
                    self.update_vertex(s)
                    # self.world.debug.draw_string(s.transform.location, f'{self.g[s.id]}', draw_shadow=False, color=carla.Color(r=220, g=0, b=00), life_time=30.0, persistent_lines=True)

            # print(f'self.U {self.U.vertices_in_heap}')
            # print(f'self.rhs[self.start.id] {self.rhs[self.start.id]}')
            # print(f'STUFF1:::{self.U.top_key() < self.calculate_key(self.start)}')
            # print(f'STUFF2::: {self.rhs[self.start.id] > self.g[self.start.id]}')
        # for i in self.U.heap:
        #     print(f'U: k1={i.priority.k1}||k2={i.priority.k2}||v={i.vertex}')


        # for i in self.U.vertices_in_heap:
        #     self.world.debug.draw_string(i.transform.location, f'!{self.rhs[i.id]}', draw_shadow=False, color=carla.Color(r=110, g=0, b=220), life_time=60.0, persistent_lines=True)
        # print(f'len(self.U.vertices_in_heap) {len(self.U.vertices_in_heap)}')
        # print(f'len(self.U) {len(self.U)}')
            
    # def rescan(self):

    #     new_edges_and_old_costs = self.new_edges_and_old_costs
    #     self.new_edges_and_old_costs = None
    #     return new_edges_and_old_costs
    def main(self):
        
        self.s_last = self.start
        self.s_current = self.start
        path = [self.s_current]
        # self.initialize()
        self.compute_shortest_path()
        
        # for i in self.all_waypoints:
        #     self.world.debug.draw_string(i.transform.location, f'{self.g[i.id]}', draw_shadow=False, color=carla.Color(r=0, g=220, b=220), life_time=30.0, persistent_lines=True)
        # for i in self.U.vertices_in_heap:
        #     self.world.debug.draw_string(i.transform.location, f'{self.rhs[i.id]}', draw_shadow=False, color=carla.Color(r=0, g=220, b=220), life_time=30.0, persistent_lines=True)
        print(f'self.s_current {self.s_current}')
        while True:
            # if self.g[self.s_current.id] == float('inf'):
            if self.rhs[self.start.id] == float('inf'):
                print("There is no known path to the goal.")
                return

            # Move to the best successor
            successor = self.successors(self.s_current)
            if successor==[]:
                print("No valid successor found.")
                return
            min_s = float('inf')
            arg_min = None
            print('bef move')
            for s_ in successor:
                # print(f's_ {s_}')
                self.world.debug.draw_string(s_.transform.location, f'{s_.id}!!!!!!!!!!!!!!!', draw_shadow=False, color=carla.Color(r=220, g=220, b=0), life_time=30.0, persistent_lines=True)
                temp = self.heuristic_c(self.s_current, s_) + self.g[s_.id]
                if temp < min_s:
                    min_s = temp
                    arg_min = s_
            # Moving the car to the best successor
            self.s_current = arg_min
            print('MOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOVED')
            print(f'self.s_current {self.s_current}')
            if self.s_current.transform.location.distance(self.goal.transform.location) < 2: # see if i can make it work with 2.0
                print('ARRIVED')
                break
            self.s_start = arg_min
            print(f'start: {self.s_start}')
            path.append(self.s_start)

            # scan graph for changed costs
            # if(keyboard.is_pressed('space')):
            #     print('scanning graph...')

            # changed_edges_with_old_cost = self.rescan()
            # # if any edge costs changed
            # if changed_edges_with_old_cost:
            #     self.k_m += self.heuristic_c(self.s_last, self.s_start)
            #     self.s_last = self.s_start

            #     # for all directed edges (u,v) with changed edge costs
            #     vertices = changed_edges_with_old_cost.vertices
            #     for vertex in vertices:
            #         v = vertex.pos
            #         succ_v = vertex.edges_and_c_old
            #         for u, c_old in succ_v.items():
            #             c_new = self.c(u, v)
            #             if c_old > c_new:
            #                 if u != self.s_goal:
            #                     self.rhs[u] = min(self.rhs[u], self.c(u, v) + self.g[v])
            #             elif self.rhs[u] == c_old + self.g[v]:
            #                 if u != self.s_goal:
            #                     min_s = float('inf')
            #                     succ_u = self.sensed_map.succ(vertex=u)
            #                     for s_ in succ_u:
            #                         temp = self.c(u, s_) + self.g[s_]
            #                         if min_s > temp:
            #                             min_s = temp
            #                     self.rhs[u] = min_s
            #                 self.update_vertex(u)
            self.compute_shortest_path()
        print('Done!')
