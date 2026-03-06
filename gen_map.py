import carla

# Connect to the CARLA server
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

# Get the world and map
world = client.get_world()
carla_map = world.get_map()

'''
for every wp, 
do previous and find the wps that are closest to the previous wps.
Repeat for next
assign g and rhs to be inf

'''

class GenMap:
    def __init__(self, world, start_waypoint, end_waypoint,all_waypoints):
        self.world = world
        self.map = world.get_map()
        self.start = start_waypoint
        self.goal = end_waypoint
        self.km = 0
        self.g = {}
        self.rhs = {}
        self.s_last = None
        self.s_current = self.start
        self.all_waypoints = all_waypoints
        self.part1 = []
        self.resolution = 1.0
        # self.og_rhs = {}
        # self.crnt_rhs = {}
        self.all_obst_wps = {} # dict of all obstacle waypoints ever encountered
        self.new_obst_wps = {} # dict of obstacle waypoints just found, resets after each scan
        self.new_obst = 0
        print('init successfully')
        self.new_edges_and_old_costs = None
        self.path = []

    def predecessors(self, waypoint):
        
        neighbors = []
        # Backward neighbor
        Backward = waypoint.previous(self.resolution)
        if Backward:
            neighbors.extend(Backward)
        
        # Legal left lane change
        if waypoint.lane_change & carla.LaneChange.Left:
            left_lane = waypoint.get_left_lane()
            if left_lane and left_lane.lane_type == carla.LaneType.Driving:
                neighbors.append(left_lane)
                # self.world.debug.draw_string(left_lane.transform.location, 'L', draw_shadow=False, color=carla.Color(r=220, g=0, b=220), life_time=30.0, persistent_lines=True)
                neighbors.append(left_lane.previous(self.resolution)[0])
                
        # Legal right lane change
        if waypoint.lane_change & carla.LaneChange.Right:
            right_lane = waypoint.get_right_lane()
            if right_lane and right_lane.lane_type == carla.LaneType.Driving:
                neighbors.append(right_lane)
                neighbors.append(right_lane.previous(self.resolution)[0])

                # self.world.debug.draw_string(right_lane.transform.location, 'R', draw_shadow=False, color=carla.Color(r=220, g=0, b=220), life_time=30.0, persistent_lines=True)
        # print(neighbors)
        test = self._localize(waypoint.transform.location)
        self.world.debug.draw_string(test.transform.location, 'SSSSSSSSSS', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
        x = [0]*len(neighbors)
        for i in range(len(neighbors)):
            # initial_dist = 0.2
            initial_dist = neighbors[i].transform.location.distance(waypoint.transform.location)
            # if self.g.get(neighbors[i].id) is None:
            for z in self.all_waypoints:
                if neighbors[i].transform.location.distance(z.transform.location) < initial_dist:# and (waypoint.road_id == z.road_id or waypoint.lane_id == z.lane_id):
                    x[i] = z
                    initial_dist=neighbors[i].transform.location.distance(z.transform.location)
            # vvv is why i use range and not just for i in neighbors, can't assign i=x
            # neighbors[i] = x

        for i in range(len(neighbors)):
            if x[i] != 0:
                neighbors[i] = x[i]

        self.part1.extend(neighbors)

        if waypoint.transform.location.distance(self.start.transform.location) < self.resolution+0.5:
            neighbors.append(self.start)
        return neighbors
    




def main():

    # Connect to the CARLA server
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    # world = client.load_world('Town05') # Use this to switch towns
    # Get the world and map
    world = client.get_world()
    carla_map = world.get_map()

    gen_points = carla_map.generate_waypoints(1)
    new_gen_points = {}
    prev_of_new_gen_points = []
    for i in range(len(gen_points)):
        world.debug.draw_string(gen_points[i].transform.location, f'{i}', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
        new_gen_points[i] = {gen_points[i]} # add wp to dictionary
        prev_of_new_gen_points.extend(new_gen_points[i].previous(1))
        left_lane = new_gen_points[i].get_left_lane()
        print('------------------------------------------------------------')
        print(f'prev_of_new_gen_points {new_gen_points[i].previous(1)[0].transform.location}') #the list from prev only has 1 element
        print(f'left_lane {left_lane}')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
