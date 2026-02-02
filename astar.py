import carla
import random
import time
from queue import PriorityQueue

def euclidean_heuristic(waypoint, end_waypoint):
    return waypoint.transform.location.distance(end_waypoint.transform.location)

def manhattan_heuristic(waypoint, end_waypoint):
    dx = abs(waypoint.transform.location.x - end_waypoint.transform.location.x)
    dy = abs(waypoint.transform.location.y - end_waypoint.transform.location.y)
    dz = abs(waypoint.transform.location.z - end_waypoint.transform.location.z)
    return dx + dy + dz

class AStarNode:
    def __init__(self, waypoint, g_cost, h_cost, parent=None):
        self.waypoint = waypoint
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.f_cost = g_cost + h_cost
        self.parent = parent

def get_legal_neighbors(waypoint):
    neighbors = []
    # Forward neighbor
    forward = waypoint.next(2.0)
    if forward:
        neighbors.extend(forward)
    
    # Legal left lane change
    if waypoint.lane_change & carla.LaneChange.Left:
        left_lane = waypoint.get_left_lane()
        if left_lane and left_lane.lane_type == carla.LaneType.Driving:
            neighbors.append(left_lane)
            neighbors.append(left_lane.next(2.0)[0])
    
    # Legal right lane change
    if waypoint.lane_change & carla.LaneChange.Right:
        right_lane = waypoint.get_right_lane()
        if right_lane and right_lane.lane_type == carla.LaneType.Driving:
            neighbors.append(right_lane)
            neighbors.append(right_lane.next(2.0)[0])
    
    return neighbors

def a_star(world, start_waypoint, end_waypoint, heuristic_func=euclidean_heuristic, max_distance=5000):
    start_node = AStarNode(start_waypoint, 0, heuristic_func(start_waypoint, end_waypoint))
    open_set = PriorityQueue()
    open_set.put((start_node.f_cost, id(start_node), start_node))
    came_from = {}
    g_score = {start_waypoint.id: 0}
    f_score = {start_waypoint.id: start_node.f_cost}
    
    while not open_set.empty():
        current_node = open_set.get()[2]
        
        # Early exit if we have reached near the goal
        if current_node.waypoint.transform.location.distance(end_waypoint.transform.location) < 2.0:
            path = []

            while current_node:
                path.append(current_node.waypoint)
                current_node = came_from.get(current_node.waypoint.id)
            return list(reversed(path))
        
        # Unlikely to happen. left here for debugging purposes
        # if g_score[current_node.waypoint.id] > max_distance:
        #     print(f"A* search stopped: exceeded max distance of {max_distance}m")
        #     return None
        
        for next_waypoint in get_legal_neighbors(current_node.waypoint):
            # Add a small cost for lane changes
            lane_change_cost = 0 if next_waypoint.lane_id != current_node.waypoint.lane_id else 0

            # tentative = g score + heuristic + lane change cost
            # This sum gives us the total cost to reach the next waypoint from the start,
            # which is the definition of the g_score for that waypoint.
            # The f_score is considered at the end of the algorithm.
            # Therefore, this is g_score and not f_score
            tentative_g_score = g_score[current_node.waypoint.id] + euclidean_heuristic(current_node.waypoint, next_waypoint) + lane_change_cost
            # If the next waypoint is already in the open set, we can skip it
            # Comparing g_score for the reason above tentative_g_score.
            if next_waypoint.id not in g_score or tentative_g_score < g_score[next_waypoint.id]:
                # Draws the possible routes A* checked
                world.debug.draw_string(next_waypoint.transform.location, '^', draw_shadow=False, color=carla.Color(r=0, g=220, b=0), life_time=25.0, persistent_lines=True)
                
                came_from[next_waypoint.id] = current_node
                
                g_score[next_waypoint.id] = tentative_g_score
                f_score[next_waypoint.id] = tentative_g_score + heuristic_func(next_waypoint, end_waypoint)
                new_node = AStarNode(next_waypoint, tentative_g_score, heuristic_func(next_waypoint, end_waypoint), current_node)
                open_set.put((f_score[next_waypoint.id], id(new_node), new_node))
                # In this implementation, the f_score is implicitly used to determine the next waypoint through the PriorityQueue.
                # Therefure, the f_score is used to determine the next waypoint through the PriorityQueue.
    print("A* search failed to find a path")
    return None

def main():
    try:
        # Connect to the CARLA server
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        # world = client.load_world('Town05') # Use this to switch towns
        # Get the world and map
        world = client.get_world()
        carla_map = world.get_map()

        # Spawn a firetruck at a random location (point A)
        blueprint_library = world.get_blueprint_library()
        firetruck_bp = blueprint_library.filter('vehicle.carlamotors.firetruck')[0]
        spawn_points = carla_map.get_spawn_points()


        # Choose a random starting location (point A)
        point_a = random.choice(spawn_points)
        point_a = spawn_points[50]
        firetruck = world.spawn_actor(firetruck_bp, point_a)

        # Choose a random destination (point B)
        point_b = random.choice(spawn_points)
        point_b = spawn_points[100]
        while point_b.location == point_a.location:
            point_b = random.choice(spawn_points)

        start_waypoint = carla_map.get_waypoint(point_a.location)
        end_waypoint = carla_map.get_waypoint(point_b.location)

        print("Firetruck starting at", point_a.location)
        print(f"Destination: {point_b.location}")


        # Manual waypoint selection
        # Uncomment below to manually test with 2 points

        # # These two points are used to test lane changes
        # point_a = carla.Location(x=-52.133560, y=-40.180298, z=0.600000)
        # point_b = carla.Location(x=-111.120361, y=72.898865, z=0.600000)

        # # Another 2 points to test if vehicle should stop nearby destination
        # point_a = carla.Location(x=-64.581863, y=-65.167366, z=0.600000)
        # point_b = carla.Location(x=-27.022133, y=69.714005, z=0.600000)

        # # Another 2 points to test if vehicle should stop nearby destination
        # # point_a = carla.Location(x=109.946968, y=-17.187952, z=0.599999)
        # # point_b = carla.Location(x=26.382587, y=-57.401386, z=0.600000)

        # # Get the waypoint closest to point_a and point_b
        # waypoint_a = carla_map.get_waypoint(point_a, project_to_road=True)
        # waypoint_b = carla_map.get_waypoint(point_b, project_to_road=True)

        # start_waypoint = waypoint_a
        # end_waypoint = waypoint_b
        # End of manual waypoint selection
        print(f"Point A wp: {start_waypoint}")
        print(f"Point B wp: {end_waypoint}")

        # Run A*
        route = a_star(world, start_waypoint, end_waypoint)

        if route is None:
            # To prevent infinite loop
            print("Failed to find a path. Try adjusting the max_distance in the a_star function.")
            firetruck.destroy()
            return

        print(f"Route found with {len(route)} waypoints")

        # Keeping for debugging purposes
        # start_time = time.time()
        # timeout = 300  # 5 minutes timeout
        # route = carla_map.generate_waypoints(2.0)
        # Draws the route the vehicle will follow (red)
        for waypoint in route:
            world.debug.draw_string(waypoint.transform.location, '^', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=25.0, persistent_lines=True)

        # Follow the route
        for i, waypoint in enumerate(route):
            # Keeping for debugging purposes
            # if time.time() - start_time > timeout:
            #     print(f"Timeout reached after {timeout} seconds")
            #     break

            firetruck.set_transform(waypoint.transform)
            if i % 10 == 0:  # Print progress every 10 waypoints
                print(f"Waypoint {i}/{len(route)}")
            time.sleep(0.05)  # Reduced delay for faster execution

        print("Firetruck has reached its destination or the route has ended!")

    finally:
        # Clean up
        firetruck.destroy()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')