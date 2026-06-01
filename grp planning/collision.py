import carla
from astar import a_star
import math
import random
from queue import PriorityQueue
import time
import math

class car_mesh:
	"""
	Create a class which stores each corner of a car's bounding box on a single plane
	We can take the minimum distance between the corners of two cars to find out if they're colliding.
	"""

	def __init__(self, actor):
		self.id = actor.id
		self.center = actor.get_transform()
		self.bb = actor.bounding_box
		self.corners = [ loc for i, loc in enumerate(actor.bounding_box.get_world_vertices(self.center)) if i % 2 == 0] # Only take the corners on a single plane (4 of the 8 total corners)

	def get_min_distance(self, mesh):
		min_distance = float('inf')

		for c in mesh.corners:
			for i in self.corners:
				min_distance = min(min_distance, euclidian_distance_2d(c, i))

		return min_distance

	def print_vertices(self):
		print("Actor Mesh {}".format(self.id))
		for v in self.corners:
			print(" - (X: {}, Y:{})".format(v.x, v.y))

	def contains_waypoint(self, waypoint_or_location, radius_m=0.0):
		"""Return True if waypoint/location is inside mesh footprint or within radius_m of its XY edges."""
		if waypoint_or_location is None or len(self.corners) < 3:
			return False

		if hasattr(waypoint_or_location, "transform") and hasattr(waypoint_or_location.transform, "location"):
			point = waypoint_or_location.transform.location
		elif isinstance(waypoint_or_location, carla.Location):
			point = waypoint_or_location
		else:
			return False

		inside = False
		x, y = point.x, point.y

		# Ensure corners are consistently ordered around centroid before polygon tests.
		# Using unsorted bbox vertices can create self-intersecting polygons and false negatives.
		cx = sum(v.x for v in self.corners) / len(self.corners)
		cy = sum(v.y for v in self.corners) / len(self.corners)
		vertices = sorted(self.corners, key=lambda v: math.atan2(v.y - cy, v.x - cx))
		j = len(vertices) - 1

		for i in range(len(vertices)):
			xi, yi = vertices[i].x, vertices[i].y
			xj, yj = vertices[j].x, vertices[j].y
			intersects = ((yi > y) != (yj > y)) and (
				x < (xj - xi) * (y - yi) / ((yj - yi) if (yj - yi) != 0 else 1e-12) + xi
			)
			if intersects:
				inside = not inside
			j = i

		if inside:
			return True

		if radius_m <= 0.0:
			return False

		# Outside polygon: check minimum distance to polygon edges in XY plane.
		for i in range(len(vertices)):
			a = vertices[i]
			b = vertices[(i + 1) % len(vertices)]

			ax, ay = a.x, a.y
			bx, by = b.x, b.y
			px, py = x, y

			abx = bx - ax
			aby = by - ay
			ab_len_sq = (abx * abx) + (aby * aby)

			if ab_len_sq <= 1e-12:
				dx = px - ax
				dy = py - ay
				dist = math.sqrt((dx * dx) + (dy * dy))
			else:
				t = ((px - ax) * abx + (py - ay) * aby) / ab_len_sq
				t = max(0.0, min(1.0, t))
				closest_x = ax + t * abx
				closest_y = ay + t * aby
				dx = px - closest_x
				dy = py - closest_y
				dist = math.sqrt((dx * dx) + (dy * dy))

			if dist <= radius_m:
				return True

		return False

def handle_collision(data):
	print("* Collision Detected! *")
	print(data)

def euclidian_distance_2d(loc1, loc2):
	x = loc2.x - loc1.x
	z = loc2.y - loc1.y
	return(math.sqrt(math.pow(x, 2) + math.pow(z, 2)))

def euclidian_distance_3d(loc1, loc2):
	x = loc2.x - loc1.x
	y = loc2.y - loc1.y
	z = loc2.z - loc1.z
	return(math.sqrt(math.pow(x, 2) + math.pow(y, 2) + math.pow(z, 2)))

def get_collisions(world, actor, min_distance=1):
	""""
	A function to use car_mesh to detect any collisions.
	World, Actor -> [Collsions]

	Gets all other cars in the sim and returns them in a list if the min distance between
	car mesh vertices is less than the "min_distance" (default 1)
	"""
	actor_mesh = car_mesh(actor)
	other_actors = list(world.get_actors())

	for act in other_actors:
		if act.id == actor.id:
			other_actors.remove(act)

	obstacles = other_actors
	collisions = []

	for obs in obstacles:
		if isinstance(obs, carla.libcarla.Vehicle):
			obs_mesh = car_mesh(obs)

			#dist = euclidian_distance_2d(actor.get_location(), obs.get_location())
			dist = actor_mesh.get_min_distance(obs_mesh)

			if (dist < min_distance):
				collisions.append(obs.id)

	return collisions

def draw_spawn_points(debug_helper, spawn_points, life_time=10.0):
	"""
	A helper function to draw spawn points with their index in the world
	Useful for spawning obstacle vehicles
	"""
	for sp in range(2, len(spawn_points)):
		sp_loc = spawn_points[sp].location
		#debug_helper.draw_point(spawn_points[sp].location, life_time=10.0, color=carla.Color(0, 0, 255))
		debug_helper.draw_string(sp_loc, "{}".format(sp), life_time=10.0, color=carla.Color(0, 0, 255))

def clear_vehicles(world):
	"""
	Destroy all vehicles in the world
	"""
	old_actors = list(world.get_actors())
	old_vehicles = [x for x in old_actors if isinstance(x, carla.libcarla.Vehicle)]
	for i in range(len(old_vehicles)):
		old_vehicles[i].destroy()

def main():
	# Connect to CARLA and get environment
	client = carla.Client("localhost", 9000)
	client.set_timeout(10)
	world = client.get_world()
	wmap = world.get_map()
	spawn_points = world.get_map().get_spawn_points()
	debug_helper = world.debug

	# Get Blueprints
	blueprint_library = world.get_blueprint_library()
	vehicle_bp = random.choice(blueprint_library.filter('vehicle.audi.a2'))
	obs_bp = random.choice(blueprint_library.filter('vehicle.audi.a2'))
	col_sensor_bp = random.choice(blueprint_library.filter('sensor.other.collision'))

	# Clean Up Set
	clear_vehicles(world)

	# Initalize Waypointsa
	start = spawn_points[0]
	end = spawn_points[1]
	start_waypoint = wmap.get_waypoint(start.location)
	end_waypoint = wmap.get_waypoint(end.location)
	debug_helper.draw_point(start.location, life_time=10.0)
	debug_helper.draw_point(end.location, life_time=10.0)
	draw_spawn_points(debug_helper, spawn_points)

	# A Star
	route = a_star(start_waypoint, end_waypoint)
	if route is None:
			print("Failed to find a path. Try adjusting the max_distance in the a_star function.")
			actor.destroy()

	# Spawn Actor, Obstacle
	actor = world.spawn_actor(vehicle_bp, start)
	obstacle = world.spawn_actor(obs_bp, spawn_points[11])

	# Track Path with Collision Detection
	for i, waypoint in enumerate(route):
		actor.set_transform(waypoint.transform)

		collisions = get_collisions(world, actor)

		if len(collisions) > 0:
			for col in collisions:
				print("Collision with actor {}".format(col))

		time.sleep(0.05)

	# Clean Up
	actor.destroy()
	obstacle.destroy()

if __name__ == '__main__':
	try:
		main()
	except Exception as e:
		print(e)
