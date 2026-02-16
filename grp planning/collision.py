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

	def __init__(self, world, actor, transform):
		self.id = actor.id
		self.center = transform
		self.world = world
		self.corners = [ loc for loc in actor.bounding_box.get_world_vertices(self.center)[1::2]]
		self.bb = actor.bounding_box

	def get_min_distance(self, mesh):
		min_distance = float('inf')

		for c in mesh.corners:
			# Check if intersecting
			if euclidian_distance_2d(c, self.center.location) < self.bb.extent.x:
				return 0.0
			# Check if colliding
			else:
				for i in self.corners:
					min_distance = min(min_distance, euclidian_distance_2d(c, i))

				return min_distance

	def print_vertices(self):
		print("Actor Mesh {}".format(self.id))
		for v in self.corners:
			print(" - (X: {}, Y:{})".format(v.x, v.y))

	def draw_mesh(self, life_time=5.0, color=carla.Color(r=100, g=0, b=0, a=150)):
		self.world.debug.draw_line(self.corners[0], self.corners[2], thickness=0.15, color=color, life_time=life_time)
		self.world.debug.draw_line(self.corners[2], self.corners[3], thickness=0.15, color=color, life_time=life_time)
		self.world.debug.draw_line(self.corners[3], self.corners[1], thickness=0.15, color=color, life_time=life_time)
		self.world.debug.draw_line(self.corners[1], self.corners[0], thickness=0.15, color=color, life_time=life_time)

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

def get_collisions(world, actor, min_distance=0.7):
	""""
	A function to use car_mesh to detect any collisions.
	World, Actor -> [Collsions]

	Gets all other cars in the sim and returns them in a list if the min distance between
	car mesh vertices is less than the "min_distance" (default 1)
	"""
	actor_mesh = car_mesh(world, actor, actor.get_transform())
	other_actors = list(world.get_actors())

	for act in other_actors:
		if act.id == actor.id:
			other_actors.remove(act)

	obstacles = other_actors
	collisions = []

	for obs in obstacles:
		if isinstance(obs, carla.libcarla.Vehicle):
			obs_mesh = car_mesh(world, obs, obs.get_transform())

			#dist = euclidian_distance_2d(actor.get_location(), obs.get_location())
			dist = actor_mesh.get_min_distance(obs_mesh)

			if (dist < min_distance):
				collisions.append(obs.id)

	return collisions

def get_projected_collisions(world, actor, waypoint, min_distance=0.7, max_distance=20, debug=False):
	""""
	Use the next waypoint from astar coupled with actor to build a projected bouding box.
	"""

	projected_transform = waypoint.transform

	actor_mesh = car_mesh(world, actor, projected_transform)
	#other_actors = list(world.get_actors())
	other_actors = [a for a in list(world.get_actors()) if euclidian_distance_2d(a.get_location(), actor.get_location()) <= max_distance]

	for act in other_actors:
		if act.id == actor.id:
			other_actors.remove(act)

	obstacles = other_actors
	collisions = []

	for obs in obstacles:
		if isinstance(obs, carla.libcarla.Vehicle):
			obs_mesh = car_mesh(world, obs, obs.get_transform())

			dist = actor_mesh.get_min_distance(obs_mesh)

			if dist < min_distance:
				if debug: obs_mesh.draw_mesh(life_time=1.0, color=carla.Color(r=100, g=0, b=0, a=50))
				if debug: actor_mesh.draw_mesh(life_time=1.0, color=carla.Color(r=100, g=100, b=0, a=50))
				collisions.append(obs)

	if len(collisions) == 0:
		if debug: actor_mesh.draw_mesh(life_time=1.0, color=carla.Color(r=0, g=100, b=0, a=50))

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

	#debug_helper.draw_point(start.location, life_time=10.0)
	#debug_helper.draw_point(end.location, life_time=10.0)
	#draw_spawn_points(debug_helper, spawn_points)

	# A Star
	route = a_star(start_waypoint, end_waypoint)
	if route is None:
			print("Failed to find a path. Try adjusting the max_distance in the a_star function.")
			actor.destroy()

	# Spawn Actor, Obstacle
	actor = world.spawn_actor(vehicle_bp, start)
	obstacle = world.spawn_actor(obs_bp, spawn_points[11])

	# Track Path with Collision Detection

	route_length = len(route)

	for i in range(route_length):
		waypoint = route[i]

		actor.set_transform(waypoint.transform)

		if i != route_length - 1:
			collisions = get_projected_collisions(world, actor, route[i+1])

			if len(collisions) > 0:
				for col in collisions:
					print("Collision with actor {}".format(col))
				exit()

		time.sleep(0.05)

	# Clean Up
	actor.destroy()
	obstacle.destroy()

if __name__ == '__main__':
	try:
		main()
	except Exception as e:
		print(e)
