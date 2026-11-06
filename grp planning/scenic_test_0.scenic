'''
Unblocked Route (Scenario 0)

Evaluates agent with no blockers. Randomized start and end.

scenic agents/navigation/scenic_test.scenic --simulate --2d --model scenic.simulators.carla.model
'''

# Import OpenDrive Road Network
param map = localPath('../../../../CarlaUE4/Content/Carla/Maps/OpenDrive/Town10HD.xodr')
param noise = 0
param timeout = 15
model scenic.simulators.carla.model

# Import DPP Module
import sys
import random
sys.path.insert(0, "/home/ubuntu/persistent/CARLA_LATEST/PythonAPI/carla")
from agents.navigation.scenic_controller import DPP_Behavior

# Randomize Spawn Location
all_lane_sections = [x for x in list(network.elements.keys()) if isinstance(network.elements[x], LaneSection)]

start_choice = random.choice(all_lane_sections)
goal_choice = random.choice(all_lane_sections)

while start_choice != goal_choice: goal_choice = random.choice(all_lane_sections)

start_section = network.elements[start_choice]
goal_section = network.elements[goal_choice]

start = new OrientedPoint on start_section
goal = new OrientedPoint on goal_section

# Spawn Ego
ego = new Car at start, with behavior DPP_Behavior(goal, globalParameters.timeout), with color Color(1, 0, 0)
