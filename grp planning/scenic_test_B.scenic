'''
Parallel Blocker (Scenario B)

Test the agent with a blocker, and another blockers directly next to it.

scenic agents/navigation/scenic_test.scenic --simulate --2d --model scenic.simulators.carla.model
'''

# Define Global Paramters
param map = localPath('../../../../CarlaUE4/Content/Carla/Maps/OpenDrive/Town10HD.xodr')
param noise = 0
param timeout = 15

# Define Model
model scenic.simulators.carla.model

# Import DPP Module
import sys
import random
sys.path.insert(0, "/home/ubuntu/persistent/CARLA_LATEST/PythonAPI/carla")
from agents.navigation.scenic_controller import DPP_Behavior

# Randomize Spawn Location
lane_sections = [x for x in list(network.elements.keys()) if isinstance(network.elements[x], LaneSection)]
choice = random.choice(lane_sections)
road = network.elements[choice]

spawn = new OrientedPoint on road
goal = new Point ahead of spawn by 5

# Spawn Ego
ego = new Car at spawn, with behavior DPP_Behavior(goal, globalParameters.timeout), with color Color(1, 0, 0)
blocker_1 = new Car ahead of ego by 4
blocker_2 = new Car left of blocker_1 by 1

mutate blocker_1 by globalParameters.noise
mutate blocker_2 by globalParameters.noise
