'''
To run this file using the Carla simulator:
    scenic agents/navigation/scenic_test.scenic --simulate --2d --model scenic.simulators.carla.model

While CARLA is running on port 2000
'''
# Import OpenDrive Road Network
param map = localPath('../../../../CarlaUE4/Content/Carla/Maps/OpenDrive/Town10HD.xodr')
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
goal = new Point ahead of spawn by 8

# Spawn Ego
ego = new Car at spawn, with behavior DPP_Behavior(goal, 15), with color Color(1, 0, 0)
blocker = new Car ahead of ego by 4
