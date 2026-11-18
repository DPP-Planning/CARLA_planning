'''
Lane Blocker (Scenario A)

Evaluates the agent with a single lane blocker.

To run this test individually, use:
scenic agents/navigation/scenic_test_A.scenic --simulate --2d --model scenic.simulators.carla.model
'''

# Import OpenDrive Road Network
param map = localPath('../../../../CarlaUE4/Content/Carla/Maps/OpenDrive/Town10HD.xodr')
param noise = 0
param timeout = 3600
model scenic.simulators.carla.model

# Import DPP Module
import sys
import random
sys.path.insert(0, "/home/ubuntu/persistent/CARLA_LATEST/PythonAPI/carla")
from agents.navigation.scenic_controller import DPPBehavior, MoveableObstacleBehavior, get_random_vector

start = get_random_vector(network, Lane)
goal = get_random_vector(network, Lane)

ego = new Car at start, with behavior DPPBehavior(goal, timeout=globalParameters.timeout), with color Color(1, 0, 0)
blocker = new Car ahead of ego by 10, with behavior MoveableObstacleBehavior(FollowLaneBehavior, timer=120)

mutate blocker by globalParameters.noise 
