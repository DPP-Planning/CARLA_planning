'''
Lane Blocker (Scenario B)

Evaluates the agent with two lane blockers.

To run this test individually, use:
scenic agents/navigation/scenic_test_B.scenic --simulate --2d --model scenic.simulators.carla.model
'''

# Import OpenDrive Road Network
param map = localPath('../../../../CarlaUE4/Content/Carla/Maps/OpenDrive/Town10HD.xodr')
param noise = 0
param timeout = 3600
param ps = ""
param mp = ""
model scenic.simulators.carla.model

# Import DPP Module
import sys
import random
sys.path.insert(0, "/home/ubuntu/persistent/CARLA_LATEST/PythonAPI/carla")
from agents.navigation.scenic_controller import DPPBehavior, MoveableObstacleBehavior, get_random_vector

start = get_random_vector(network, Lane)
goal = get_random_vector(network, Lane)

ego = new Car at start, with behavior DPPBehavior(goal, timeout=globalParameters.timeout, mp=globalParameters.mp, ps=globalParameters.ps), with color Color(1, 0, 0)
blocker_1 = new Car ahead of ego by 15, with behavior MoveableObstacleBehavior(FollowLaneBehavior, timer=-1)
blocker_2 = new Car right of blocker_1 by 3, with behavior MoveableObstacleBehavior(FollowLaneBehavior, timer=90)

mutate blocker_1 by globalParameters.noise 
mutate blocker_2 by globalParameters.noise 
