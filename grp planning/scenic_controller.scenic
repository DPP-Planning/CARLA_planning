import sys
sys.path.insert(0, "/home/ubuntu/persistent/CARLA_LATEST/PythonAPI/carla")
from scenic.core.simulators import Action
from scenic.domains.driving.actions import SetThrottleAction, SetSteerAction, SetBrakeAction
from agents.navigation.d_agent import DPP_Controller
import carla
import time
import random

param map = localPath('../../../../CarlaUE4/Content/Carla/Maps/OpenDrive/Town10HD.xodr')

def get_random_vector(network, road_element, avoid_junctions=True):
    all_lane_sections = [x for x in list(network.elements.keys()) if isinstance(network.elements[x], road_element)]
    choice = random.choice(all_lane_sections)
    return new OrientedPoint on network.elements[choice]

class EmptyAction(Action):
    def __init__(self):
        pass

    def applyTo(self, obj, sim):
        pass

behavior DPPBehavior(goal, timeout=180):

    """
    Behavior to initialize and run DPP Controller instance. Runs in a seperate thread.
    Terminates when Controller signals completion. 
    Terminates when timeout elsapses
    """

    g = goal.position.coordinates
    t = 0    

    controller = DPP_Controller(
        self.carlaActor, 
        carla.Location(g[0], g[1], g[2]),
        self.carlaActor.get_world().get_map().get_spawn_points(), 
        search_algorithm="dlite"
    )

    controller.start()
    goal_distance = self.carlaActor.get_location().distance(carla.Location(g[0], g[1], g[2]))

    while not controller.done():
        goal_distance = self.carlaActor.get_location().distance(carla.Location(g[0], g[1], g[2]))
        #print(goal_distance)
        take EmptyAction()

        t += 1
        if t >= timeout: 
            break

    if t >= timeout:
        print(f"Scene: Agent timed out. Distance from goal {goal_distance}")
    elif goal_distance <= 4:
        print("Scene: Agent reached goal.")
    else:
        print(f"Scene: Agent failed unexpectedly. Distance from goal {goal_distance}")

    terminate 

behavior MoveableObstacleBehavior(next, timer=-1):
    i = 0

    while True:
        i += 1
        if i == timer:
            break
        take EmptyAction()
        
    while True:
        do next()
