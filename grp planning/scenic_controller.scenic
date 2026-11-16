import sys
sys.path.insert(0, "/home/ubuntu/persistent/CARLA_LATEST/PythonAPI/carla")
from scenic.core.simulators import Action
from scenic.domains.driving.actions import SetThrottleAction, SetSteerAction, SetBrakeAction
from agents.navigation.d_agent import DPP_Controller
import carla
import time

class EmptyAction(Action):
    def __init__(self):
        pass

    def applyTo(self, obj, sim):
        pass

behavior DPP_Behavior(goal, timeout=180):

    g = goal.position.coordinates
    t = 0    

    controller = DPP_Controller(
        self.carlaActor, 
        carla.Location(g[0], g[1], g[2]),
        self.carlaActor.get_world().get_map().get_spawn_points(), 
        search_algorithm="dlite"
    )

    controller._debug = False
    controller.start()   

    while not controller.done():
        take EmptyAction()

        t += 1
        if t >= timeout: 
            break

    print("exited main loop")

    if t > timeout:
        print("Scene: Agent timed out.")
    elif controller.done():
        print("Scene: Agent reached goal.")
    else:
        print("Scene: Agent failed unexpectedly.")

    terminate 

behavior Obstacle_Behavior(timer=-1):
    i = 0

    while True:
        i += 1
        if i == timer:
            break
        take EmptyAction()
        
    while True:
        i += 1
        take SetThrottleAction(1.0)

