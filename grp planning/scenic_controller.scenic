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

behavior DPP_Behavior(goal, timeout=15):

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
        print(distance from goal)
        #control = controller._agent.run_step()
        #take SetThrottleAction(control.throttle), SetSteerAction(control.steer), SetBrakeAction(control.brake)
        take EmptyAction()
    terminate 
