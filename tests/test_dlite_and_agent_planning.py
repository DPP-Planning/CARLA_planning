import importlib
import importlib.util
import math
import sys
import types
from collections import deque
from pathlib import Path
from threading import Event

import pytest


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class FakeLocation:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def distance(self, other):
        return math.sqrt(
            (self.x - other.x) ** 2
            + (self.y - other.y) ** 2
            + (self.z - other.z) ** 2
        )


class FakeWaypoint:
    def __init__(self, waypoint_id, x, y=0.0, z=0.0):
        self.id = waypoint_id
        self.transform = types.SimpleNamespace(
            location=FakeLocation(x, y, z)
        )
        self.lane_id = 1


@pytest.fixture
def planning_modules(monkeypatch):
    monkeypatch.syspath_prepend(str(PROJECT_ROOT))

    fake_carla = types.ModuleType("carla")
    fake_carla.Vehicle = type("Vehicle", (), {})
    fake_carla.Location = FakeLocation
    fake_carla.Color = lambda **kwargs: kwargs
    fake_carla.VehicleControl = type("VehicleControl", (), {})
    fake_carla.libcarla = types.SimpleNamespace(Waypoint=FakeWaypoint)
    monkeypatch.setitem(sys.modules, "carla", fake_carla)

    fake_generate_map = types.ModuleType("Generate_map")
    fake_generate_map.gen_map_initial = lambda *args, **kwargs: None
    fake_generate_map.save_waypoint_graph = lambda *args, **kwargs: None
    monkeypatch.setitem(sys.modules, "Generate_map", fake_generate_map)

    navigation = types.ModuleType("agents.navigation")
    navigation.__path__ = []
    agents = types.ModuleType("agents")
    agents.__path__ = []
    tools = types.ModuleType("agents.tools")
    tools.__path__ = []

    basic_agent = types.ModuleType("agents.navigation.basic_agent")
    basic_agent.BasicAgent = type("BasicAgent", (), {})
    controller = types.ModuleType("agents.navigation.controller")
    controller.VehiclePIDController = type("VehiclePIDController", (), {})
    misc = types.ModuleType("agents.tools.misc")
    misc.get_speed = lambda vehicle: 0.0
    navigation_generate_map = types.ModuleType(
        "agents.navigation.Generate_map"
    )
    navigation_generate_map.gen_map_initial = lambda *args, **kwargs: None

    for name, module in (
        ("agents", agents),
        ("agents.navigation", navigation),
        ("agents.tools", tools),
        ("agents.navigation.basic_agent", basic_agent),
        ("agents.navigation.controller", controller),
        ("agents.tools.misc", misc),
        ("agents.navigation.Generate_map", navigation_generate_map),
    ):
        monkeypatch.setitem(sys.modules, name, module)

    dlite = importlib.import_module("dLite.dlite")

    agent_path = PROJECT_ROOT / "grp planning" / "d_agent.py"
    spec = importlib.util.spec_from_file_location(
        "d_agent_under_test", agent_path
    )
    agent_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(agent_module)
    return dlite, agent_module


def make_dstar(dlite, waypoints, graph, start, goal):
    dstar = dlite.DStarLite.__new__(dlite.DStarLite)
    dstar.resolution = 1.0
    dstar.start = start
    dstar.goal = goal
    dstar.all_waypoints = waypoints
    dstar.waypoint_lookup = {waypoint.id: waypoint for waypoint in waypoints}
    dstar.waypoint_graph = graph
    dstar._waypoint_spatial_index = dstar._build_waypoint_spatial_index()
    return dstar


def test_perception_state_tracks_and_consumes_path_updates(planning_modules):
    dlite, _ = planning_modules
    state = dlite.PerceptionState()
    first, second = FakeWaypoint(1, 0), FakeWaypoint(2, 1)

    state.set_path([first, None, second])
    updated, path = state.consume_path_update()

    assert updated is True
    assert path == {0: first.id, 2: second.id}
    path.clear()
    assert state.consume_path_update() == (False, {0: first.id, 2: second.id})


def test_perception_state_tracks_and_clears_obstacle_blocking(planning_modules):
    dlite, _ = planning_modules
    state = dlite.PerceptionState()
    state.mark_obstacle(FakeWaypoint(9, 0))
    state.mark_obstacle(FakeWaypoint(3, 1))

    assert state.snapshot() == {
        "seen_obstacles": [3, 9],
        "obstacle_blocking": True,
        "path_updated": False,
        "path": {},
    }

    state.clear_obstacle_blocking()
    assert state.snapshot()["obstacle_blocking"] is False


def test_graph_neighbors_resolve_generated_waypoints_and_ignore_unknown_ids(
    planning_modules,
):
    dlite, _ = planning_modules
    first = FakeWaypoint(1, 0)
    nearest = FakeWaypoint(2, 0.8)
    waypoints = [first, nearest]
    dstar = make_dstar(
        dlite,
        waypoints,
        {2: {"successors": [1, 404], "predecessors": []}},
        start=first,
        goal=nearest,
    )
    off_graph_waypoint = FakeWaypoint(99, 0.7)

    assert dstar._closest_generated_waypoint(off_graph_waypoint) is nearest
    assert dstar._graph_neighbors(off_graph_waypoint, "successors") == [first]


def test_successors_include_goal_when_near_destination(planning_modules):
    dlite, _ = planning_modules
    start = FakeWaypoint(1, 0)
    neighbor = FakeWaypoint(2, 1)
    goal = FakeWaypoint(3, 1.2)
    dstar = make_dstar(
        dlite,
        [start, neighbor, goal],
        {1: {"successors": [2], "predecessors": []}},
        start,
        goal,
    )

    assert dstar.successors(start) == [neighbor, goal]


def test_threaded_planner_selects_successor_with_lowest_path_cost(planning_modules):
    dlite, _ = planning_modules
    start = FakeWaypoint(1, 0)
    expensive = FakeWaypoint(2, 1)
    cheap = FakeWaypoint(3, 2)
    goal = FakeWaypoint(4, 10)
    dstar = make_dstar(
        dlite,
        [start, expensive, cheap, goal],
        {
            1: {"successors": [2, 3], "predecessors": []},
            2: {"successors": [], "predecessors": [1]},
            3: {"successors": [], "predecessors": [1]},
            4: {"successors": [], "predecessors": []},
        },
        start,
        goal,
    )
    dstar.g = {1: 0, 2: 8, 3: 1, 4: 0}
    dstar.rhs = dict(dstar.g)
    planner = dlite.ThreadedDStarLite(dstar)
    planner._build_waypoint_index()

    assert planner.get_best_successor(start) is cheap
    assert planner.get_best_successor(None) is None


class FakeDebug:
    def __init__(self):
        self.drawn = []

    def draw_string(self, location, text, **kwargs):
        self.drawn.append((location, text))


class FakeSearch:
    def __init__(self, successors, replan=False):
        self.successors = successors
        self._needs_replan = Event()
        if replan:
            self._needs_replan.set()
        self.calls = []

    def get_best_successor(self, waypoint):
        self.calls.append(waypoint)
        return self.successors.get(waypoint.id)


def make_agent(agent_module, start, search, max_waypoints=10):
    agent = agent_module.BasicAgentD.__new__(agent_module.BasicAgentD)
    agent._queue = deque([FakeWaypoint(-1, -1)])
    agent._q_max = max_waypoints
    agent._search = search
    agent._vehicle = types.SimpleNamespace(
        get_location=lambda: start.transform.location
    )
    agent._map = types.SimpleNamespace(get_waypoint=lambda location: start)
    agent._world = types.SimpleNamespace(debug=FakeDebug())
    return agent


def test_agent_waypoint_queue_stops_on_cycle_and_respects_capacity(
    planning_modules,
):
    _, agent_module = planning_modules
    start, first, second = (
        FakeWaypoint(1, 0),
        FakeWaypoint(2, 1),
        FakeWaypoint(3, 2),
    )
    search = FakeSearch({1: first, 2: second, 3: first})
    agent = make_agent(agent_module, start, search, max_waypoints=2)

    agent._get_next_waypoints()

    assert list(agent._queue) == [first, second]
    assert search.calls == [start, first]
    assert [text for _, text in agent._world.debug.drawn] == ["*", "*"]


def test_agent_clears_old_queue_while_replan_is_pending(planning_modules):
    _, agent_module = planning_modules
    start, next_waypoint = FakeWaypoint(1, 0), FakeWaypoint(2, 1)
    search = FakeSearch({1: next_waypoint}, replan=True)
    agent = make_agent(agent_module, start, search)

    agent._get_next_waypoints()

    assert list(agent._queue) == []
    assert search.calls == []
