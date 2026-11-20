import importlib
import importlib.util
import math
import sys
import types
from collections import deque
from enum import IntEnum, IntFlag
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
    def __init__(self, waypoint_id, x, is_junction=False, lane_change=0):
        self.id = waypoint_id
        self.transform = types.SimpleNamespace(location=FakeLocation(x))
        self.is_junction = is_junction
        self.lane_id = 1
        self.lane_change = lane_change
        self.lane_type = None
        self.next_waypoints = []
        self.left_lane = None
        self.right_lane = None

    def next(self, distance):
        return self.next_waypoints

    def get_left_lane(self):
        return self.left_lane

    def get_right_lane(self):
        return self.right_lane


class FakeDebug:
    def __init__(self):
        self.drawn = []

    def draw_string(self, location, text, **kwargs):
        self.drawn.append((location, text))


@pytest.fixture
def planning_modules(monkeypatch):
    monkeypatch.syspath_prepend(str(PROJECT_ROOT))

    fake_carla = types.ModuleType("carla")
    fake_carla.Vehicle = type("Vehicle", (), {})
    fake_carla.Waypoint = FakeWaypoint
    fake_carla.Location = FakeLocation
    fake_carla.Color = lambda **kwargs: kwargs
    fake_carla.VehicleControl = type("VehicleControl", (), {})
    fake_carla.LaneChange = IntFlag(
        "LaneChange", {"None_": 0, "Left": 1, "Right": 2}
    )
    fake_carla.LaneType = IntEnum("LaneType", {"Driving": 1, "Sidewalk": 2})
    fake_carla.libcarla = types.SimpleNamespace(Waypoint=FakeWaypoint)
    monkeypatch.setitem(sys.modules, "carla", fake_carla)

    fake_generate_map = types.ModuleType("Generate_map")
    fake_generate_map.gen_map_initial = lambda *args, **kwargs: None
    fake_generate_map.save_waypoint_graph = lambda *args, **kwargs: None
    monkeypatch.setitem(sys.modules, "Generate_map", fake_generate_map)

    agents = types.ModuleType("agents")
    agents.__path__ = []
    navigation = types.ModuleType("agents.navigation")
    navigation.__path__ = []
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

    astar_path = PROJECT_ROOT / "grp planning" / "astar.py"
    astar_spec = importlib.util.spec_from_file_location(
        "astar_under_test", astar_path
    )
    astar_module = importlib.util.module_from_spec(astar_spec)
    astar_spec.loader.exec_module(astar_module)
    return dlite, agent_module, astar_module


def make_dstar(dlite, waypoints, graph, start, goal):
    dstar = dlite.DStarLite.__new__(dlite.DStarLite)
    dstar.resolution = 1.0
    dstar.start = start
    dstar.goal = goal
    dstar.all_waypoints = waypoints
    dstar.waypoint_lookup = {waypoint.id: waypoint for waypoint in waypoints}
    dstar.waypoint_graph = graph
    dstar._waypoint_spatial_index = dstar._build_waypoint_spatial_index()
    dstar.all_obst_wps = {}
    dstar.nodes_expanded = 0
    return dstar


def test_initialize_resolves_start_and_goal_to_generated_waypoints(planning_modules):
    dlite, _, _ = planning_modules
    generated_start = FakeWaypoint(1, 0)
    generated_goal = FakeWaypoint(2, 4)
    start = FakeWaypoint(101, 0.2)
    goal = FakeWaypoint(102, 3.8)
    world = types.SimpleNamespace(
        get_map=lambda: object(),
        debug=FakeDebug(),
    )
    dstar = dlite.DStarLite(
        world,
        start,
        goal,
        [generated_start, generated_goal],
        {},
        vehicle=None,
        waypoint_graph={
            1: {"successors": [2], "predecessors": []},
            2: {"successors": [], "predecessors": [1]},
        },
        waypoint_lookup={1: generated_start, 2: generated_goal},
    )

    dstar.initialize()

    assert dstar.start is generated_start
    assert dstar.goal is generated_goal
    assert dstar.nodes_expanded == 1
    assert {text for _, text in world.debug.drawn} == {"start", "goal"}


def test_graph_neighbor_lookup_counts_expanded_node(planning_modules):
    dlite, _, _ = planning_modules
    first, second = FakeWaypoint(1, 0), FakeWaypoint(2, 1)
    dstar = make_dstar(
        dlite,
        [first, second],
        {1: {"successors": [2], "predecessors": []}},
        first,
        second,
    )
    dstar.nodes_expanded = 0

    assert dstar._graph_neighbors(first, "successors") == [second]
    assert dstar.nodes_expanded == 1


def test_best_successor_resolves_waypoint_and_returns_sole_successor(
    planning_modules,
):
    dlite, _, _ = planning_modules
    current, successor = FakeWaypoint(1, 0), FakeWaypoint(2, 1)
    goal = FakeWaypoint(3, 10)
    dstar = make_dstar(
        dlite,
        [current, successor, goal],
        {
            1: {"successors": [2], "predecessors": []},
            2: {"successors": [], "predecessors": [1]},
            3: {"successors": [], "predecessors": []},
        },
        current,
        goal,
    )
    planner = dlite.ThreadedDStarLite(dstar)
    off_graph_current = FakeWaypoint(99, 0.1)

    assert planner.get_best_successor(off_graph_current) is successor


def test_best_successor_uses_cost_when_multiple_successors(planning_modules):
    dlite, _, _ = planning_modules
    current = FakeWaypoint(1, 0)
    expensive, cheap = FakeWaypoint(2, 1), FakeWaypoint(3, 2)
    goal = FakeWaypoint(4, 10)
    dstar = make_dstar(
        dlite,
        [current, expensive, cheap, goal],
        {
            1: {"successors": [2, 3], "predecessors": []},
            2: {"successors": [], "predecessors": [1]},
            3: {"successors": [], "predecessors": [1]},
            4: {"successors": [], "predecessors": []},
        },
        current,
        goal,
    )
    dstar.g = {1: 0, 2: 8, 3: 1, 4: 0}
    dstar.rhs = dict(dstar.g)
    planner = dlite.ThreadedDStarLite(dstar)
    planner._build_waypoint_index()

    assert planner.get_best_successor(current) is cheap


class FakeSearch:
    def __init__(self, successors):
        self.successors = successors
        self.calls = []
        self._needs_replan = Event()

    def get_best_successor(self, waypoint):
        self.calls.append(waypoint)
        return self.successors.get(waypoint.id)


def make_agent(agent_module, current, destination, search):
    agent = agent_module.BasicAgentD.__new__(agent_module.BasicAgentD)
    agent.current_wp = current
    agent._destination = destination
    agent._search = search
    agent._queue = deque()
    agent._q_max = 10
    agent._debug = False
    agent._vehicle = types.SimpleNamespace(
        get_location=lambda: current.transform.location
    )
    agent._map = types.SimpleNamespace(
        get_waypoint=lambda location: current
    )
    agent._world = types.SimpleNamespace(debug=FakeDebug())
    return agent


def test_junction_route_follows_successors_until_leaving_junction(planning_modules):
    _, agent_module, _ = planning_modules
    first = FakeWaypoint(1, 0, is_junction=True)
    second = FakeWaypoint(2, 1, is_junction=True)
    exit_waypoint = FakeWaypoint(3, 2, is_junction=False)
    destination = FakeLocation(20)
    search = FakeSearch({1: second, 2: exit_waypoint})
    agent = make_agent(agent_module, first, destination, search)

    agent._get_junction_route()

    assert list(agent._queue) == [second, exit_waypoint]
    assert search.calls == [first, second]


def test_junction_route_stops_when_successor_reaches_destination(planning_modules):
    _, agent_module, _ = planning_modules
    current = FakeWaypoint(1, 0, is_junction=True)
    destination_waypoint = FakeWaypoint(2, 9, is_junction=False)
    search = FakeSearch({1: destination_waypoint})
    agent = make_agent(
        agent_module,
        current,
        FakeLocation(10),
        search,
    )

    agent._get_junction_route()

    assert list(agent._queue) == [destination_waypoint]
    assert search.calls == [current]


def test_junction_route_stops_on_cycle(planning_modules):
    _, agent_module, _ = planning_modules
    first = FakeWaypoint(1, 0, is_junction=True)
    second = FakeWaypoint(2, 1, is_junction=True)
    search = FakeSearch({1: second, 2: first})
    agent = make_agent(agent_module, first, FakeLocation(20), search)

    agent._get_junction_route()

    assert list(agent._queue) == [second]
    assert search.calls == [first, second]


def test_junction_route_stops_when_no_successor(planning_modules):
    _, agent_module, _ = planning_modules
    current = FakeWaypoint(1, 0, is_junction=True)
    search = FakeSearch({})
    agent = make_agent(agent_module, current, FakeLocation(20), search)

    agent._get_junction_route()

    assert list(agent._queue) == []
    assert search.calls == [current]


def test_waypoint_refill_preserves_queued_route(planning_modules):
    _, agent_module, _ = planning_modules
    current, first, second = (
        FakeWaypoint(1, 0),
        FakeWaypoint(2, 1),
        FakeWaypoint(3, 2),
    )
    existing = FakeWaypoint(4, -1)
    search = FakeSearch({1: first, 2: second})
    agent = make_agent(agent_module, current, FakeLocation(20), search)
    agent._q_max = 3
    agent._queue.append(existing)

    agent._get_next_waypoints()

    assert list(agent._queue) == [existing, first, second]
    assert search.calls == [current, first]


def test_waypoint_refill_keeps_queued_route_during_replan(planning_modules):
    _, agent_module, _ = planning_modules
    current, existing, next_waypoint = (
        FakeWaypoint(1, 0),
        FakeWaypoint(2, 1),
        FakeWaypoint(3, 2),
    )
    search = FakeSearch({1: next_waypoint})
    search._needs_replan.set()
    agent = make_agent(agent_module, current, FakeLocation(20), search)
    agent._queue.append(existing)

    agent._get_next_waypoints()

    assert list(agent._queue) == [existing]
    assert search.calls == []


def test_astar_neighbors_include_forward_and_legal_driving_lane_change(
    planning_modules,
):
    _, _, astar = planning_modules
    carla = sys.modules["carla"]
    start = FakeWaypoint(
        1, 0, lane_change=carla.LaneChange.Left | carla.LaneChange.Right
    )
    forward = FakeWaypoint(2, 2)
    left_lane = FakeWaypoint(3, 0)
    left_lane.lane_type = carla.LaneType.Driving
    left_next = FakeWaypoint(4, 2)
    left_lane.next_waypoints = [left_next]
    right_lane = FakeWaypoint(5, 0)
    right_lane.lane_type = carla.LaneType.Sidewalk
    start.next_waypoints = [forward]
    start.left_lane = left_lane
    start.right_lane = right_lane

    assert astar.get_legal_neighbors(start) == [forward, left_lane, left_next]


def test_astar_ignores_dead_obstacles_and_missing_mesh(planning_modules):
    _, _, astar = planning_modules
    waypoint = FakeWaypoint(1, 0)

    class Mesh:
        def contains_waypoint(self, candidate, radius_m):
            return candidate is waypoint and radius_m == 0.2

    assert astar._near_obstacle(
        waypoint,
        [{"actor": types.SimpleNamespace(is_alive=False), "mesh": Mesh()}],
    ) is False
    assert astar._near_obstacle(
        waypoint,
        [{"actor": types.SimpleNamespace(is_alive=True), "mesh": None}],
    ) is False
    assert astar._near_obstacle(
        waypoint,
        [
            {
                "actor": types.SimpleNamespace(is_alive=True),
                "mesh": Mesh(),
            }
        ],
    ) is True


def test_astar_returns_route_around_perceived_obstacle(planning_modules):
    _, _, astar = planning_modules
    start, blocked, alternate, next_waypoint, near_goal, goal = [
        FakeWaypoint(waypoint_id, x)
        for waypoint_id, x in enumerate((0, 1, 2, 4, 6, 8), start=1)
    ]
    start.next_waypoints = [blocked, alternate]
    alternate.next_waypoints = [next_waypoint]
    next_waypoint.next_waypoints = [near_goal]
    near_goal.next_waypoints = [goal]

    class Mesh:
        def contains_waypoint(self, candidate, radius_m):
            return candidate is blocked

    obstacles = [
        {
            "actor": types.SimpleNamespace(is_alive=True),
            "mesh": Mesh(),
        }
    ]

    route = astar.a_star(start, goal, world=None, seen_obstacles_snapshot=obstacles)

    assert route[0] is start
    assert blocked not in route
    assert route[-1] is near_goal


def test_dpp_controller_starts_planner_and_agent_and_reports_completion(
    planning_modules, monkeypatch
):
    _, agent_module, _ = planning_modules
    start, goal = FakeWaypoint(1, 0), FakeWaypoint(2, 10)
    world_map = types.SimpleNamespace(
        get_waypoint=lambda location: start if location.x == 0 else goal
    )
    world = types.SimpleNamespace(get_map=lambda: world_map)
    vehicle = types.SimpleNamespace(
        get_world=lambda: world,
        get_location=lambda: FakeLocation(0),
    )
    map_data = types.SimpleNamespace(
        all_waypoints=[start, goal],
        wp_pts={},
        waypoint_graph={},
        waypoint_lookup={1: start, 2: goal},
    )
    monkeypatch.setattr(agent_module, "gen_map_initial", lambda *args, **kwargs: map_data)
    monkeypatch.setattr(agent_module, "DStarLite", lambda *args, **kwargs: "dstar")

    calls = []

    class Search:
        def __init__(self, dstar):
            calls.append(("make_search", dstar))

        def start(self):
            calls.append(("start_search",))

    class Agent:
        def __init__(self, vehicle, destination, search):
            self.completed = False
            calls.append(("make_agent", vehicle, destination, search))

        def start(self):
            calls.append(("start_agent",))

        def done(self):
            return self.completed

    monkeypatch.setattr(agent_module, "ThreadedDStarLite", Search)
    monkeypatch.setattr(agent_module, "BasicAgentD", Agent)
    controller = agent_module.DPP_Controller(vehicle, FakeLocation(10), [])

    controller.start()
    assert calls[-2:] == [("start_search",), ("start_agent",)]
    assert controller.done() is False

    controller._agent.completed = True
    assert controller.done() is True


def test_dpp_controller_astar_strategy_sets_destination(planning_modules, monkeypatch):
    _, agent_module, _ = planning_modules
    start, goal = FakeWaypoint(1, 0), FakeWaypoint(2, 10)
    world_map = types.SimpleNamespace(
        get_waypoint=lambda location: start if location.x == 0 else goal
    )
    vehicle = types.SimpleNamespace(
        get_world=lambda: types.SimpleNamespace(get_map=lambda: world_map),
        get_location=lambda: FakeLocation(0),
    )
    map_data = types.SimpleNamespace(
        all_waypoints=[start, goal],
        wp_pts={},
    )
    monkeypatch.setattr(agent_module, "gen_map_initial", lambda *args, **kwargs: map_data)

    class BasicAgent:
        def __init__(self, vehicle):
            self.destination = None

        def set_destination(self, destination, start):
            self.destination = (destination, start)

    monkeypatch.setattr(agent_module, "BasicAgent", BasicAgent)
    destination = FakeLocation(10)

    controller = agent_module.DPP_Controller(
        vehicle, destination, [], search_algorithm="astar"
    )

    assert controller._search is None
    assert controller._agent.destination == (destination, None)


def test_agent_loop_signals_done_and_brakes_when_already_at_destination(
    planning_modules,
):
    _, agent_module, _ = planning_modules
    location = FakeLocation(5)
    applied_controls = []
    vehicle = types.SimpleNamespace(
        get_location=lambda: location,
        apply_control=applied_controls.append,
    )
    agent = agent_module.BasicAgentD.__new__(agent_module.BasicAgentD)
    agent._vehicle = vehicle
    agent._destination = location
    agent._stop_event = Event()

    agent._agent_loop()

    assert agent.done() is True
    assert len(applied_controls) == 1
    assert applied_controls[0].brake == 1.0
    assert applied_controls[0].throttle == 0.0
