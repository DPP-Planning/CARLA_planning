"""
obstacle_spawner.py

Formalized API for spawning obstacle actors in CARLA test scenarios.

Consolidates the hand-derived `carla.Transform` offset math and manual
try/finally actor cleanup found across test_scenarios/*.py into:

  1. ObstacleType   - a named taxonomy of obstacle categories.
  2. ObstacleSpec    - a declarative description of one obstacle to spawn.
  3. ObstacleSpawner - spawns/tracks obstacles and guarantees cleanup.
"""

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional, List
import random

import carla


class ObstacleType(Enum):
    LANE_BLOCKER = auto()
    ADJACENT_LANE_BLOCKER = auto()
    INTERSECTION_BLOCKER = auto()
    FOLLOWING_TOO_CLOSE = auto()
    OPPOSING_LANE = auto()
    MOVING_OBSTACLE = auto()
    JUNCTION_CROSS_TRAFFIC = auto()
    PEDESTRIAN = auto()
    STATIC_PROP = auto()


@dataclass
class ObstacleSpec:
    obstacle_type: ObstacleType
    offset: "carla.Location" = field(default_factory=lambda: carla.Location(0.0, 0.0, 0.0))
    blueprint_filter: str = "vehicle.audi.a2"
    rotation: Optional["carla.Rotation"] = None
    autopilot: bool = False
    label: Optional[str] = None


DEFAULT_OFFSETS = {
    ObstacleType.LANE_BLOCKER: carla.Location(-20.0, 0.0, 0.0),
    ObstacleType.ADJACENT_LANE_BLOCKER: carla.Location(-20.0, 4.0, 0.0),
    ObstacleType.INTERSECTION_BLOCKER: carla.Location(-65.0, 1.25, 0.0),
    ObstacleType.FOLLOWING_TOO_CLOSE: carla.Location(0.0, 5.0, 0.0),
    ObstacleType.OPPOSING_LANE: carla.Location(-3.5, 0.0, 0.0),
    ObstacleType.JUNCTION_CROSS_TRAFFIC: carla.Location(0.0, 20.0, 0.0),
}

DEFAULT_BLUEPRINTS = {
    ObstacleType.PEDESTRIAN: "walker.pedestrian.*",
    ObstacleType.STATIC_PROP: "static.prop.streetbarrier",
}


class ObstacleSpawner:
    def __init__(self, world: "carla.World", reference: "carla.Transform"):
        self._world = world
        self._blueprint_library = world.get_blueprint_library()
        self._reference = reference
        self._actors: List["carla.Actor"] = []
        self._specs: List[ObstacleSpec] = []

    def spawn(self, obstacle_type: ObstacleType, offset=None, blueprint_filter=None,
              rotation=None, autopilot: bool = False, label: Optional[str] = None,
              reference: Optional["carla.Transform"] = None):
        """
        Spawn a single obstacle. By default, `offset` is applied relative to
        the reference transform passed to the constructor (typically the
        agent's spawn point). Pass `reference` to offset from a different
        transform instead (e.g. a separate spawn point, as in
        junction_test.py or yellow_line.py) without needing a second
        ObstacleSpawner instance.
        """
        spec = ObstacleSpec(
            obstacle_type=obstacle_type,
            offset=offset if offset is not None else DEFAULT_OFFSETS.get(obstacle_type, carla.Location(0.0, 0.0, 0.0)),
            blueprint_filter=blueprint_filter or DEFAULT_BLUEPRINTS.get(obstacle_type, "vehicle.audi.a2"),
            rotation=rotation,
            autopilot=autopilot,
            label=label or obstacle_type.name,
        )
        return self._spawn_from_spec(spec, reference=reference)

    def spawn_at(self, transform: "carla.Transform", obstacle_type: ObstacleType = ObstacleType.STATIC_PROP,
                 blueprint_filter: Optional[str] = None, autopilot: bool = False,
                 label: Optional[str] = None):
        """
        Spawn an obstacle at an absolute transform, bypassing offset math
        entirely. Useful for obstacles defined by their own spawn point
        (e.g. `spawn_points[11]` in yellow_line.py) rather than as an
        offset from the agent.
        """
        candidates = self._blueprint_library.filter(blueprint_filter or DEFAULT_BLUEPRINTS.get(obstacle_type, "vehicle.audi.a2"))
        if not candidates:
            raise ValueError(f"No blueprints matched filter '{blueprint_filter}' for {obstacle_type.name}")
        bp = random.choice(candidates)

        actor = self._world.try_spawn_actor(bp, transform)
        if actor is None:
            raise RuntimeError(f"Failed to spawn obstacle '{label or obstacle_type.name}' at {transform}.")

        if autopilot and hasattr(actor, "set_autopilot"):
            actor.set_autopilot(True)

        spec = ObstacleSpec(obstacle_type=obstacle_type, blueprint_filter=blueprint_filter or "vehicle.audi.a2",
                             autopilot=autopilot, label=label or obstacle_type.name)
        self._actors.append(actor)
        self._specs.append(spec)
        print(f"[ObstacleSpawner] spawned {spec.label} ({obstacle_type.name}) at {transform}")
        return actor

    def spawn_batch(self, specs: List[ObstacleSpec], reference: Optional["carla.Transform"] = None):
        return [self._spawn_from_spec(s, reference=reference) for s in specs]

    def _spawn_from_spec(self, spec: ObstacleSpec, reference: Optional["carla.Transform"] = None):
        ref = reference if reference is not None else self._reference
        ref_loc = ref.location
        ref_rot = ref.rotation
        location = carla.Location(ref_loc.x + spec.offset.x, ref_loc.y + spec.offset.y, ref_loc.z + spec.offset.z)
        rotation = spec.rotation if spec.rotation is not None else ref_rot
        transform = carla.Transform(location, rotation)

        candidates = self._blueprint_library.filter(spec.blueprint_filter)
        if not candidates:
            raise ValueError(f"No blueprints matched filter '{spec.blueprint_filter}' for {spec.obstacle_type.name}")
        bp = random.choice(candidates)

        actor = self._world.try_spawn_actor(bp, transform)
        if actor is None:
            raise RuntimeError(f"Failed to spawn obstacle '{spec.label}' ({spec.obstacle_type.name}) at {transform}.")

        if spec.autopilot and hasattr(actor, "set_autopilot"):
            actor.set_autopilot(True)

        self._actors.append(actor)
        self._specs.append(spec)
        print(f"[ObstacleSpawner] spawned {spec.label} ({spec.obstacle_type.name}) at {transform}")
        return actor

    def destroy_all(self):
        for actor, spec in zip(reversed(self._actors), reversed(self._specs)):
            if actor is not None and actor.is_alive:
                actor.destroy()
                print(f"[ObstacleSpawner] destroyed {spec.label}")
        self._actors.clear()
        self._specs.clear()

    @property
    def actors(self):
        return list(self._actors)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.destroy_all()
        return False
