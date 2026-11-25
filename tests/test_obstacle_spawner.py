"""
Unit tests for grp planning/obstacle_spawner.py.

obstacle_spawner.py does `import carla`, which normally requires a full
CARLA install. To keep these tests runnable in CI / any dev environment
without CARLA, we inject a minimal fake `carla` module into sys.modules
before importing obstacle_spawner. The fake only implements the handful
of attributes obstacle_spawner.py actually touches (Location, Rotation,
Transform) plus fake World/BlueprintLibrary/Actor doubles used by the
tests themselves.

Run with:  pytest tests/test_obstacle_spawner.py -v
"""

import sys
import types
from pathlib import Path

import pytest

# ---------------------------------------------------------------------------
# Fake `carla` module - installed into sys.modules before obstacle_spawner
# is imported, so `import carla` inside it resolves to this stand-in.
# ---------------------------------------------------------------------------

class FakeLocation:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z

    def __eq__(self, other):
        return (self.x, self.y, self.z) == (other.x, other.y, other.z)

    def __repr__(self):
        return f"Location({self.x}, {self.y}, {self.z})"


class FakeRotation:
    def __init__(self, pitch=0.0, yaw=0.0, roll=0.0):
        self.pitch, self.yaw, self.roll = pitch, yaw, roll

    def __eq__(self, other):
        return (self.pitch, self.yaw, self.roll) == (other.pitch, other.yaw, other.roll)


class FakeTransform:
    def __init__(self, location=None, rotation=None):
        self.location = location or FakeLocation()
        self.rotation = rotation or FakeRotation()

    def __repr__(self):
        return f"Transform({self.location}, yaw={self.rotation.yaw})"


fake_carla = types.ModuleType("carla")
fake_carla.Location = FakeLocation
fake_carla.Rotation = FakeRotation
fake_carla.Transform = FakeTransform
sys.modules["carla"] = fake_carla

# Make "grp planning" importable (its dirname has a space, so it can't be a
# normal package import - add it to sys.path like the test_scenarios files do).
sys.path.append(str(Path(__file__).resolve().parents[1] / "grp planning"))

from obstacle_spawner import (  # noqa: E402
    ObstacleSpawner,
    ObstacleSpec,
    ObstacleType,
    DEFAULT_OFFSETS,
)


# ---------------------------------------------------------------------------
# Fake CARLA world/actor doubles used only by these tests.
# ---------------------------------------------------------------------------

class FakeActor:
    def __init__(self, blueprint_id, transform):
        self.blueprint_id = blueprint_id
        self.transform = transform
        self.is_alive = True
        self.autopilot = False

    def set_autopilot(self, enabled):
        self.autopilot = enabled

    def destroy(self):
        self.is_alive = False


class FakeBlueprint:
    def __init__(self, bp_id):
        self.id = bp_id


class FakeBlueprintLibrary:
    """filter() returns [] for the 'no.match' filter to test failure paths,
    and a single fake blueprint for everything else."""

    def filter(self, pattern):
        if pattern == "no.match":
            return []
        return [FakeBlueprint(pattern)]


class FakeWorld:
    def __init__(self, fail_to_spawn=False):
        self._blueprint_library = FakeBlueprintLibrary()
        self._fail_to_spawn = fail_to_spawn
        self.spawned = []

    def get_blueprint_library(self):
        return self._blueprint_library

    def try_spawn_actor(self, blueprint, transform):
        if self._fail_to_spawn:
            return None
        actor = FakeActor(blueprint.id, transform)
        self.spawned.append(actor)
        return actor


@pytest.fixture
def reference():
    return FakeTransform(FakeLocation(100.0, 200.0, 0.0), FakeRotation(yaw=90.0))


@pytest.fixture
def world():
    return FakeWorld()


# ---------------------------------------------------------------------------
# Taxonomy sanity checks
# ---------------------------------------------------------------------------

def test_default_offsets_cover_offset_based_types():
    """Every type that's meant to be placed relative to a reference
    transform should have a documented default offset, so callers aren't
    silently placed at (0, 0, 0)."""
    offset_based_types = {
        ObstacleType.LANE_BLOCKER,
        ObstacleType.ADJACENT_LANE_BLOCKER,
        ObstacleType.INTERSECTION_BLOCKER,
        ObstacleType.FOLLOWING_TOO_CLOSE,
        ObstacleType.OPPOSING_LANE,
        ObstacleType.JUNCTION_CROSS_TRAFFIC,
    }
    assert offset_based_types.issubset(DEFAULT_OFFSETS.keys())


# ---------------------------------------------------------------------------
# spawn()
# ---------------------------------------------------------------------------

def test_spawn_uses_default_offset(world, reference):
    spawner = ObstacleSpawner(world, reference)
    actor = spawner.spawn(ObstacleType.LANE_BLOCKER)

    expected_offset = DEFAULT_OFFSETS[ObstacleType.LANE_BLOCKER]
    assert actor.transform.location.x == reference.location.x + expected_offset.x
    assert actor.transform.location.y == reference.location.y + expected_offset.y
    assert actor in spawner.actors


def test_spawn_with_explicit_offset_overrides_default(world, reference):
    spawner = ObstacleSpawner(world, reference)
    custom_offset = FakeLocation(-1.0, -2.0, 0.0)
    actor = spawner.spawn(ObstacleType.LANE_BLOCKER, offset=custom_offset)

    assert actor.transform.location.x == reference.location.x - 1.0
    assert actor.transform.location.y == reference.location.y - 2.0


def test_spawn_with_reference_override(world, reference):
    """junction_test.py / yellow_line.py-style usage: offset from a
    different transform than the one passed to the constructor."""
    spawner = ObstacleSpawner(world, reference)
    other_reference = FakeTransform(FakeLocation(0.0, 0.0, 0.0))

    actor = spawner.spawn(ObstacleType.JUNCTION_CROSS_TRAFFIC, reference=other_reference)

    expected_offset = DEFAULT_OFFSETS[ObstacleType.JUNCTION_CROSS_TRAFFIC]
    assert actor.transform.location.x == expected_offset.x
    assert actor.transform.location.y == expected_offset.y


def test_spawn_uses_reference_rotation_when_unset(world, reference):
    spawner = ObstacleSpawner(world, reference)
    actor = spawner.spawn(ObstacleType.LANE_BLOCKER)
    assert actor.transform.rotation.yaw == reference.rotation.yaw


def test_spawn_rotation_override(world, reference):
    spawner = ObstacleSpawner(world, reference)
    custom_rotation = FakeRotation(yaw=180.0)
    actor = spawner.spawn(ObstacleType.LANE_BLOCKER, rotation=custom_rotation)
    assert actor.transform.rotation.yaw == 180.0


def test_spawn_autopilot_flag_is_applied(world, reference):
    spawner = ObstacleSpawner(world, reference)
    actor = spawner.spawn(ObstacleType.MOVING_OBSTACLE, autopilot=True)
    assert actor.autopilot is True


def test_spawn_without_autopilot_flag_stays_static(world, reference):
    """Regression guard for the moving_obs.py bug: MOVING_OBSTACLE must be
    explicitly told to move. Silently defaulting to static would recreate
    the original bug where a 'moving' obstacle never actually moved."""
    spawner = ObstacleSpawner(world, reference)
    actor = spawner.spawn(ObstacleType.MOVING_OBSTACLE)  # autopilot not requested
    assert actor.autopilot is False


def test_spawn_at_autopilot_flag_is_applied(world, reference):
    spawner = ObstacleSpawner(world, reference)
    absolute = FakeTransform(FakeLocation(5.0, 5.0, 0.0))
    actor = spawner.spawn_at(absolute, obstacle_type=ObstacleType.MOVING_OBSTACLE, autopilot=True)
    assert actor.autopilot is True


def test_spawn_failure_raises_runtime_error(reference):
    failing_world = FakeWorld(fail_to_spawn=True)
    spawner = ObstacleSpawner(failing_world, reference)
    with pytest.raises(RuntimeError):
        spawner.spawn(ObstacleType.LANE_BLOCKER)


def test_spawn_unknown_blueprint_filter_raises_value_error(world, reference):
    spawner = ObstacleSpawner(world, reference)
    with pytest.raises(ValueError):
        spawner.spawn(ObstacleType.LANE_BLOCKER, blueprint_filter="no.match")


# ---------------------------------------------------------------------------
# spawn_at()
# ---------------------------------------------------------------------------

def test_spawn_at_uses_absolute_transform(world, reference):
    spawner = ObstacleSpawner(world, reference)
    absolute = FakeTransform(FakeLocation(5.0, 5.0, 0.0))

    actor = spawner.spawn_at(absolute, obstacle_type=ObstacleType.OPPOSING_LANE)

    assert actor.transform is absolute
    assert actor in spawner.actors


# ---------------------------------------------------------------------------
# spawn_batch()
# ---------------------------------------------------------------------------

def test_spawn_batch_spawns_all_specs(world, reference):
    spawner = ObstacleSpawner(world, reference)
    specs = [
        ObstacleSpec(obstacle_type=ObstacleType.LANE_BLOCKER, offset=FakeLocation(-20, 0, 0)),
        ObstacleSpec(obstacle_type=ObstacleType.ADJACENT_LANE_BLOCKER, offset=FakeLocation(-20, 4, 0)),
    ]
    actors = spawner.spawn_batch(specs)
    assert len(actors) == 2
    assert len(spawner.actors) == 2


# ---------------------------------------------------------------------------
# destroy_all() / context manager
# ---------------------------------------------------------------------------

def test_destroy_all_destroys_every_tracked_actor(world, reference):
    spawner = ObstacleSpawner(world, reference)
    a1 = spawner.spawn(ObstacleType.LANE_BLOCKER)
    a2 = spawner.spawn(ObstacleType.ADJACENT_LANE_BLOCKER)

    spawner.destroy_all()

    assert a1.is_alive is False
    assert a2.is_alive is False
    assert spawner.actors == []


def test_destroy_all_is_safe_to_call_when_nothing_spawned(world, reference):
    spawner = ObstacleSpawner(world, reference)
    spawner.destroy_all()  # should not raise
    assert spawner.actors == []


def test_context_manager_destroys_on_normal_exit(world, reference):
    with ObstacleSpawner(world, reference) as spawner:
        actor = spawner.spawn(ObstacleType.LANE_BLOCKER)
    assert actor.is_alive is False


def test_context_manager_destroys_on_exception(world, reference):
    actor = None
    with pytest.raises(ValueError):
        with ObstacleSpawner(world, reference) as spawner:
            actor = spawner.spawn(ObstacleType.LANE_BLOCKER)
            raise ValueError("boom")
    assert actor.is_alive is False


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
