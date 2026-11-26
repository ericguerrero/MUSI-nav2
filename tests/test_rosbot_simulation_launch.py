"""Unit tests for the pure helpers of rosbot_simulation.launch.py.

Runs inside the nav2_student:humble container. See tests/run_unit_tests.sh.
"""

import importlib.util

import pytest

LAUNCH_FILE = "/ros2_ws/src/rosbot_simulation.launch.py"

# The file name holds dots, so it cannot be imported by module name.
_spec = importlib.util.spec_from_file_location("rosbot_simulation_launch", LAUNCH_FILE)
assert _spec is not None and _spec.loader is not None
launch = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(launch)


@pytest.fixture
def worlds_dir(tmp_path):
    for name in ("husarion_world", "husarion_office"):
        (tmp_path / f"{name}.sdf").write_text("<sdf/>")
    (tmp_path / "notes.txt").write_text("not a world")
    return tmp_path


def test_world_name_resolves_to_sdf_in_worlds_dir(worlds_dir):
    assert launch.resolve_world("husarion_office", str(worlds_dir)) == str(
        worlds_dir / "husarion_office.sdf"
    )


def test_absolute_sdf_path_is_used_as_given(worlds_dir, tmp_path_factory):
    own = tmp_path_factory.mktemp("own") / "landmark_world.sdf"
    own.write_text("<sdf/>")
    assert launch.resolve_world(str(own), str(worlds_dir)) == str(own)


def test_unknown_world_names_the_available_ones(worlds_dir):
    with pytest.raises(launch.WorldNotFoundError) as err:
        launch.resolve_world("turtlebot3_world", str(worlds_dir))
    message = str(err.value)
    assert "turtlebot3_world" in message
    assert "husarion_office, husarion_world" in message
    assert "notes" not in message


def test_missing_absolute_path_is_an_error(worlds_dir):
    with pytest.raises(launch.WorldNotFoundError):
        launch.resolve_world("/nowhere/world.sdf", str(worlds_dir))


@pytest.mark.parametrize(
    "value, expected",
    [("true", "True"), ("True", "True"), ("false", "False"), ("FALSE", "False")],
)
def test_flags_become_the_capitalised_form_husarion_expects(value, expected):
    assert launch.gz_flag(value) == expected


def test_flag_other_than_true_or_false_is_rejected():
    with pytest.raises(launch.InvalidFlagError):
        launch.gz_flag("yes")
