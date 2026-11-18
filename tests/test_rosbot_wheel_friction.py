"""Wheel contact friction in the simulated ROSbot 2R URDF.

Husarion's wheel macro gives every wheel the anisotropic friction Gazebo uses to
fake mecanum rollers (mu along a 45 degree direction, mu2 = 0 across it), also
for the ordinary wheels. A skid-steer robot built that way never scrubs
sideways, so turning costs nothing and odometry matches the body exactly.
patches/rosbot_ros/0002-skid-steer-friction.patch gives ordinary wheels
isotropic rubber friction and leaves the mecanum wheels as they were.

Runs inside the nav2_student:humble container. See tests/run_unit_tests.sh.
"""

import subprocess
import xml.etree.ElementTree as ET

import pytest

UNDERLAY = "/opt/rosbot_ws/install/local_setup.bash"
URDF ="/opt/rosbot_ws/install/rosbot_description/share/rosbot_description/urdf/rosbot.urdf.xacro"
WHEELS = ("fl", "fr", "rl", "rr")


def wheel_friction(mecanum):
    """Return {side: (mu, mu2, fdir1 or None)} from the URDF the simulation spawns."""
    # The unit test runner sources /opt/ros only; xacro's $(find) needs the
    # ROSbot underlay as well.
    command = (f"source {UNDERLAY} && xacro {URDF} use_sim:=true "
               f"mecanum:={str(mecanum).lower()}")
    urdf = subprocess.run(
        ["bash", "-c", command], check=True, capture_output=True, text=True,
    ).stdout
    root = ET.fromstring(urdf)
    friction = {}
    for gazebo in root.iter("gazebo"):
        side = gazebo.get("reference", "").removesuffix("_wheel_link")
        ode = gazebo.find("collision/surface/friction/ode")
        if side in WHEELS and ode is not None:
            fdir = ode.find("fdir1")
            friction[side] = (float(ode.findtext("mu", "nan")), float(ode.findtext("mu2", "nan")),
                              None if fdir is None else (fdir.text or "").strip())
    return friction


@pytest.fixture(scope="module")
def skid():
    return wheel_friction(mecanum=False)


@pytest.fixture(scope="module")
def mecanum():
    return wheel_friction(mecanum=True)


def test_every_wheel_has_a_friction_block(skid, mecanum):
    assert sorted(skid) == sorted(WHEELS)
    assert sorted(mecanum) == sorted(WHEELS)


def test_skid_wheels_grip_equally_in_every_direction(skid):
    for side, (mu, mu2, fdir) in skid.items():
        assert mu == mu2, f"{side}: mu {mu} != mu2 {mu2}"
        assert mu > 0.5, f"{side}: mu {mu} is too low for rubber"
        assert fdir is None, f"{side}: skid wheel has a roller direction {fdir}"


def test_mecanum_wheels_keep_their_roller_friction(mecanum):
    for side, (mu, mu2, fdir) in mecanum.items():
        assert mu == 1.0 and mu2 == 0.0, f"{side}: mu {mu}, mu2 {mu2}"
        assert fdir is not None, f"{side}: mecanum wheel lost its roller direction"
