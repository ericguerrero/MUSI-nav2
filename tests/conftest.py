"""Make the robot-side power log modules importable by the unit tests.

They are plain scripts deployed to the robot, not an installed package.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1] / "robot" / "powerlog"))
