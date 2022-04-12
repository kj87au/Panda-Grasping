#!usr/bin/env python3 

"""
Used to test base settings
"""

# IMPORTS
import pytest
from src.base.robot_base import BaseArm

# MAIN AREA
class TestBase:
    """
    Used to test BaseArm 
    """
    arm = BaseArm()

    def reset_arm(self):
        self.arm = BaseArm()

    def test_home(self):
        h = self.arm.home
        self.arm.homeX = 0.4
        assert h != self.arm.home

    def test_setX(self):
        self.arm.homeX = 0.
        assert self.arm.homeX == 0.

    def test_setY(self):
        self.arm.homeY = 0.
        assert self.arm.homeY == 0.

    def test_setZ(self):
        self.arm.homeZ = 0.3
        assert self.arm.homeZ == 0.3

    def test_setZ_BAD(self):
        z = self.arm.homeZ
        with pytest.raises(Exception):
            self.arm.homeZ = -0.4
        assert z == self.arm.homeZ

    def test_reset_Base(self):
        self.arm.base = -0.2
        self.arm.homeZ = -0.1
        assert self.arm.homeZ == -0.1
        self.reset_arm()

    def test_reach_GOOD(self):
        self.arm.home = (0.4, 0.2, 0.5)
        assert self.arm.home[:3] == (0.4, 0.2, 0.5)

    def test_reach_BAD(self):
        h = self.arm.home
        with pytest.raises(Exception):
            self.arm.home = (0.6, 0.65, 0.4)





if __name__ == "__main__":
    pass

