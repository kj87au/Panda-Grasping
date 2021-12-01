#!usr/bin/env python3

"""

"""

# IMPORTS
from math import hypot


class BaseArm:
    """
    Base arm class for getter, setters and basic functions.
    """
    def __init__(self):
        self._hx = 0.3
        self._hy = 0.
        self._hz = 0.5
        self._reach = 0.855

    @property
    def hx(self):
        return self._hx

    @hx.setter
    def hx(self, x):
        if not self.check_pose((x, self._hy, self._hz)):
            return
        self._hx = x

    @property
    def hy(self):
        return self._hy

    @hy.setter
    def hy(self, y):
        if not self.check_pose((self._hx, self._hy, z)):
            return

        self._hx = y

    @property
    def hz(self):
        return self._hz

    @hz.setter
    def hz(self, z):
        if not self.check_pose((self._hx, self._hy, z)):
            return
        self._hz = z

    @property
    def reach(self):
        return self._reach

    @reach.setter
    def reach(self, value):
        self._reach = value

    @property
    def home(self):
        return (self._hx, self._hy, self.hz)

    @home.setter
    def home(self, coords):
        if not self.check_pose(coords):
            return
        self._hx = coord[0]
        self._hy = coord[1]
        self._hz = coord[2]

    def check_pose(self, coord):
        """
        Used to check if given coords are valid.
        :Param coords (tuple/list): coords to be checked.
        """
        if len(coord) != 3:
            print(f"ERROR: Coords len of {len(coords)} not valid, " +
                  "shoule be 3!")
            return False

        # check to see if the coords are of valid type.
        types = [float, int]
        if not all([True if (type(i) in types) else False for i in coord]):
            print(f"Error: All coords need to be Float or Int.")
            return False

        # check to see if x, y is in the 0.855 reach of the panda.
        if abs(hypot(coord[0], coord[1])) > 0.855:
            print(f"ERROR: Value {hypot(coord[0], coord[1])} is outside" +
                  "Pandas Default reach of 0.85m")
            return False

        return True
