#!usr/bin/env python3

"""

"""

# IMPORTS
from math import hypot


class BaseArm:
    """
    Base arm class for getter, setters and basic functions.
    """
    def __init__(self, X=0.3, Y=0., Z=0.5):
        # Home of the Robot
        self._homeX = 0.3
        self._homeY = 0.
        self._homeZ = 0.5
        self._homeRX = 1.
        self._homeRY = 0.
        self._homeRZ = 0.
        self._homeRW = 0.

        # Desired point
        self._goalX = None
        self._goalY = None
        self._goalZ = None
        self._goalRX = None
        self._goalRY = None
        self._goalRZ = None
        self._goalRW = None

        # Area Control Vars
        self._reach = 0.855
        self._base = 0.025

        # Set the home
        self.home = (X, Y, Z)

    #### GETTER/SETTER AREA ####

    @property
    def homeX(self):
        return self._homeX

    @homeX.setter
    def homeX(self, value):
        if not self.check_pose((value, self._homeY, self._homeZ)):
            return
        self._homeX = value

    @property
    def homeY(self):
        return self._homeY

    @homeY.setter
    def homeY(self, value):
        if not self.check_pose((self._homeX, value, self._homeZ)):
            return
        self._homeY = value

    @property
    def homeZ(self):
        return self._homeZ

    @homeZ.setter
    def homeZ(self, z):
        if not self.check_pose((self._homeX, self._homeY, z)):
            return
        self._homeZ = z

    @property
    def homeRX(self):
        return self._homeRX

    @homeRX.setter
    def homeRX(self, rx):
        self._homeRX = rx

    @property
    def homeRY(self):
        return self._homeRZ

    @homeRY.setter
    def homeRY(self, value):
        self._homeRY = value

    @property
    def homeRZ(self):
        return self._homeRZ

    @homeRZ.setter
    def homeRZ(self, value):
        self._homeRZ = value

    @property
    def homeRW(self):
        return self._homeRW

    @homeRW.setter
    def homeRW(self, value):
        self._homeZ = value

    @property
    def home(self):
        return (self._homeX, self._homeY, self._homeZ,
                self._homeRX, self._homeRY, self._homeRZ, self._homeRW)

    @home.setter
    def home(self, coords):
        if not self.check_pose(coords):
            return
        if len(coords) == 3:
            self._homeX = coords[0]
            self._homeY = coords[1]
            self._homeZ = coords[2]
        elif len(coords) == 7:
            self._homeX = coords[0]
            self._homeY = coords[1]
            self._homeZ = coords[2]
            self._homeRX = coords[3]
            self._homeRY = coords[4]
            self._homeRZ = coords[5]
            self._homeRW = coords[6]

    @property
    def reach(self):
        return self._reach

    @reach.setter
    def reach(self, value):
        self._reach = value

    @property
    def base(self):
        return self._base

    @base.setter
    def base(self, value):
        self._base = value

    #### METHOD AREA ####

    def check_pose(self, coords):
        """
        Used to check if given coords are valid.
        :Param coords (tuple/list): coords to be checked.
        """
        if not (isinstance(coords, list) or isinstance(coords, tuple)):
            raise TypeError('Coords need to be of type(list/tuple)')
            return False

        # check to see if the coords are of valid type.
        types = [float, int]
        if not all([True if (type(i) in types) else False for i in coords]):
            raise TypeError(f"All coords need to be Float or Int.")
            return False

        if len(coords) not in [3, 7]:
            raise ValueError("Coords len of {len(coords)} not valid, "
                             + "shoule be at least 3 (or 7)!")
            return False


        # Check to see if z is negative
        if coords[2] < self._base:
            raise ValueError(f"Z below base of {self._base} which is"
                             + " invalid.")
            return False

        # check to see if x, y is in the 0.855 reach of the panda.
        if abs(hypot(coords[0], coords[1])) > self._reach:
            raise ValueError("Home of reach {hypot(coord[0], coord[1])} is"
                             + f" outside of {self._reach}")
            return False

        return True
