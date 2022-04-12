#!usr/bin/env python3 

"""
This is the main file for the Grasping Algorithm Test Suite (RATS)
"""

# IMPORTS
from src.ArmController import FrankaPanda
from src.DockerLaunch import ContainerController
from src.CamController import RealSenseCamera
from utils.parser import GATSParser
# GLOBALS


class GATS:
    """

    """
    def __init__(self, network, pose):
        pass

    def capture_image(self, path=None):
        pass


if __name__ == "__main__":
    Parser = GATSParser()
    G = GATS(Parser.rtn_unpacked())
