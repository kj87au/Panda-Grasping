#!usr/bin/env python3 

"""
This houses the arg parser for the project.
"""

# IMPORTS
import argparse

# GLOBALS
DESCRIPTION = """Welcome to Grasping Algorithm Test Suite (GATS). This program
                 is designed to allow for testing of current robotic grasping
                 algorithms on a single system without minimal dependancy
                 conflict using Docker containerisation."""
NETWORKS = ["giga",
            "contact",
            "gg-convnet"]


class GATSParser(argparse.ArgumentParser):
    """
    TODO: Comment
    """
    def __init__(self):
        super().__init__(description=DESCRIPTION)
        self.get_args()    
    
    def get_args(self):
        self.add_argument('-n',
                          '--network',
                          default="giga",
                          help="Used to choose the network which will be used"
                                + f" to predict the Grasp. {NETWORKS}",
                          type=str)
        self.add_argument('-p',
                          '--pose',
                          help="Input the desired pose to view the scene."
                               + "'x,y,x,rot x,rot y,rot z' example: '0.5,"
                               + "0.0,0.7,-1.0,0.0,0.0' which is the default"
                               + " position",
                          type=str)

    def rtn_unpacked(self):
        args = self.parse_args()
        print(args)
        # need to split pose here.
        return None  # vars(**args)


if __name__ == "__main__":
    pass
