#!usr/bin/env python3

import rospy
import actionlib
import time

from rv_msgs.msg import MoveToPoseAction, MoveToPoseGoal
from geometry_msgs.msg import PoseStamped
from base.robot_base import BaseArm


class FrankaPanda(BaseArm):
    """
    Could make a Panda class to allow for higher level control?
    """
    def __init__(self, name, *args, **kwargs):
        super().__init__()
        rospy.init_node(f"Panda_class_{name}")
        self.pose_client = actionlib.SimpleActionClient('/arm/cartesian/pose', MoveToPoseAction)
        self.pose_client.wait_for_server()

    def MoveToPose(self, coord, ref="base", speed=None, wait=True):
        """
        Simplified move to pose.

        Param x (float): Cartesian x in Meters.
        Param y (float): Cartesian y in Meters.
        Param z (float): Cartesian z in Meters.
        Param ref (string): Reference frame to use. Currently "base" or "EE"
                            (End Effector).
        Param wait (bool): Wait for arm to move.
        Return (bool): True if move was successful else false.
        """
        if not self.check_pose(coord):
            return False

        # Create a target pose
        target = PoseStamped()
        if ref == "base":
            target.header.frame_id = 'panda_link0'
        elif ref == "EE":
            target.header.frame_id = 'panda_EE'
        else:
            print(f"No frame named {ref}, Please try again!")
            return False
    
        # Populate with target position/orientation (READY POSE)
        target.pose.position.x = coord[0] 
        target.pose.position.y = coord[1]
        target.pose.position.z = coord[2]

        target.pose.orientation.x = -1.00
        target.pose.orientation.y =  0.00
        target.pose.orientation.z =  0.00
        target.pose.orientation.w =  0.00

        # Create goal from target pose
        if speed == None:
            goal = MoveToPoseGoal(goal_pose=target)
        else:
            goal = MoveToPoseGoal(goal_pose=target, speed=speed)

        # Send goal and wait for it to finish
        return self.pose_client.send_goal_and_wait(goal)

    def MoveHome(self):
        """
	Returns the robot to the Home position.
        """
        # Create a target pose
        target = PoseStamped()

        target.header.frame_id = 'panda_link0'

        home = self.home
        target.pose.position.x = home[0]
        target.pose.position.y = home[1]
        target.pose.position.z = home[2]

        target.pose.orientation.x = -1.00
        target.pose.orientation.y =  0.00
        target.pose.orientation.z =  0.00
        target.pose.orientation.w =  0.00

        goal = MoveToPoseGoal(goal_pose=target)

        # Send goal and wait for it to finish
        return self.pose_client.send_goal_and_wait(goal)


if __name__ == "__main__":
    # main()
    panda = FrankaPanda("prac")
    panda.MoveToPose((0.5, 0, 0.6))
    time.sleep(1)
    panda.MoveHome()
    time.sleep(1)
    panda.MoveToPose((0.5, 0, 0.2))
    time.sleep(1)
    panda.MoveHome()
