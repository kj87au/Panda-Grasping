#!usr/bin/env python3

import rospy
import actionlib
import time

from rv_msgs.msg import (MoveToPoseAction, MoveToPoseGoal,
                         ServoToPoseAction, ServoToPoseGoal,
                         MoveToNamedPoseAction, MoveToNamedPoseGoal)
from geometry_msgs.msg import PoseStamped
from base.robot_base import BaseArm


class FrankaPanda(BaseArm):
    """
    Could make a Panda class to allow for higher level control?
    """
    def __init__(self, name, *args, **kwargs):
        super().__init__()
        rospy.init_node(f"Panda_class_{name}")
        self.client = {}
        self.client["pose"] = actionlib.SimpleActionClient('/arm/cartesian/pose',
                                                           MoveToPoseAction)
        self.client["pose"].wait_for_server()
        self.client["servo"] = actionlib.SimpleActionClient('/arm/cartesian/servo_pose',
                                                            ServoToPoseAction)
        self.client["servo"].wait_for_server()
        self.client["named"] = actionlib.SimpleActionClient('/arm/cartesian/named_pose',
                                                            MoveToNamedPoseAction)
        self.client["named"].wait_for_server()

    def MoveToPose(self, coord, ref="base", speed=None, wait=True, method="pose"):
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
        target.pose.orientation.y = 0.00
        target.pose.orientation.z = 0.00
        target.pose.orientation.w = 0.00

        # Create goal from target pose
        if method in ["pose", "servo"]:
            if method == "pose":
                if speed is None:
                    goal = MoveToPoseGoal(goal_pose=target)
                else:
                    goal = MoveToPoseGoal(goal_pose=target, speed=speed)
            else:
                goal = ServoToPoseGoal(stamped_pose=target, scaling=.5)
            # Send goal and wait for it to finish
            return self.client[method].send_goal_and_wait(goal)
        return False

    def ServoToPose(self, coord, ref="base", speed=None, wait=True):
        """

        """
        return self.MoveToPose(coord, ref, speed, wait, method="servo")

    def NamedToPose(self, name):
        """

        """
        return self.client["named"].send_goal_and_wait(MoveToNamedPoseGoal(pose_name=name))

    def GraspFromBase(self):
        """

        """
        pass

    def GraspFromCamera(self):
        """

        """
        pass

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
        target.pose.orientation.y = 0.00
        target.pose.orientation.z = 0.00
        target.pose.orientation.w = 0.00

        goal = MoveToPoseGoal(goal_pose=target)

        # Send goal and wait for it to finish
        return self.client["pose"].send_goal_and_wait(goal)


if __name__ == "__main__":
    # main()
    panda = FrankaPanda("prac")
    panda.MoveToPose((0.5, 0.3, 0.2))
    time.sleep(1)
    panda.MoveHome()
    time.sleep(1)
    panda.ServoToPose((0.5, -0.2, 0.2))
    time.sleep(1)
    panda.ServoToPose((0.7, 0.2, 0.2))
    time.sleep(1)
    panda.NamedToPose('grasp_home')
