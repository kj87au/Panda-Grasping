#!usr/bin/env python3

import rospy
import actionlib
import time

import armer_msgs.msg
from armer_msgs.msg import (MoveToPoseAction, MoveToPoseGoal)
from franka_gripper.msg import (HomingAction, HomingGoal,
                                MoveGoal, MoveAction)
from geometry_msgs.msg import PoseStamped
import rospy


class FrankaPanda:
    """
    Could make a Panda class tofrom geometry_msgs.msg import PoseStamped
    allow for higher level control?
    """
    def __init__(self, name):
        rospy.init_node(f"Panda_class_{name}")

        print("Starting Arm.")
        # Final Home setting
        self.home = [0.3,
                     0.0,
                     0.5]

        # get our client and wait for it to load up
        self.client = actionlib.SimpleActionClient('/arm/cartesian/pose',
                                                   MoveToPoseAction)
        self.client.wait_for_server()
        print("Arm started!")
        
        print("Homing Gripper.")
        self.gripper = actionlib.SimpleActionClient('/franka_gripper/homing',
                                                    HomingAction)
        self.gripper.wait_for_server()

        goal = HomingGoal()
        self.gripper.send_goal_and_wait(goal)
        print("Gripper Homed!")

        print("Initalising Gripper.")
        client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
        client.wait_for_server()
        print("Gripper Started!")

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
        if speed is None:
            goal = MoveToPoseGoal()
            goal.pose_stamped = target
        else:
            goal = MoveToPoseGoal()
            goal.pose_stamped = target
            goal.speed = speed
        # Send goal and wait for it to finish
        return self.client.send_goal_and_wait(goal)

    def NamedToPose(self, name):
        """

        """
        raise NotImplementedError

    def GraspFromBase(self, goal):
        """

        """
        raise NotImplementedError

    def GraspFromCamera(self):
        """

        """
        raise NotImplementedError

    def MoveHome(self):
        """
        Returns the robot to the Home position.
        """
        # Create a target pose
        target = PoseStamped()

        target.header.frame_id = 'panda_link0'

        target.pose.position.x = self.home[0]
        target.pose.position.y = self.home[1]
        target.pose.position.z = self.home[2]

        target.pose.orientation.x = 1.00
        target.pose.orientation.y = 0.00
        target.pose.orientation.z = 0.00
        target.pose.orientation.w = 0.00

        goal = MoveToPoseGoal()
        goal.pose_stamped = target

        # Send goal and wait for it to finish
        self.client.send_goal_and_wait(goal)


if __name__ == "__main__":
    # main()
    panda = FrankaPanda("prac")
    print("Moving to pose (0.5, 0.3, 0.2)")
    panda.MoveToPose((0.5, 0.3, 0.2))
    time.sleep(1)
    print("Moving to pose (0.5, -0.2, 0.2)")
    panda.MoveToPose((0.5, -0.2, 0.2))
    time.sleep(1)
    print("Moving to pose (0.7, 0.2, 0.2)")
    panda.MoveToPose((0.7, 0.2, 0.2))
    time.sleep(1)
    print("Home Time!")
    panda.MoveHome()
