#!usr/bin/env python3

import rospy
import actionlib
import time

import armer_msgs.msg
from armer_msgs.msg import (MoveToPoseAction, MoveToPoseGoal)
from franka_gripper.msg import (HomingAction, HomingGoal,
                                MoveGoal, MoveAction)
from geometry_msgs.msg import PoseStamped
from base.robot_base import BaseArm
import rospy



class FrankaPanda(BaseArm):
    """
    Could make a Panda class tofrom geometry_msgs.msg import PoseStamped
    allow for higher level control?
    """
    def __init__(self, name, arm_type="panda", home_arm=True, home_gripper=False):
        super().__init__()
        rospy.init_node(f"{arm_type}_class_{name}")

        print("Initalising Arm.")
        # get our client and wait for it to load up
        self.arm = actionlib.SimpleActionClient('/arm/cartesian/pose',
                                                   MoveToPoseAction)
        self.arm.wait_for_server()
        print("Arm started!")

        if home_arm:
            print("Homing Arm")
            self.MoveHome()
            print("Arm Homed!")

        if home_gripper:
            print("Homing Gripper")
            self.gripper = actionlib.SimpleActionClient('/franka_gripper/homing',
                                                        HomingAction)
            self.gripper.wait_for_server()
            goal = HomingGoal()
            self.gripper.send_goal_and_wait(goal)
            print("Gripper Homed!")

        print("Initalising Gripper.")
        self.gripper = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
        self.gripper.wait_for_server()
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
        return self.arm.send_goal_and_wait(goal)

    def GraspFromBase(self, goal):
        """

        """
        raise NotImplementedError
        # Move to position close to goal with rotation and orenation correct

        # Move forward 5-10cm into grasp position

        # Close Grasper

        # Move Object up

        # Place in goal location

    def MoveHome(self):
        """
        Returns the robot to the Home position.
        """
        # Create a target pose
        target = PoseStamped()

        target.header.frame_id = 'panda_link0'

        target.pose.position.x = self.homeX
        target.pose.position.y = self.homeY
        target.pose.position.z = self.homeZ

        target.pose.orientation.x = self.homeRX 
        target.pose.orientation.y = self.homeRY
        target.pose.orientation.z = self.homeRZ
        target.pose.orientation.w = self.homeRW

        goal = MoveToPoseGoal()
        goal.pose_stamped = target

        # Send goal and wait for it to finish
        self.arm.send_goal_and_wait(goal)


def main():
    
    panda = FrankaPanda("prac")

    print("Moving to pose (0.5, 0.3, 0.2)")
    panda.MoveToPose((0.5, 0.3, 0.2))
    # Settle and then get image.
    time.sleep(0.5)

    print("Home Time!")
    panda.MoveHome()


if __name__ == "__main__":
    main()
