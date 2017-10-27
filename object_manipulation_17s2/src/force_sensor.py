#!/usr/bin/env python
import math
import os
import sys
import time
from geometry_msgs.msg import WrenchStamped
import rospy
import trajectory_msgs.msg


import actionlib
from actionlib_msgs.msg import GoalStatus

from tmc_control_msgs.msg import (
    GripperApplyEffortAction,
    GripperApplyEffortGoal
)

FT_SENSOR_TOPIC = '/hsrb/wrist_wrench/raw'
GRASP_ACTION = '/hsrb/gripper_controller/grasp'
GRIPPER_COMMAND_TOPIC = '/hsrb/gripper_controller/command'
GRIPPER_OPEN_ANGLE = 1.239
GRIPPER_CLOSE_ANGLE = -0.105

class Force_Sensor_Capture(object):

    def __init__(self):
        self.init = False
        self.init_force_x = 0
        self.init_force_z = 0
        self.init_force_y = 0
        self.force_data_x = 0.0
        self.force_data_y = 0.0
        self.force_data_z = 0.0
        self.threshold_x = 2.0
        self.threshold_z = 2.0
        self.threshold_y = 2.0
        self.holding_object = False

        # ROS gripper publisher
        self.gripper_pub = rospy.Publisher(GRIPPER_COMMAND_TOPIC,
                        trajectory_msgs.msg.JointTrajectory, queue_size=10)
        # Grasping action library
        self._gripper_control_client = actionlib.SimpleActionClient(
            GRASP_ACTION, GripperApplyEffortAction)

    # Callback function checks if any force is being applied to the grippers or the touch sensor
    # at the palm of the robot's hand
    def force_sensor_cb(self, data):  
        if self.holding_object:
            self._wrist_wrench_sub.unregister()
            return

        self.force_data_x = data.wrench.force.x
        self.force_data_y = data.wrench.force.y
        self.force_data_z = data.wrench.force.z
        # Initialise initial force values to compare againt
        if self.init == False:
            self.init_force_x = self.force_data_x
            self.init_force_y = self.force_data_y
            self.init_force_z = self.force_data_z
            self.init = True
            return
        # Check if anything is touching the right gripper
        if self.force_data_y > self.init_force_y + self.threshold_y:
            self.grasp(-0.1)
            self.holding_object = True
            rospy.sleep(1)
            return
        # Check if anything is touching the left gripper
        if self.force_data_y < self.init_force_y - self.threshold_y:
            self.grasp(-0.1)
            self.holding_object = True
            rospy.sleep(1)
            return
        # Check if the object has touched the palm of the robot hand
        if self.force_data_z > self.init_force_z + self.threshold_z:
            self.grasp(-0.1)
            self.holding_object = True
            rospy.sleep(1)
        # Otherwise keep the gripper open so that objects can be sensed by touch
        else:
            self.open_gripper()
            self.holding_object = False
            rospy.sleep(1)

    def sub_to_force_sensor(self):
        self._wrist_wrench_sub = rospy.Subscriber(FT_SENSOR_TOPIC, WrenchStamped, 
            self.force_sensor_cb, queue_size=1)

    def unsub_to_force_sensor(self):
        self._wrist_wrench_sub.unregister()

    # Grasp the object by applying force until the gripper detects that the object is in place
    def grasp(self, effort):
        goal = GripperApplyEffortGoal()
        goal.effort = effort
        # Send message to the action server
        if (self._gripper_control_client.send_goal_and_wait(goal) ==
                GoalStatus.SUCCEEDED):
            return True
        else:
            return False

    # Functions to open and close the gripper
    def open_gripper(self):
        self.move_gripper(GRIPPER_OPEN_ANGLE)

    def close_gripper(self):
        self.move_gripper(GRIPPER_CLOSE_ANGLE)

    # Move the gripper by publishing a joint trajectory message with the hand motor joint
    def move_gripper(self, hm_joint):
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["hand_motor_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [hm_joint]
        p.velocities = [0.3]
        p.effort = [0.1]
        p.time_from_start = rospy.Time(1)
        # Update joint
        self.hm_joint = hm_joint
        traj.points = [p]
        # publish ROS message
        self.gripper_pub.publish(traj)

def main(args):
    rospy.init_node('force_capture')
    fc = Force_Sensor_Capture()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)