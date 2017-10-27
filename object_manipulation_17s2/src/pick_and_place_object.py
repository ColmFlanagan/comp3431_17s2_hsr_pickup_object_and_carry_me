#!/usr/bin/env python

# COMP3431 17s2 Object manipulation code to pick up an object and place on a surface
# Robot: Toyota hsr
# Packages: MoveIt and hsr ROS API

import math
import sys
import time
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import rospy
import shape_msgs.msg
from geometry_msgs.msg import WrenchStamped
from tf.transformations import quaternion_from_euler, quaternion_multiply
import trajectory_msgs.msg
from myvis.msg import Object
import force_sensor

# Constants
OBJECT_POS_TOPIC = "Objects_final"
GRIPPER_OPEN_ANGLE = 1.2
GRIPPER_CLOSE_ANGLE = -0.105
# Variables related to the object we are trying to pick up
object_found = False
object_pos = []
object_height = 0
object_width = 0

class Object_Manipulator(object):
    def __init__(self, x, y, z, object_height, object_width):
        # initialize
        moveit_commander.roscpp_initialize(sys.argv)
        # Initialise the groups of joints used by the planner to interact with the world
        self.wait = 1
        self.reference_frame = "odom"       
        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.base = moveit_commander.MoveGroupCommander("base")
        self.gripper = moveit_commander.MoveGroupCommander("gripper")
        self.head = moveit_commander.MoveGroupCommander("head")
        self.whole_body = moveit_commander.MoveGroupCommander("whole_body")
        self.whole_body.allow_replanning(True)
        self.whole_body.set_planning_time(20)
        self.whole_body.set_pose_reference_frame(self.reference_frame)
        self.end_effector = self.whole_body.get_end_effector_link()
        rospy.sleep(1)

        # Subscribe to the gripper publisher
        self.gripper_pub = rospy.Publisher('/hsrb/gripper_controller/command',
            trajectory_msgs.msg.JointTrajectory, queue_size=10)

        # Planning scene
        self.scene = moveit_commander.PlanningSceneInterface()
        # remove all objects from the planning scene
        self.scene.remove_attached_object(self.end_effector)
        self.scene.remove_world_object()
        rospy.sleep(1)

        # Add the object to the planning scene
        self.object_pos = [x, y, z]
        self.object_height = object_height
        self.object_width = object_width
        self.add_box("object",
                     [self.object_width/4, self.object_width/4, self.object_height],
                     [x, y, z])

        # Add the table to the planning scene relative to the object
        # Unfortunately we assume that the object is always placed in a certain area
        # on the table but with a working collision map from MoveIt a table is not required
        tableSize = [0.8, 0.6, self.object_pos[2]-(self.object_height/2)]
        tableZ = z-(self.object_height/2)
        tablePos = [x+0.3, y, tableSize[2]/2]
        self.add_box("table", tableSize, tablePos)

        # Add a stool that sits to the right of the table and will the surface that
        # the robot will place the object on
        stoolSize = [0.4, 0.4, 0.51]
        stoolPos = [self.object_pos[0]+0.1, self.object_pos[1]-0.7, stoolSize[2]/2]
        self.add_box("stool", stoolSize, stoolPos)
        rospy.sleep(1) 

    # Attaches a fake object based on the real object to the gripper in the MoveIt planning scene
    # This is to ensure the robot
    def attach_object(self):
        end_effector_link = self.whole_body.get_end_effector_link()
        # Divide object width by 2 because can't model cylinder as object attached
        object_size = [self.object_width/4, self.object_width/4, self.object_height]
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = end_effector_link
        p.pose.position.x = 0.0
        p.pose.position.y = 0.0
        # Offset object by 3cm so moveit doesn't think there is a collision with the object
        # and its hand/gripper
        p.pose.position.z = object_size[1]/2 + 0.03
        p.pose.orientation.x = 0
        p.pose.orientation.y = 0.707
        p.pose.orientation.z = 0
        p.pose.orientation.w = 0.707
        self.scene.attach_box(end_effector_link, "fake_object", p, object_size)

    # Detaches the fake object we created from the gripper in the MoveIt planning scene
    def detach_object(self):
        end_effector_link = self.whole_body.get_end_effector_link()
        self.scene.remove_attached_object(end_effector_link)

    # Deletes an object from the MoveIt Planning scene
    def remove_object_from_scene(self, name):
        self.scene.remove_world_object(name)

    # Adds a rectangular prism object into the planning scene
    def add_box(self, name, size, pos, isWall=False):
        p = geometry_msgs.msg.PoseStamped() 
        p.header.frame_id = self.reference_frame
        p.pose.position.x = pos[0]
        p.pose.position.y = pos[1]
        p.pose.position.z = pos[2]
        p.pose.orientation.w = 1.0
        self.scene.add_box(name, p, size)
        print("Added %s at pos: (%f, %f, %f)" %(name, pos[0], pos[1], pos[2]))
        print("Box object has size (x: %f, y: %f, z: %f)" %(size[0], size[1], size[2]))

    # Adds a cyclinder object into the planning scene
    def add_cylinder(self, name, radius, height, pos):
        co = moveit_msgs.msg.CollisionObject()
        co.operation = moveit_msgs.msg.CollisionObject.ADD
        co.id = name
        co.header.frame_id = self.reference_frame
        box = shape_msgs.msg.SolidPrimitive()
        box.type = shape_msgs.msg.SolidPrimitive.CYLINDER
        box.dimensions = [height, radius]
        co.primitives = [box]
        p = geometry_msgs.msg.Pose()
        p.position.x = pos[0]
        p.position.y = pos[1]
        p.position.z = pos[2]
        co.primitive_poses = [p]
        self.scene._pub_co.publish(co)
        print("Added %s at pos: (%f, %f, %f)" %(name, pos[0], pos[1], pos[2]))
        print("Cylinder object has height %f and radius %f" %(height, radius))

    # Moves the arm to the specified position while setting the orientation of 
    # the hand to be facing forward in relation to the robots starting position
    def move_arm_to_pos(self, x, y, z):
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self.reference_frame
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z
        # Set orientation of hand to face forward
        p.pose.orientation.x = 0.707
        p.pose.orientation.y = 0
        p.pose.orientation.z = 0.707
        p.pose.orientation.w = 0
        self.whole_body.set_pose_target(p)
        self.whole_body.go()       

    # Moves the base, arm, head and gripper to its default position
    def move_to_neutral_pos(self):
        rospy.loginfo("step1: move_to_neutral")
        self.base.go()
        self.arm.set_named_target("neutral")
        self.arm.go()
        self.head.set_named_target("neutral")
        self.head.go()
        self.gripper.set_joint_value_target("hand_motor_joint", 0.5)
        self.gripper.go()
        rospy.sleep(self.wait)        

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

    def open_gripper(self):
        self.move_gripper(GRIPPER_OPEN_ANGLE)

    # Moves the arm up to avoid colliding with objects that are not mapped
    # Though if the object is very tall then the object will most likely hit
    # the hsr's screen
    def move_arm_lift_up(self):
        joints = ['arm_lift_joint', 'arm_flex_joint', 'arm_roll_joint', 
        'wrist_flex_joint', 'wrist_roll_joint']
        joint_values = self.arm.get_joint_value_target()
        self.arm.set_joint_value_target("arm_lift_joint", 0.2)
        # Set other joint values to their current values
        for i in range(1, len(joints)):
            self.arm.set_joint_value_target(joints[i], joint_values[i])
        self.arm.go()
        rospy.sleep(2)

    # Shuts down the MoveIt commander
    def shutdown(self):
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

# Callback function to record the x,y,z positions and 2D dimensions of the
# object that is to be picked and placed
def object_pos_callback(object_msg):
    global object_found
    global object_pos
    global object_width
    global object_height
    if object_found != True:
        print("Object has been found")
        object_found = True
        object_pos = [object_msg.x, object_msg.y, object_msg.z] 
        object_height = object_msg.camera_y
        object_width = object_msg.camera_x

def main():
    rospy.init_node("moveit_pickup_object", anonymous=True)
    object_sub = rospy.Subscriber(OBJECT_POS_TOPIC, Object, object_pos_callback)
    # Busy wait until the object has been found
    while object_found == False:
        pass
    # Start up MoveIt
    om = Object_Manipulator(object_pos[0], object_pos[1], object_pos[2], 
        object_height, object_width)
    print("Moving hand in front of object")
    om.open_gripper()
    # Offset by 15cm in front of object so the gripper doesn't collide with it
    om.move_arm_to_pos(object_pos[0]-0.15, object_pos[1], object_pos[2])
    om.remove_object_from_scene("object")
    # Initiate the force sensor capture
    # Only using it to grasp the object, not subscribed to the force torque topic
    # as it was very unreliable during testing trying to detect if the object was
    # touching the palm of the hand
    fc = force_sensor.Force_Sensor_Capture()
    rospy.sleep(2)
    om.move_arm_to_pos(object_pos[0]+0.05, object_pos[1], object_pos[2])
    print("Close grippers even if closed already")
    # Since force sensor may not be able to detect the object, assume the object
    # is within reach of the gripper any way and grab it even if already grabbed
    fc.grasp(-0.1)
    om.attach_object()
    rospy.sleep(3)
    # Move arm lift to avoid unmapped objects in environment
    om.move_arm_lift_up();
    # Set a position to place the object move the arm towards it
    print("Moving hand to place location")
    place_x = object_pos[0]+0.05
    place_y = object_pos[1]-0.7
    place_z = 0.51 + (object_height/2)
    om.move_arm_to_pos(place_x, place_y, place_z)
    # Open the gripper and then move back to the starting position
    print("Opening gripper")
    om.open_gripper()
    om.detach_object()
    rospy.sleep(2)
    om.move_to_neutral_pos()
    om.shutdown()
    

if __name__ == '__main__':
    main()
