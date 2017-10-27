#!/usr/bin/env python
# Copyright (C) 2017 Toyota Motor Corporation

# Modified the moveit_pick_and_place.py to pick up a can of coke and place
# it onto a stool using MoveIt. No extra grasping points generated with
# the ones the toyota people have provided but can be extended with agile grasps

from copy import deepcopy
import math
import sys

import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import rospy
import shape_msgs.msg
from tf.transformations import quaternion_from_euler, quaternion_multiply
import trajectory_msgs.msg


class MoveItPickupObject(object):
    def __init__(self, x, y, z):
        # initialize
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("moveit_pickup_object", anonymous=True)
        # Initialise the groups of joints used by the planner to interact with the world
        self.wait = 0
        # NOTE: Change to map when testing with real robot
        self.reference_frame = "map"       
        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.base = moveit_commander.MoveGroupCommander("base")
        self.gripper = moveit_commander.MoveGroupCommander("gripper")
        self.head = moveit_commander.MoveGroupCommander("head")
        self.whole_body = moveit_commander.MoveGroupCommander("whole_body")
        self.whole_body.allow_replanning(True)
        self.whole_body.set_planning_time(30)
        self.whole_body.set_pose_reference_frame(self.reference_frame)
        self.end_effector = self.whole_body.get_end_effector_link()
        rospy.sleep(1)
        # Reset to the default position
        self.move_to_neutral_pos()

        # Planning scene
        self.scene = moveit_commander.PlanningSceneInterface()
        # remove all objects from the planning scene
        self.scene.remove_attached_object(self.end_effector)
        self.scene.remove_world_object()
        rospy.sleep(1)

        # Set constraint for arm
        self.set_horizontal_constraint()

        cokePos = [x, y, z]
        cokeHeight = 0.23
        cokeRadius = 0.03
        self.add_cylinder("coke", cokeRadius, cokeHeight, cokePos)

        tableSize = [0.8, 0.6, cokePos[2]-cokeHeight/2]
        tableZ = z-(cokeHeight/2)
        tablePos = [x+0.3, y, tableSize[2]/2]
        self.add_box("table", tableSize, tablePos)

        stoolSize = [0.4, 0.4, 0.51]
        stoolPos = [cokePos[0]+0.1, cokePos[1]-0.7, stoolSize[2]/2]
        self.add_box("stool", stoolSize, stoolPos)
        
        rospy.sleep(1)      
        

        # Set the object that we are going to place objects onto
        self.whole_body.set_support_surface_name("table")

        # Pickup object
        currTarget = "coke"
        self.pick_from_top(currTarget)
        
        # Place object
        self.whole_body.set_support_surface_name("stool")
        placePos = [stoolPos[0], stoolPos[1], stoolSize[2]+cokeHeight/2]
        self.place_object(currTarget, placePos)

        # Return to previous position
        self.move_to_neutral_pos()
               

        # finalize
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def set_horizontal_constraint(self):
        constraints = moveit_msgs.msg.Constraints()
        constraints.name = "keep horizontal"
        orientation_constraint = moveit_msgs.msg.OrientationConstraint()
        orientation_constraint.header.frame_id = "odom"
        orientation_constraint.link_name = self.whole_body.get_end_effector_link()
        orientation_constraint.orientation.x = 0.707
        orientation_constraint.orientation.y = 0
        orientation_constraint.orientation.z = 0.707
        orientation_constraint.orientation.w = 0
        orientation_constraint.absolute_x_axis_tolerance = 3.14
        orientation_constraint.absolute_y_axis_tolerance = 0.001
        orientation_constraint.absolute_z_axis_tolerance = 3.14
        orientation_constraint.weight = 1.0
        constraints.orientation_constraints.append(orientation_constraint)
        self.whole_body.set_path_constraints(constraints)

    def pick_from_top(self, target):
        rospy.loginfo("Pick %s" %(target))
        # Grasps from the top of a cylindrical object
        yaw = [i / 180.0 * math.pi for i in range(0, 360, 30)]
        grasps = self.make_grasps(target,
                                  (1, 0, 0, 0),
                                  z=[0.1],
                                  yaw=yaw)
        self.pick(target, grasps)
        rospy.sleep(1.0)

    def pick_from_side(self, target):
        # Creates a list of all possible grasping poses to pick the object from
        # Graps from the side of the object
        grasps = self.make_grasps(target,
                                  (0.707, 0.0, 0.707, 0.0),
                                  quality=lambda x, y, z, roll, pitch, yaw: 1 - abs(pitch), # noqa
                                  x=[-0.07],
                                  pitch=[-0.2, -0.1, 0, 0.1, 0.2])
        # Attempt to create a motion plan to pickup the coke using the list of 
        # grasping poses given
        self.pick(target, grasps)
        rospy.logdebug("done")
        rospy.sleep(1.0)

    def place_object(self, target, pos):
        rospy.loginfo("Placing %s" %(target))
        newCokePos = [1.1, 0.2, 0.78]
        location = self.make_place_location(pos[0], pos[1], pos[2])
        self.place(target, location)
        rospy.logdebug("done")
        rospy.sleep(1.0)

    def pick(self, target, grasps):
        n_attempts = 0
        max_pick_attempts = 10
        result = None

        while result != moveit_msgs.msg.MoveItErrorCodes.SUCCESS and \
              n_attempts < max_pick_attempts:
            n_attempts += 1
            rospy.loginfo("Pick attempt: " + str(n_attempts))
            result = self.whole_body.pick(target, grasps)
            rospy.sleep(0.2)
        if result != moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
            self.scene.remove_attached_object(self.end_effector)
        return result

    def place(self, target, location):
        n_attempts = 0
        max_pick_attempts = 10
        result = None

        while result != moveit_msgs.msg.MoveItErrorCodes.SUCCESS and \
              n_attempts < max_pick_attempts:
            n_attempts += 1
            rospy.loginfo("Place attempt: " + str(n_attempts))
            result = self.whole_body.place(target, location)
            rospy.sleep(0.2)
        if result != moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
            self.scene.remove_attached_object(self.end_effector)
        return result

    def make_gripper_posture(self, pos, effort=0.0):
        t = trajectory_msgs.msg.JointTrajectory()
        t.joint_names = ["hand_motor_joint"]
        tp = trajectory_msgs.msg.JointTrajectoryPoint()
        tp.positions = [pos]
        tp.effort = [effort]
        tp.time_from_start = rospy.Duration(2.0)
        t.points.append(tp)
        return t

    def make_gripper_translation(self, min_dist, desired, vector, frame=None):
        g = moveit_msgs.msg.GripperTranslation()
        g.direction.vector.x = vector[0]
        g.direction.vector.y = vector[1]
        g.direction.vector.z = vector[2]
        if frame is None:
            g.direction.header.frame_id = self.end_effector
        else:
            g.direction.header.frame_id = frame
        g.min_distance = min_dist
        g.desired_distance = desired
        return g

    def make_pose(self, init, x, y, z, roll, pitch, yaw):
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = self.reference_frame
        q = quaternion_from_euler(roll, pitch, yaw)
        q = quaternion_multiply(init, q)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        return pose

    def make_grasps(self, target, init,
                    quality=None,
                    x=[0], y=[0], z=[0],
                    roll=[0], pitch=[0], yaw=[0]):
        poses = self.scene.get_object_poses([target])
        pose = poses[target]
        g = moveit_msgs.msg.Grasp()
        g.pre_grasp_posture = self.make_gripper_posture(0.8)
        g.grasp_posture = self.make_gripper_posture(0.2, -0.01)
        g.pre_grasp_approach \
            = self.make_gripper_translation(0.01, 0.02, [0.0, 0.0, 1.0])
        g.post_grasp_retreat \
            = self.make_gripper_translation(0.01, 0.02, [0.0, 0.0, 1.0],
                                            "base_footprint")
        grasps = []
        for ix in x:
            for iy in y:
                for iz in z:
                    for iroll in roll:
                        for ipitch in pitch:
                            for iyaw in yaw:
                                x = pose.position.x + ix
                                y = pose.position.y + iy
                                z = pose.position.z + iz
                                g.grasp_pose = self.make_pose(init,
                                                              x, y, z,
                                                              iroll,
                                                              ipitch,
                                                              iyaw)
            g.id = str(len(grasps))
            g.allowed_touch_objects = ["coke"]
            g.max_contact_force = 0
            if quality is None:
                g.grasp_quality = 1.0
            else:
                g.grasp_quality = quality(ix, iy, iz, iroll, ipitch, iyaw)
            grasps.append(deepcopy(g))
        print(grasps)
        return grasps

    def make_place_location(self, x, y, z):
        location = moveit_msgs.msg.PlaceLocation()
        location.pre_place_approach \
            = self.make_gripper_translation(0.03, 0.05, [0, 0, -1.0],
                                            "base_footprint")
        location.post_place_posture \
            = self.make_gripper_posture(0.8)
        location.post_place_retreat \
            = self.make_gripper_translation(0.03, 0.05, [0, 0, -1.0])
        location.place_pose = self.make_pose((0, 0, 0, 1),
                                             x, y, z,
                                             0, 0, 0)
        return location


    def move_to_neutral_pos(self):
        rospy.loginfo("step1: move_to_neutral")
        self.base.go()
        self.arm.set_named_target("neutral")
        self.arm.go()
        self.head.set_named_target("neutral")
        self.head.go()
        self.gripper.set_joint_value_target("hand_motor_joint", 0.5)
        self.gripper.go()
        rospy.logdebug("done")
        rospy.sleep(self.wait)

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


if __name__ == "__main__":
    target_x = float(sys.argv[1])
    target_y = float(sys.argv[2])
    target_z = float(sys.argv[3])
    MoveItPickupObject(target_x, target_y, target_z)
