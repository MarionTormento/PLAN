#!/usr/bin/env python

# Copyright (C) 2017 Maik Heufekes, 05/07/2017.
# License, GNU LGPL, free software, without any warranty.

import sys
import cProfile, pstats
import time 
import rospy
import roslib; roslib.load_manifest("moveit_python")
import baxter_interface
from moveit_python import PlanningSceneInterface, MoveGroupInterface
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from math import pi, sqrt
from operator import itemgetter
from copy import deepcopy


# Define universe
# Table
table_size_x = 0.375  
table_size_y = 0.755
table_size_z = 0.74
center_x = 0.7
center_y = 0
center_table_z = -0.595+table_size_z/2
# The distance from the zero point in Moveit to the ground is 0.635 m.
can_height = 0.20
can_radius = 0.025
center_can_z= -0.635+table_size_z+can_height/2

class Box():
    def __init__(self, name):
        self.name = name
        self.x = table_size_x
        self.y = table_size_y
        self.z = table_size_z
        self.cx = center_x
        self.cy = center_y
        self.cz = center_table_z
        self.frame = 'base'
        self.touch_links = ['base']

class Cylinder():
    def __init__(self, name):
        self.name = name
        self.h = can_height
        self.r = can_radius
        self.z = table_size_z
        self.cx = center_x
        self.cy = center_y
        self.cz = center_can_z

# Define the joints for the positions.
jts_both = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2', 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
jts_right = ['right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
jts_left = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2']
pos1 = [-1.5992511877797613, 1.3277948504704176, 0.689270903962913, -1.4012007670249345, 0.1754894648887202, 1.8146491089961856, 3.0589327838441367, 1.9602758615925566, 1.4333515313858163, -0.5903707508267928, -1.005913550861348, -0.5306659661076933, 2.051042542353038, -2.677815699467131]
approach = [0.9257574054888472, 1.249810846929641, -0.060975736318445196, -0.6415874645330742, 0.3992185000471789, 1.4243011615516066, -1.7092380929013222]



# rpos = PoseStamped()
# rpos.header.frame_id = "base"
# rpos.header.stamp = rospy.Time.now()
# rpos.pose.position.x = 0.555
# rpos.pose.position.y = 0.0
# rpos.pose.position.z = 0.406
# rpos.pose.orientation.x = 1.0
# rpos.pose.orientation.y = 0.0
# rpos.pose.orientation.z = 0.0
# rpos.pose.orientation.w = 0.0

# canpos = PoseStamped()
# canpos.header.frame_id = "base"
# canpos.header.stamp = rospy.Time.now()
# canpos.pose.position.x = 0.705 #0.686258743968
# canpos.pose.position.y = 0.0158779273329
# canpos.pose.position.z = 0.220 #0.226870532021
# canpos.pose.orientation.x = -0.581081706794
# canpos.pose.orientation.y = 0.620254560677
# canpos.pose.orientation.z = 0.395666284238
# canpos.pose.orientation.w = 0.347960517166

# def init():
#     rospy.init_node('pnp', anonymous=True)
#     # Initialize the planning scene interface.
#     p = PlanningSceneInterface("base")
#     # Connect the arms to the move group.
#     g = MoveGroupInterface("both_arms", "base")
#     gr = MoveGroupInterface("right_arm", "base")
#     gl = MoveGroupInterface("left_arm", "base")
#     # Create baxter_interface limb instance.
#     rightarm = baxter_interface.limb.Limb('right')
#     leftarm = baxter_interface.limb.Limb('left')
#     # Create baxter_interface gripper instance.
#     rightgripper = baxter_interface.Gripper('right')
#     leftgripper = baxter_interface.Gripper('left')

def update_ps(planscene, *arg):
    # Clear planning scene.
    planscene.clear()
    # Add table as attached object.
    planscene.attachBox(arg[0].name, arg[0].x, arg[0].y, arg[0].z, arg[0].cx, arg[0].cy, arg[0].cz, arg[0].frame, touch_links=arg[0].touch_links)
    for i in range(1, len(arg)):
        planscene.addCylinder(arg[i].name, arg[i].h, arg[i].r, arg[i].cx, arg[i].cy, arg[i].cz)
    planscene.waitForSync()

def pick(objectpose, side, planscene, *arg):    
    if side == 'right':
        arm = gr
        gripper = rightgripper
        grip = "right_gripper"
    elif side == 'left':
        arm = gl
        gripper = leftgripper
        grip = "left_gripper"
    else: 
        print("WARNING! This is a single arm manipulation task")

    approach=deepcopy(objectpose)
    approach.pose.position.z = approach.pose.position.z + 0.15

    arm.moveToPose(approach, grip, max_velocity_scaling_factor = 0.5, plan_only=False)

    update_ps(planscene, *arg)

    arm.moveToPose(objectpose, grip, max_velocity_scaling_factor = 0.5, plan_only=False)
    gripper.close()

    arm.moveToPose(approach, grip, max_velocity_scaling_factor = 0.5, plan_only=False)


    # arm.moveToPose(rpos, "right_gripper", max_velocity_scaling_factor = 0.5, plan_only=False)
    # print("right pose reached")
    # arm.moveToJointPosition(jts_right, approach, max_velocity_scaling_factor = 0.5, plan_only=False)
    # print("approach reached")
    # arm.moveToPose(canpos, "right_gripper", max_velocity_scaling_factor = 0.5, plan_only=False)
    # print("can reached")
    # gripper.close()
    # time.sleep(2)
    # arm.moveToPose(rpos, "right_gripper", max_velocity_scaling_factor = 0.5, plan_only=False)
    # print("right pose 2 reached")

def place(basketpose, side):    
    if side == 'right':
        arm = gr
        gripper = rightgripper
        grip = "right_gripper"
    elif side == 'left':
        arm = gl
        gripper = leftgripper
        grip = "left_gripper"
    else: 
        print("WARNING! This is a single arm manipulation task")

    approach=deepcopy(basketpose)
    approach.pose.position.z = approach.pose.position.z + 0.15

    arm.moveToPose(approach, grip, max_velocity_scaling_factor = 0.5, plan_only=False)

    arm.moveToPose(basketpose, grip, max_velocity_scaling_factor = 0.5, plan_only=False)
    gripper.open()

    arm.moveToPose(approach, grip, max_velocity_scaling_factor = 0.5, plan_only=False)

if __name__=='__main__':
    try:
        # init()
        rospy.init_node('pnp', anonymous=True)
        # Initialize the planning scene interface.
        p = PlanningSceneInterface("base")
        # Connect the arms to the move group.
        g = MoveGroupInterface("both_arms", "base")
        gr = MoveGroupInterface("right_arm", "base")
        gl = MoveGroupInterface("left_arm", "base")
        # Create baxter_interface limb instance.
        rightarm = baxter_interface.limb.Limb('right')
        leftarm = baxter_interface.limb.Limb('left')
        # Create baxter_interface gripper instance.
        rightgripper = baxter_interface.Gripper('right')
        leftgripper = baxter_interface.Gripper('left')
        rightgripper.calibrate()
        leftgripper.calibrate()
        rightgripper.open()
        leftgripper.open()
        
        # Create pos
        canpos = PoseStamped()
        canpos.header.frame_id = "base"
        canpos.header.stamp = rospy.Time.now()
        canpos.pose.position.x = 0.70 #0.686258743968
        canpos.pose.position.y = 0.00
        canpos.pose.position.z = 0.25 #0.226870532021
        canpos.pose.orientation.x = 1.0
        canpos.pose.orientation.y = 0.0
        canpos.pose.orientation.z = 0.0
        canpos.pose.orientation.w = 0.0

        bskpos = PoseStamped()
        bskpos.header.frame_id = "base"
        bskpos.header.stamp = rospy.Time.now()
        bskpos.pose.position.x = 0.50
        bskpos.pose.position.y = -0.50
        bskpos.pose.position.z = 0.10
        bskpos.pose.orientation.x = 1.0
        bskpos.pose.orientation.y = 0.0
        bskpos.pose.orientation.z = 0.0
        bskpos.pose.orientation.w = 0.0
        
        # Set planning scene.
        table = Box('table')
        can = Cylinder('can')
        update_ps(p, table, can)
        
        # cProfile to measure the performance (time) of the task.
        pr = cProfile.Profile()
        pr.enable()

        #Init
        rightgripper.open()
        # g.moveToJointPosition(jts_both, pos1,  max_velocity_scaling_factor = 0.5, plan_only=False)

        # Pick and place
        pick(canpos, 'right', p, table)
        place(bskpos, 'right')

        pr.disable()
        sortby = 'cumulative'
        ps=pstats.Stats(pr).sort_stats(sortby).print_stats(0.0)

    except rospy.ROSInterruptException:
        pass
