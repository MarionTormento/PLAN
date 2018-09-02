#!/usr/bin/env python

# Copyright (C) 2018 Marion Tormento 05/07/2018.
# License, GNU LGPL, free software, without any warranty.
import rospy
# import moveit_commander
import roslib
from pcl_detection.msg import obj
import sys
import cProfile, pstats
import time 
import rospy
import roslib; roslib.load_manifest("moveit_python")
import baxter_interface
from moveit_python import PlanningSceneInterface, MoveGroupInterface
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from math import pi, sqrt, cos, sin
from operator import itemgetter
from copy import deepcopy
import numpy as np
import baxter_pykdl as kdl
from tf.transformations import *

# Init node
rospy.init_node('kinect_pnp', anonymous=True)
# Baxter Kinematics
rightSolver = kdl.baxter_kinematics('right')
leftSolver = kdl.baxter_kinematics('left')
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

# Global variables
approach = PoseStamped()

bskposR = PoseStamped()
bskposR.header.frame_id = "base"
bskposR.header.stamp = rospy.Time.now()
bskposR.pose.position.x = 1.0
bskposR.pose.position.y = -0.30
bskposR.pose.position.z = 0.70
bskposR.pose.orientation.x = 0.0
bskposR.pose.orientation.y = 0.707
bskposR.pose.orientation.z = 0.0
bskposR.pose.orientation.w = 0.707

bskposL = PoseStamped()
bskposL.header.frame_id = "base"
bskposL.header.stamp = rospy.Time.now()
bskposL.pose.position.x = 1.0
bskposL.pose.position.y = 0.30
bskposL.pose.position.z = 0.70
bskposL.pose.orientation.x = 0.0
bskposL.pose.orientation.y = 0.707
bskposL.pose.orientation.z = 0.0
bskposL.pose.orientation.w = 0.707

compTime = 2.0
robotSpd = 0.7
# Define the joints for the positions.
jts_both = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2', 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
jts_right = ['right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
jts_left = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2']
pos1 = [-1.5992511877797613, 1.3277948504704176, 0.689270903962913, -1.4012007670249345, 0.1754894648887202, 1.8146491089961856, 3.0589327838441367, 1.9602758615925566, 1.4333515313858163, -0.5903707508267928, -1.005913550861348, -0.5306659661076933, 2.051042542353038, -2.677815699467131]
pos_high = [-1.3525875597179635, 1.1662088939898858, 1.0538448012772792, -1.3380147422330522, 0.24850488763743703, 1.7560245069318274, 3.045335359149518, 1.9190099656446526, 1.3533545501119062, -1.151252581308003, -0.9921020745648913, -0.6787864986392956, 1.8607186957050068, -2.7857091107999112]
approach = [0.9257574054888472, 1.249810846929641, -0.060975736318445196, -0.6415874645330742, 0.3992185000471789, 1.4243011615516066, -1.7092380929013222]
# jts_zero = [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0] # Neutral
jts_zero = [0.0, (-2.147+1.047)/2, 0.0, (-0.05+2.618)/2, 0.0, (-1.5707+2.094)/2, 0.0] #[right_s0, right_s1, right_e0, right_e1, right_w0, right_w1, right_w2]

objectName = []
closeRC = [0,0,0]
closeLC = [0,0,0]
# Define the orientation for the tested poses
# Try different orientation to find best option.
# right side : -0.5 0.5 0.5 0.5
right_side = Quaternion()
right_side.x = -0.653
right_side.y = 0.653
right_side.z = 0.271
right_side.w = 0.271
# left side : 0.5 0.5 -0.5 0.5
left_side = Quaternion()
left_side.x = 0.653
left_side.y = 0.653
left_side.z = -0.271
left_side.w = 0.271
# right/left front : 0.0 0.707 0.0 0.707
front = Quaternion()
front.x = 0.0
front.y = 0.707
front.z = 0.0
front.w = 0.707
# right/left front : 0.0 0.707 0.0 0.707
frontOriented = Quaternion()
frontOriented.x = -0.924
frontOriented.y = 0.0
frontOriented.z = -0.383
frontOriented.w = 0.0
# top : 1.0 0.0 0.0 0.0
top = Quaternion()
top.x = 1.0
top.y = 0.0
top.z = 0.0
top.w = 0.0
# top box
top_box = deepcopy(top)
# side box
box_side = deepcopy(right_side)
box_front = deepcopy(front)

# list of poses for each arm for the cylinder case
rightPoses = [right_side, front, frontOriented, top]
leftPoses = [left_side, front, frontOriented, top]


def addObj(msg):
    global objectName, closeRC, closeLC

    obj = SolidPrimitive()
    obj_ps = PoseStamped()
    obj_ps.header.frame_id = p._fixed_frame

    if msg.name == 'table':
        p.clear()
        objectName = []

        obj.dimensions = [msg.dimensions[0], msg.dimensions[1], msg.center[2]+0.65]
        obj.type = obj.BOX

        obj_ps.pose.position.x = msg.center[0]
        obj_ps.pose.position.y = msg.center[1]
        obj_ps.pose.position.z = -0.595+(msg.center[2]+0.65)/2
        obj_ps.pose.orientation.x = msg.quaternion[0]
        obj_ps.pose.orientation.y = msg.quaternion[1]
        obj_ps.pose.orientation.z = msg.quaternion[2]
        obj_ps.pose.orientation.w = msg.quaternion[3]

        closeRC = [obj_ps.pose.position.x-obj.dimensions[0]/2, obj_ps.pose.position.y-obj.dimensions[1]/2, obj_ps.pose.position.z+obj.dimensions[2]/2]
        closeLC = [obj_ps.pose.position.x-obj.dimensions[0]/2, obj_ps.pose.position.y+obj.dimensions[1]/2, obj_ps.pose.position.z+obj.dimensions[2]/2]
        
        print "The table has been detected"

    if msg.name[0:8] == 'cylinder':
        obj.dimensions = [msg.dimensions[0], msg.dimensions[1]]
        obj.type = obj.CYLINDER

        obj_ps.pose.position.x = msg.center[0]
        obj_ps.pose.position.y = msg.center[1]
        obj_ps.pose.position.z = -0.595+0.65+msg.center[2]
        obj_ps.pose.orientation.x = msg.quaternion[0]
        obj_ps.pose.orientation.y = msg.quaternion[1]
        obj_ps.pose.orientation.z = msg.quaternion[2]
        obj_ps.pose.orientation.w = -msg.quaternion[3]        

        if msg.name not in objectName:
            objectName.append(msg.name)
        print "The object", msg.name, "has been detected"

    if msg.name[0:3] == 'box':
        obj.dimensions = [msg.dimensions[0], msg.dimensions[1], msg.dimensions[2]]
        obj.type = obj.BOX

        obj_ps.pose.position.x = msg.center[0]
        obj_ps.pose.position.y = msg.center[1]
        obj_ps.pose.position.z = -0.595+0.65+msg.center[2]


        obj_ps.pose.orientation.x = msg.quaternion[0]
        obj_ps.pose.orientation.y = msg.quaternion[1]
        obj_ps.pose.orientation.z = msg.quaternion[2]
        obj_ps.pose.orientation.w = msg.quaternion[3]

        if msg.name not in objectName:
            objectName.append(msg.name)
        print "The object", msg.name, "has been detected"

    p.addSolidPrimitive(msg.name, obj, obj_ps.pose, True)

def pick(nameobject, collisionObject, side):    
    global approach

    if side == 'right':
        arm = gr
        gripper = rightgripper
        grip = "right_gripper"
    elif side == 'left':
        arm = gl
        gripper = leftgripper
        grip = "left_gripper"
    else: 
        print "WARNING! This is a single arm manipulation task"

    # Find the possible grasp depending on the shape of the object
    graspList = isGraspable(collisionObject, arm)
    if len(graspList) != 0:
        # if nameobject == 'box0':
        #     grasp = graspList[1]
        #     approach = findApproach(grasp)
        #     print "POSE1"
        #     # p.clear()

        #     approachTest = arm.moveToPose(approach, grip, max_velocity_scaling_factor = robotSpd, plan_only = False,  planning_time = compTime)
        #     p.removeCollisionObject(nameobject)
        #     approachTest = arm.moveToPose(grasp, grip, max_velocity_scaling_factor = robotSpd, plan_only = False,  planning_time = compTime)
        #     # p.addSolidPrimitive(nameobject, collisionObject.primitives[0], collisionObject.primitive_poses[0], True)      
        #     # time.sleep(5)
        #     # grasp = graspList[1]
        #     # # approach = findApproach(grasp)
        #     # print "POSE1"
        #     # # approachTest = arm.moveToPose(approach, grip, max_velocity_scaling_factor = robotSpd, plan_only = False,  planning_time = compTime)
        #     # # p.removeCollisionObject(nameobject)
        #     # approachTest = arm.moveToPose(grasp, grip, max_velocity_scaling_factor = robotSpd, plan_only = False,  planning_time = compTime)
        #     # # p.addSolidPrimitive(nameobject, collisionObject.primitives[0], collisionObject.primitive_poses[0], True)      
        #     # time.sleep(5)
        #     # # print "POSE3"
        #     # grasp = graspList[2]
        #     # approachTest = arm.moveToPose(grasp, grip, max_velocity_scaling_factor = robotSpd, plan_only = False,  planning_time = compTime)
        #     time.sleep(100)
        # else:
        p.removeCollisionObject(nameobject)
        grasp, approach = findBestPose(graspList, arm, grip)
        p.addSolidPrimitive(nameobject, collisionObject.primitives[0], collisionObject.primitive_poses[0], True)

        
        # grasp = graspList[0]
        # approach = deepcopy(grasp)
        # p.clear()
     
        if grasp != None: 
            approachTest = arm.moveToPose(approach, grip, max_velocity_scaling_factor = robotSpd, plan_only = False,  planning_time = compTime)
            print "APPROACH", approachTest.error_code, "COST", costFunction(approachTest, approach, arm)
            # costFunction(approachTest, approach, arm)
            if approachTest.error_code.val == -1 or approachTest.error_code.val == -2:
                print "IMPOSSIBLE TO REACH THE APPROACH, GOING TO REACH NEXT OBJECT"
                return False, grasp
            else:
                print "OBJECT APPROACHED"
                p.removeCollisionObject(nameobject)
                graspTest = arm.moveToPose(grasp, grip, max_velocity_scaling_factor = robotSpd, plan_only = False,  planning_time = compTime)
                costFunction(graspTest, grasp, arm)
                print "GRASP", graspTest.error_code, "COST", costFunction(graspTest, grasp, arm)
                if graspTest.error_code.val == -1 or graspTest.error_code.val == -2:
                    print "IMPOSSIBLE TO REACH THE OBJECT, GOING TO REACH NEXT OBJECT"
                    p.addSolidPrimitive(nameobject, collisionObject.primitives[0], collisionObject.primitive_poses[0], True)
                    return False, grasp
                else:
                    print "OBJECT GRASPED"
                    gripper.close()
                    # attachObject2Robot(nameobject, collisionObject, grip)
                    return True, grasp


        elif grasp == None:
            print "IMPOSSIBLE TO REACH THE OBJECT, GOING TO REACH NEXT OBJECT"
            return False, grasp
    else:
        grasp = None
        print "IMPOSSIBLE TO REACH THE OBJECT BECAUSE IT IS TOO BIG FOR THE GRIPPERS, GOING TO REACH NEXT OBJECT"
        return False, grasp       

def isGraspable(collisionObject, arm):
    objectType = SolidPrimitive()
    GRIP_SIZE = 0.1
    graspList = []

    if collisionObject.primitives[0].type == objectType.CYLINDER:
        if arm == gr:
            testPoses = rightPoses
        elif arm == gl:
            testPoses = leftPoses
        # if collisionObject.primitives[0].dimensions[1] >= GRIP_SIZE: #range of grippers
        #     return graspList
        # else:
        if collisionObject.primitives[0].dimensions[1] <= GRIP_SIZE:        
            for quat in testPoses:
                grasp = PoseStamped()
                grasp.header = deepcopy(collisionObject.header)
                grasp.pose = deepcopy(collisionObject.primitive_poses[0])
                grasp.pose.orientation = quat
                if quat == right_side: # OK
                    grasp.pose.position.x = 0.97*grasp.pose.position.x 
                    grasp.pose.position.y = grasp.pose.position.y + min((collisionObject.primitives[0].dimensions[1]),0.05) #account for object being too tall most of the time
                    grasp.pose.position.z = grasp.pose.position.z - 0.5*(collisionObject.primitives[0].dimensions[0]) #account for object being too tall most of the time
                elif quat == left_side: #?
                    grasp.pose.position.z = grasp.pose.position.z - 0.5*(collisionObject.primitives[0].dimensions[0]) #account for object being too tall most of the time
                    grasp.pose.position.y = grasp.pose.position.y - min((collisionObject.primitives[0].dimensions[1]),0.05) #account for object being too tall most of the time
                elif quat == front: # FRONT OK DROIT PAS GAUCHE
                    grasp.pose.position.y = 0.975*grasp.pose.position.y 
                elif quat == frontOriented: # ok
                    grasp.pose.position.y = 0.975*grasp.pose.position.y 
                    grasp.pose.position.z = grasp.pose.position.z - 0.5*(collisionObject.primitives[0].dimensions[0]) #account for object being too tall most of the time
                elif quat == top: # ok
                    grasp.pose.position.z = grasp.pose.position.z + collisionObject.primitives[0].dimensions[0]/2 - 0.08
                graspList.append(grasp)
        # return graspList


    elif collisionObject.primitives[0].type == objectType.BOX:
        grasp = PoseStamped()
        grasp.header = deepcopy(collisionObject.header)
        grasp.pose = deepcopy(collisionObject.primitive_poses[0])
        grasp2 = deepcopy(grasp)
        grasp3 = deepcopy(grasp)
        quat = [grasp.pose.orientation.x, grasp.pose.orientation.y, grasp.pose.orientation.z, grasp.pose.orientation.w]
        euler = euler_from_quaternion(quat)
        yaw = -(euler[2]+pi/2)
        if collisionObject.primitives[0].dimensions[0] <= GRIP_SIZE:
            # top
            quat_top = quaternion_from_euler(pi, 0.0, yaw, axes='rxyz')
            top_box.x = quat_top[0]
            top_box.y = quat_top[1]
            top_box.z = quat_top[2]
            top_box.w = quat_top[3]
            grasp.pose.position.z = grasp.pose.position.z + 0.5*collisionObject.primitives[0].dimensions[2] - 0.08#account for object being too tall most of the time
            # grasp.pose.position.x = grasp.pose.position.x - 0.10*(collisionObject.primitives[0].dimensions[0]) #account for object being too tall most of the time
            # grasp.pose.position.y = grasp.pose.position.y - 0.10*(collisionObject.primitives[0].dimensions[0]) #account for object being too tall most of the time           
            grasp.pose.orientation = top_box
            graspList.append(grasp)
            # side
            quat_side = quaternion_from_euler(pi, yaw, pi/2, axes='rxzy')#
            # euler_side = euler_from_quaternion(quat_side)
            # angle = euler[0]
            # grasp2.pose.position.x = grasp2.pose.position.x + (0.5*collisionObject.primitives[0].dimensions[1]-0.15)*sin(angle) 
            # grasp2.pose.position.y = grasp2.pose.position.y + (0.5*collisionObject.primitives[0].dimensions[1]-0.15)*cos(angle)            
            box_side.x = quat_side[0]
            box_side.y = quat_side[1]
            box_side.z = quat_side[2]
            box_side.w = quat_side[3]
            grasp2.pose.orientation = box_side
            graspList.append(grasp2) 

        if collisionObject.primitives[0].dimensions[1] <= GRIP_SIZE:
            # front
            quat_front = quaternion_from_euler(pi, yaw-pi/2, pi/2, axes='rxzy')
            euler_front = euler_from_quaternion(quat_front)
            angle2 = euler[0]
            grasp3.pose.position.x = grasp3.pose.position.x + (0.5*collisionObject.primitives[0].dimensions[1]-0.03)*sin(angle) 
            grasp3.pose.position.y = grasp3.pose.position.y + (0.5*collisionObject.primitives[0].dimensions[1]-0.03)*cos(angle)
            box_front.x = quat_front[0]
            box_front.y = quat_front[1]
            box_front.z = quat_front[2]
            box_front.w = quat_front[3]
            grasp3.pose.orientation = box_front
            graspList.append(grasp3) 
           
        
    return graspList

def findBestPose(graspList, arm, grip):
    globalCost = []
    idx = 1
    
    for grasp in graspList:
        print "TESTING FOR GRASP", idx
        graspCost = testPose(grasp, arm, grip)
        if graspCost != 10000:
            print "TESTING FOR APPROACH", idx
            approach = findApproach(grasp)
            approachCost = testPose(approach, arm, grip)
        else:
            approachCost = 10000
        globalCost.append(graspCost+approachCost)
        idx += 1

    print "GLOBAL COST", globalCost

    if min(globalCost) != 20000:
        idx = np.where(globalCost == min(globalCost))[0][0]
        grasp = graspList[idx]
        approach = findApproach(grasp)
    else:
        grasp = None
        approach = None

    return grasp, approach

def testPose(pose, arm, grip):
    test = arm.moveToPose(pose, grip, max_velocity_scaling_factor = robotSpd, plan_only = True,  planning_time = compTime)
    if test.error_code.val == -1 or test.error_code.val == -2:
        return 10000
    else:
        cost = costFunction(test, pose, arm)
        return cost

def costFunction(trajectory, pose, arm):
        poseFct = 150
        jointFct = 1
        costPose = 0
        costJoints = 0
        finalPose = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
        for point in trajectory.planned_trajectory.joint_trajectory.points:
            joint = point.positions
            currentPose = endPose(joint, arm)
            currentPose = currentPose[0:3]
            costPose += np.sum((finalPose - currentPose)**2)
            costJoints += np.sum((np.asarray(joint)-np.asarray(jts_zero))**2)
        cost = poseFct*costPose + jointFct*costJoints
        print "COST POSE", poseFct*costPose, "COST JOINTS", jointFct*costJoints, "COST TOTAL", cost
        return cost

def endPose(joints, arm):
        if arm == gr:
            solver = rightSolver
            jointsdico = {'right_s0': joints[0], 
              'right_s1': joints[1], 
              'right_w0': joints[4], 
              'right_w1': joints[5], 
              'right_w2': joints[6], 
              'right_e0': joints[2], 
              'right_e1': joints[3]}
            
        elif arm == gl:
            solver = leftSolver
            jointsdico = {'left_s0': joints[0], 
              'left_s1': joints[1], 
              'left_w0': joints[4], 
              'left_w1': joints[5], 
              'left_w2': joints[6], 
              'left_e0': joints[2], 
              'left_e1': joints[3]}            

        correspondingPose = solver.forward_position_kinematics(jointsdico)
        return correspondingPose

def findApproach(grasp):
    approach = deepcopy(grasp)
    quat = grasp.pose.orientation
    SHIFT = 0.10

    if quat == front or quat == frontOriented:
        approach.pose.position.x = approach.pose.position.x - SHIFT
    elif quat == top or quat == top_box:
        approach.pose.position.z = approach.pose.position.z + SHIFT
    elif quat == right_side:
        approach.pose.position.y = approach.pose.position.y - SHIFT
    elif quat == left_side:
        approach.pose.position.y = approach.pose.position.y + SHIFT
    elif quat == box_side or quat == box_front:
        quat2 = [quat.x, quat.y, quat.z, quat.w]
        euler = euler_from_quaternion(quat2)
        angle = euler[0]#+pi/2
        approach.pose.position.x = approach.pose.position.x + SHIFT*sin(angle) 
        approach.pose.position.y = approach.pose.position.y + SHIFT*cos(angle)

       
    return approach        

def attachObject2Robot(name, collisionObject, link_name):
    collisionObject.primitives[0].dimensions[0] = collisionObject.primitives[0].dimensions[0]/2
    collisionObject.header.frame_id = link_name
    collisionObject.primitive_poses[0].position.x = 0
    collisionObject.primitive_poses[0].position.y = 0
    collisionObject.primitive_poses[0].position.z = collisionObject.primitives[0].dimensions[0]+0.02
    collisionObject.primitive_poses[0].orientation.x = 0
    collisionObject.primitive_poses[0].orientation.y = 0
    collisionObject.primitive_poses[0].orientation.z = 0    
    collisionObject.primitive_poses[0].orientation.w = 1    
    a = p.makeAttached(link_name, collisionObject, touch_links=None, detach_posture=None, weight=0.0)
    p._attached_objects[name] = a
    p.sendUpdate(None, a, use_service=True)

def place(side, currentPose):    
    if side == 'right':
        arm = gr
        gripper = rightgripper
        grip = "right_gripper"
        bskpos = bskposR
    elif side == 'left':
        arm = gl
        gripper = leftgripper
        grip = "left_gripper"
        bskpos = bskposL
    else: 
        print "WARNING! This is a single arm manipulation task"

    
    retreat =deepcopy(currentPose)
    retreat.pose.position.z = approach.pose.position.z + 0.20


    retreatTest = arm.moveToPose(retreat, grip, max_velocity_scaling_factor = robotSpd, plan_only=False,  planning_time = compTime)
    print "RETREAT", retreatTest.error_code, "COST", costFunction(retreatTest, retreat, arm)
    
    bskTest = arm.moveToPose(bskpos, grip, max_velocity_scaling_factor = robotSpd, plan_only=False,  planning_time = compTime)  
    gripper.open()
    print "RETURN", bskTest.error_code, "COST", costFunction(bskTest, bskpos, arm)

    # print "REMOVING ATTACHED OBJECT"
    # for name in p.getKnownAttachedObjects():
    #     print "COUUUCOU", name
    #     # p.removeAttachedObject(name, True)
    #     """ Remove an attached object. """
    #     o = AttachedCollisionObject()
    #     o.object.operation = CollisionObject.REMOVE
    #     o.object.id = name

    #     try:
    #         del p._attached_objects[name]
    #         p._attached_removed[name] = o
    #     except KeyError:
    #         pass
        # p.sendUpdate(None, o, use_service=True)

def distance2Corner(x,y, corner1, corner2):
    distance1 = (x-corner1[0])**2+(y-corner1[1])**2
    distance2 = (x-corner2[0])**2+(y-corner2[1])**2
    distance = min(distance1, distance2)
    return distance

if __name__=='__main__':
    try:
        global objectName, closeRC, closeLC

        object_topic       = rospy.get_param('~object_topic')
        rospy.Subscriber(object_topic, obj, addObj)
    
        rightgripper.calibrate()
        leftgripper.calibrate()
        rightgripper.open()
        leftgripper.open()
        g.moveToJointPosition(jts_both, pos_high,  max_velocity_scaling_factor = 0.5, plan_only=False)

        # cProfile to measure the performance (time) of the task.
        pr = cProfile.Profile()
        pr.enable()        

        time.sleep(5)
        print "There is", len(objectName), "object(s) detected on the table"
        g.moveToJointPosition(jts_both, pos1,  max_velocity_scaling_factor = 0.5, plan_only=False)

        # Ordered list of object poses from the closest to the further away in x direction
        collisionObjects = [0]*len(objectName)
        objNameOrdered = [0]*len(objectName)
        # allX = [] # both side
        distance = [] # only right side
        for obj in objectName:
            exec(obj+"=CollisionObject()")
            collisionObject = globals()[obj]
            collisionObject = p._objects[obj]
            if collisionObjects[0] == 0:
                collisionObjects[0] = collisionObject
                objNameOrdered[0] = obj
                # allX.append(collisionObject.primitive_poses[0].position.x)
                dist = distance2Corner(collisionObject.primitive_poses[0].position.x, collisionObject.primitive_poses[0].position.y, closeRC, closeLC)
                distance.append(dist)
                # print "COUCOUUU2", dist
            else:
                # newX = collisionObject.primitive_poses[0].position.x
                newDist = distance2Corner(collisionObject.primitive_poses[0].position.x, collisionObject.primitive_poses[0].position.y, closeRC, closeLC)
                i = 0
                # for x in allX:
                for d in distance:
                    # if newX > x:
                    if newDist > d:
                        i += 1
                    else:
                        collisionObjects[i+1:len(distance)+1] = collisionObjects[i:len(distance)]
                        collisionObjects[i] = collisionObject
                        objNameOrdered[i+1:len(distance)+1] = objNameOrdered[i:len(distance)]
                        objNameOrdered[i] = obj
                        # collisionObjects[i+1:len(allX)+1] = collisionObjects[i:len(allX)]
                        # collisionObjects[i] = collisionObject
                        # objNameOrdered[i+1:len(allX)+1] = objNameOrdered[i:len(allX)]
                        # objNameOrdered[i] = obj
                        break
                    # if i == len(allX):
                    if i == len(distance):
                        collisionObjects[i] = collisionObject
                        objNameOrdered[i] = obj
                # allX.append(newX)
                # allX.sort()
                distance.append(newDist)
                distance.sort()

        objectName = objNameOrdered
        del objNameOrdered
        
    
        # cProfile to measure the performance (time) of the task.
        pr = cProfile.Profile()
        pr.enable()

        i = 0        
        for collisionObject in collisionObjects:
            name = objectName[i]
            if collisionObject.primitive_poses[0].position.y < 0:
                side = 'right'
            else:
                side = 'left'
            # print "Correct side should be ", side
            # side = 'right'
            print "READY TO PICK UP OBJECT", name
            picked, grasp = pick(name, collisionObject, side)
            if picked:
                print "READY TO PLACE OBJECT", name
                place(side, grasp)
                g.moveToJointPosition(jts_both, pos1,  max_velocity_scaling_factor = 0.8, plan_only=False)
            i += 1
   

        pr.disable()
        sortby = 'cumulative'
        ps=pstats.Stats(pr).sort_stats(sortby).print_stats(0.0)

    except rospy.ROSInterruptException:
        pass