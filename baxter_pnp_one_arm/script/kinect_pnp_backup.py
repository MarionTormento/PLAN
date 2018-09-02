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
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from math import pi, sqrt
from operator import itemgetter
from copy import deepcopy
import numpy as np

# Init node
rospy.init_node('kinect_pnp', anonymous=True)
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

bskpos = PoseStamped()
bskpos.header.frame_id = "base"
bskpos.header.stamp = rospy.Time.now()
bskpos.pose.position.x = 1.0
bskpos.pose.position.y = -0.30
bskpos.pose.position.z = 0.70
bskpos.pose.orientation.x = 0.0
bskpos.pose.orientation.y = 0.707
bskpos.pose.orientation.z = 0.0
bskpos.pose.orientation.w = 0.707

compTime = 5.0
robotSpd = 1.0
# Define the joints for the positions.
jts_both = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2', 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
jts_right = ['right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
jts_left = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2']
pos1 = [-1.5992511877797613, 1.3277948504704176, 0.689270903962913, -1.4012007670249345, 0.1754894648887202, 1.8146491089961856, 3.0589327838441367, 1.9602758615925566, 1.4333515313858163, -0.5903707508267928, -1.005913550861348, -0.5306659661076933, 2.051042542353038, -2.677815699467131]
pos_high = [-1.3525875597179635, 1.1662088939898858, 1.0538448012772792, -1.3380147422330522, 0.24850488763743703, 1.7560245069318274, 3.045335359149518, 1.9190099656446526, 1.3533545501119062, -1.151252581308003, -0.9921020745648913, -0.6787864986392956, 1.8607186957050068, -2.7857091107999112]
approach = [0.9257574054888472, 1.249810846929641, -0.060975736318445196, -0.6415874645330742, 0.3992185000471789, 1.4243011615516066, -1.7092380929013222]
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
# list of poses for each arm
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
        obj_ps.pose.orientation.w = msg.quaternion[3]        

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
    p.removeCollisionObject(nameobject)
    grasp, approach = findBestPose(graspList, arm, grip)
    p.addSolidPrimitive(nameobject, collisionObject.primitives[0], collisionObject.primitive_poses[0], True)
   
    # grasp = graspList[3]
    # approach = findApproach(grasp)
    # p.clear()
    # print(grasp, approach)
 
    if grasp != None: 
        approachTest = arm.moveToPose(approach, grip, max_velocity_scaling_factor = robotSpd, plan_only = False,  planning_time = compTime)
        if approachTest.error_code.val != 1:
            print "IMPOSSIBLE TO REACH THE APPROACH, GOING TO REACH NEXT OBJECT"
            return False
        else:
            print "OBJECT APPROACHED"
            p.removeCollisionObject(nameobject)
            graspTest = arm.moveToPose(grasp, grip, max_velocity_scaling_factor = robotSpd, plan_only = False,  planning_time = compTime)
            if graspTest.error_code.val != 1:
                print "IMPOSSIBLE TO REACH THE OBJECT, GOING TO REACH NEXT OBJECT"
                return False
            else:
                print "OBJECT GRASPED"
                gripper.close()
                return True

    elif grasp == None:
        print "IMPOSSIBLE TO REACH THE OBJECT, GOING TO REACH NEXT OBJECT"
        return False

def isGraspable(collisionObject, arm):
    objectType = SolidPrimitive()
    graspList = []

    if arm == gr:
        testPoses = rightPoses
    elif arm == gl:
        testPoses = leftPoses

    if collisionObject.primitives[0].type == objectType.CYLINDER:
        if collisionObject.primitives[0].dimensions[1] >= 0.06: #range of grippers
            return graspList
        else:
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
            return graspList

    elif collisionObject.primitives[0].type == objectType.BOX:
        return graspList

def findBestPose(graspList, arm, grip):
    globalCost = []
    idx = 1
    
    for grasp in graspList:
        print "TESTING FOR GRASP", idx
        graspCost = testPose(grasp, arm, grip)
        if graspCost != 1000:
            print "TESTING FOR APPROACH", idx
            approach = findApproach(grasp)
            approachCost = testPose(approach, arm, grip)
        else:
            approachCost = 1000
        globalCost.append(graspCost+approachCost)
        idx += 1

    print "GLOBAL COST", globalCost

    if min(globalCost) != 2000:
        idx = np.where(globalCost == min(globalCost))[0][0]
        grasp = graspList[idx]
        approach = findApproach(grasp)
    else:
        grasp = None
        approach = None

    return grasp, approach

def testPose(pose, arm, grip):
    test = arm.moveToPose(pose, grip, max_velocity_scaling_factor = robotSpd, plan_only = True,  planning_time = compTime)
    if test.error_code.val == -1:
        return 1000
    else:
        finalJointPose = test.planned_trajectory.joint_trajectory.points[-1].positions
        cost = np.sum((np.asarray(finalJointPose) - np.asarray(jts_zero))**2)
        return cost

def findApproach(grasp):
    approach = deepcopy(grasp)
    quat = grasp.pose.orientation
    SHIFT = 0.10

    if quat == front or quat == frontOriented:
        approach.pose.position.x = approach.pose.position.x - SHIFT
    elif quat == top:
        approach.pose.position.z = approach.pose.position.z + SHIFT
    elif quat == right_side:
        approach.pose.position.y = approach.pose.position.y - SHIFT
    elif quat == left_side:
        approach.pose.position.y = approach.pose.position.y + SHIFT

    return approach        

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
        print "WARNING! This is a single arm manipulation task"

    # approach=deepcopy(basketpose)
    # approach.pose.position.z = approach.pose.position.z + 0.05

    # arm.moveToPose(approach, grip, max_velocity_scaling_factor = robotSpd, plan_only=False,  planning_time = compTime)
    bskTest = arm.moveToPose(basketpose, grip, max_velocity_scaling_factor = robotSpd, plan_only=False,  planning_time = compTime)  
    gripper.open()
    # arm.moveToPose(approach, grip, max_velocity_scaling_factor = robotSpd, plan_only=False,  planning_time = compTime)

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
            picked = pick(name, collisionObject, side)
            if picked:
                print "READY TO PLACE OBJECT", name
                place(bskpos, side)
                g.moveToJointPosition(jts_both, pos1,  max_velocity_scaling_factor = 0.8, plan_only=False)
            i += 1
   

        pr.disable()
        sortby = 'cumulative'
        ps=pstats.Stats(pr).sort_stats(sortby).print_stats(0.0)

    except rospy.ROSInterruptException:
        pass


########## DIFFERENT MOVE IT COMMANDER ################
# # Initialize the move_group API.
# moveit_commander.roscpp_initialize(sys.argv)
# # Connect the arms to the move group.
# both_arms = moveit_commander.MoveGroupCommander('both_arms')
# right_arm = moveit_commander.MoveGroupCommander('right_arm')
# left_arm = moveit_commander.MoveGroupCommander('left_arm')
# # Allow replanning to increase the odds of a solution.
# right_arm.allow_replanning(True)
# left_arm.allow_replanning(True)
# # Set the arms reference frames.
# right_arm.set_pose_reference_frame('base')
# left_arm.set_pose_reference_frame('base')
# # Create baxter_interface limb instance.
# leftarm = baxter_interface.limb.Limb('left')
# rightarm = baxter_interface.limb.Limb('right')
# # Initialize the planning scene interface.
# p = PlanningSceneInterface("base")
# # Create baxter_interface gripper instance.
# rightgripper = baxter_interface.Gripper('right')
# leftgripper = baxter_interface.Gripper('left')
# rightgripper.calibrate()
# leftgripper.calibrate()
# rightgripper.open()
# leftgripper.open()

# pick    
# approach = deepcopy(objectpose.pose)
# approach.position.z = approach.position.z + 0.20

# # gripper.open()
# p.removeCollisionObject(nameobject)

# right_arm.set_pose_target(approach)
# plan = right_arm.go(wait=True)
# # right_arm.stop()
# right_arm.clear_pose_targets()

#     moveit_commander.roscpp_shutdown()
# # Exit MoveIt.
# moveit_commander.os._exit(0)
# time.sleep(10)