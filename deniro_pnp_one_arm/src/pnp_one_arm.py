#!/usr/bin/env python
# ------------------------------------------------------------------------
# Baxter pick and place
# Prepared by : Marion Tormento
# 2018
# ------------------------------------------------------------------------

import rospy
import roslib
from object_detection.msg import obj
import sys
import cProfile, pstats
import time 
import rospy
import roslib; roslib.load_manifest("moveit_python")
import baxter_interface
from moveit_python import PlanningSceneInterface, MoveGroupInterface
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion
from moveit_msgs.msg import CollisionObject #, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from math import pi, sqrt, cos, sin
from copy import deepcopy
import numpy as np
import baxter_pykdl as kdl
from tf.transformations import *

#################################### INITIALISATION ####################################
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

# Constant
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
pos_high = [-1.5992511877797613, 1.3277948504704176, 0.689270903962913, -1.4012007670249345, 0.1754894648887202, 1.8146491089961856, 3.0589327838441367, 1.9602758615925566, 1.4333515313858163, -0.5903707508267928, -1.005913550861348, -0.5306659661076933, 2.051042542353038, -2.677815699467131]
# pos_high = [-1.3525875597179635, 1.1662088939898858, 1.0538448012772792, -1.3380147422330522, 0.24850488763743703, 1.7560245069318274, 3.045335359149518, 1.9190099656446526, 1.3533545501119062, -1.151252581308003, -0.9921020745648913, -0.6787864986392956, 1.8607186957050068, -2.7857091107999112]
# approach = [0.9257574054888472, 1.249810846929641, -0.060975736318445196, -0.6415874645330742, 0.3992185000471789, 1.4243011615516066, -1.7092380929013222]
# jts_zero = [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0] # Neutral
jts_zero = [0.0, (-2.147+1.047)/2, 0.0, (-0.05+2.618)/2, 0.0, (-1.5707+2.094)/2, 0.0] #[right_s0, right_s1, right_e0, right_e1, right_w0, right_w1, right_w2]

# Global variables
objectName = [] # List of collision objects' name
closeRC = [0,0,0] # Coordinates of the close right corner of the table
closeLC = [0,0,0] # Coordinates of the clos left corner of the table

# Definition of the different orientation tested to grasp an object
## Cylinder
# right side
right_side = Quaternion()
right_side.x = -0.653
right_side.y = 0.653
right_side.z = 0.271
right_side.w = 0.271
# left side
left_side = Quaternion()
left_side.x = 0.653
left_side.y = 0.653
left_side.z = -0.271
left_side.w = 0.271
# front flat
front = Quaternion()
front.x = 0.0
front.y = 0.707
front.z = 0.0
front.w = 0.707
# front oriented
frontOriented = Quaternion()
frontOriented.x = -0.924
frontOriented.y = 0.0
frontOriented.z = -0.383
frontOriented.w = 0.0
# top
top = Quaternion()
top.x = 1.0
top.y = 0.0
top.z = 0.0
top.w = 0.0
# list of poses for each arm for the cylinder case
rightPoses = [right_side, front, frontOriented, top]
leftPoses = [left_side, front, frontOriented, top]

## Box
# top
top_box = deepcopy(top)
# side
box_side = deepcopy(right_side)
# front
box_front = deepcopy(front)

#################################### FUNCTIONS ####################################

def addObj(msg):
    global objectName, closeRC, closeLC

    obj = SolidPrimitive()
    obj_ps = PoseStamped()
    obj_ps.header.frame_id = p._fixed_frame

    if msg.name == 'table':
        # Reset the planning scene
        p.clear()
        objectName = []
        # Table dimensions and pose
        obj.dimensions = [msg.dimensions[0], msg.dimensions[1], msg.center[2]+0.65]
        obj.type = obj.BOX
        obj_ps.pose.position.x = msg.center[0]
        obj_ps.pose.position.y = msg.center[1]
        obj_ps.pose.position.z = -0.595+(msg.center[2]+0.65)/2
        obj_ps.pose.orientation.x = msg.quaternion[0]
        obj_ps.pose.orientation.y = msg.quaternion[1]
        obj_ps.pose.orientation.z = msg.quaternion[2]
        obj_ps.pose.orientation.w = msg.quaternion[3]
        # Find the coordinates of the table's corner
        closeRC = [obj_ps.pose.position.x-obj.dimensions[0]/2, obj_ps.pose.position.y-obj.dimensions[1]/2, obj_ps.pose.position.z+obj.dimensions[2]/2]
        closeLC = [obj_ps.pose.position.x-obj.dimensions[0]/2, obj_ps.pose.position.y+obj.dimensions[1]/2, obj_ps.pose.position.z+obj.dimensions[2]/2]

        print "The table has been detected"

    if msg.name[0:8] == 'cylinder':
        # Cylinder dimensions and pose
        obj.dimensions = [msg.dimensions[0], msg.dimensions[1]]
        obj.type = obj.CYLINDER
        obj_ps.pose.position.x = msg.center[0]
        obj_ps.pose.position.y = msg.center[1]
        obj_ps.pose.position.z = -0.595+0.65+msg.center[2]
        obj_ps.pose.orientation.x = msg.quaternion[0]
        obj_ps.pose.orientation.y = msg.quaternion[1]
        obj_ps.pose.orientation.z = msg.quaternion[2]
        obj_ps.pose.orientation.w = -msg.quaternion[3]        
        # Add the object to the list
        if msg.name not in objectName:
            objectName.append(msg.name)
        print "The object", msg.name, "has been detected"

    if msg.name[0:3] == 'box':
        # Box dimensions and pose
        obj.dimensions = [msg.dimensions[0], msg.dimensions[1], msg.dimensions[2]]
        obj.type = obj.BOX
        obj_ps.pose.position.x = msg.center[0]
        obj_ps.pose.position.y = msg.center[1]
        obj_ps.pose.position.z = -0.595+0.65+msg.center[2]
        obj_ps.pose.orientation.x = msg.quaternion[0]
        obj_ps.pose.orientation.y = msg.quaternion[1]
        obj_ps.pose.orientation.z = msg.quaternion[2]
        obj_ps.pose.orientation.w = msg.quaternion[3]
        # Add the object to the list
        if msg.name not in objectName:
            objectName.append(msg.name)
        print "The object", msg.name, "has been detected"

    # Updates the planning scene with the new object
    p.addSolidPrimitive(msg.name, obj, obj_ps.pose, True)

def distance2Corner(x,y, corner1, corner2):
    # Computes the distance between the object and the closest "close" table corner
    distance1 = (x-corner1[0])**2+(y-corner1[1])**2
    distance2 = (x-corner2[0])**2+(y-corner2[1])**2
    distance = min(distance1, distance2)
    return distance

def sortObjects(objectName):
    # Sorted list of object poses from the closest to the further away in x direction
    collisionObjects = [0]*len(objectName)
    objNameOrdered = [0]*len(objectName)
    distance = [] 
    
    for obj in objectName:
        print obj
        exec(obj+"=CollisionObject()")
        collisionObject = p._objects[obj]
        if collisionObjects[0] == 0:
            collisionObjects[0] = collisionObject
            objNameOrdered[0] = obj
            dist = distance2Corner(collisionObject.primitive_poses[0].position.x, collisionObject.primitive_poses[0].position.y, closeRC, closeLC)
            distance.append(dist)
        else:
            newDist = distance2Corner(collisionObject.primitive_poses[0].position.x, collisionObject.primitive_poses[0].position.y, closeRC, closeLC)
            i = 0
            for d in distance:
                if newDist > d:
                    i += 1
                else:
                    collisionObjects[i+1:len(distance)+1] = collisionObjects[i:len(distance)]
                    collisionObjects[i] = collisionObject
                    objNameOrdered[i+1:len(distance)+1] = objNameOrdered[i:len(distance)]
                    objNameOrdered[i] = obj
                    break
                if i == len(distance):
                    collisionObjects[i] = collisionObject
                    objNameOrdered[i] = obj
            distance.append(newDist)
            distance.sort()

    objectName = objNameOrdered
    return objectName, collisionObjects

def pick(nameobject, collisionObject, side):    
    # Defines the move group depending on which arm should be used
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

    # Find the list of possible grasp depending on the shape and dimension of the object
    graspList, approachList = graspAndApproach(collisionObject, arm)
    if len(graspList) != 0:
        # Test if object and approach reachable
        p.removeCollisionObject(nameobject)
        grasp, approach = findBestPose(graspList, approachList, arm, grip) # returns best pose and corresponding approach if it exists
        p.addSolidPrimitive(nameobject, collisionObject.primitives[0], collisionObject.primitive_poses[0], True)
     
        if grasp != None: 
            # Try to approach the object without collision
            approachTest = arm.moveToPose(approach, grip, max_velocity_scaling_factor = robotSpd, plan_only = False,  planning_time = compTime)
            print "APPROACH", approachTest.error_code, "COST", costFunction(approachTest, approach, arm)
            if approachTest.error_code.val == -1 or approachTest.error_code.val == -2:
                print "IMPOSSIBLE TO REACH THE APPROACH, GOING TO REACH NEXT OBJECT"
                return False, grasp
            else:
                # Try to grasp the object
                print "OBJECT APPROACHED"
                p.removeCollisionObject(nameobject)
                graspTest = arm.moveToPose(grasp, grip, max_velocity_scaling_factor = robotSpd, plan_only = False,  planning_time = compTime)
                print "GRASP", graspTest.error_code, "COST", costFunction(graspTest, grasp, arm)
                if graspTest.error_code.val == -1 or graspTest.error_code.val == -2:
                    # If it was not possible to grasp the object, leave it in the planning scene
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

def graspAndApproach(collisionObject, arm):
    objectType = SolidPrimitive()
    GRIP_SIZE = 0.1 # size of the open grippers
    SHIFT = 0.10 # distance between approach and grasp  
    graspList = []
    approachList = []
    # The possible grasp depends on the type of the object (BOX or CYLINDER)
    if collisionObject.primitives[0].type == objectType.CYLINDER:
        # Poses depends on the arm group used
        if arm == gr:
            testPoses = rightPoses
        elif arm == gl:
            testPoses = leftPoses
        # If the diameter of the cylinder is smaller than the size of the grippers
        if collisionObject.primitives[0].dimensions[1] <= GRIP_SIZE:        
            for quat in testPoses:
                grasp = PoseStamped()
                grasp.header = deepcopy(collisionObject.header)
                grasp.pose = deepcopy(collisionObject.primitive_poses[0])
                grasp.pose.orientation = quat
                # Fine tuning of the grasp position
                if quat == right_side: 
                    grasp.pose.position.x = 0.97*grasp.pose.position.x 
                    grasp.pose.position.y += min((collisionObject.primitives[0].dimensions[1]),0.05) #account for object being too tall most of the time
                    grasp.pose.position.z -= 0.5*(collisionObject.primitives[0].dimensions[0]) #account for object being too tall most of the time
                    approach = deepcopy(grasp)
                    approach.pose.position.y -= SHIFT
                elif quat == left_side: 
                    grasp.pose.position.z -= 0.5*(collisionObject.primitives[0].dimensions[0]) #account for object being too tall most of the time
                    grasp.pose.position.y -= min((collisionObject.primitives[0].dimensions[1]),0.05) #account for object being too tall most of the time
                    approach = deepcopy(grasp)
                    approach.pose.position.y += SHIFT
                elif quat == front: 
                    grasp.pose.position.y = 0.975*grasp.pose.position.y 
                    approach = deepcopy(grasp)
                    approach.pose.position.x -= SHIFT       
                elif quat == frontOriented:
                    grasp.pose.position.y = 0.975*grasp.pose.position.y 
                    grasp.pose.position.z -= 0.5*(collisionObject.primitives[0].dimensions[0]) #account for object being too tall most of the time
                    approach = deepcopy(grasp)
                    approach.pose.position.x -= SHIFT
                elif quat == top: 
                    grasp.pose.position.z += collisionObject.primitives[0].dimensions[0]/2 - 0.08
                    grasp.pose.position.x = 0.97*grasp.pose.position.x
                    if grasp.pose.position.y < 0:
                        grasp.pose.position.y = 0.96*grasp.pose.position.y
                    else:
                        grasp.pose.position.y = 1.04*grasp.pose.position.y
                    approach = deepcopy(grasp)
                    approach.pose.position.z += SHIFT
                graspList.append(grasp)
                approachList.append(approach)
    # Box grasp is not fined tuned yet !
    elif collisionObject.primitives[0].type == objectType.BOX:
        grasp_top = PoseStamped()
        grasp_top.header = deepcopy(collisionObject.header)
        grasp_top.pose = deepcopy(collisionObject.primitive_poses[0])
        grasp_side = deepcopy(grasp_top)
        grasp_front = deepcopy(grasp_top)
        # Orientation of the object in the base frame
        quat = [grasp_top.pose.orientation.x, grasp_top.pose.orientation.y, grasp_top.pose.orientation.z, grasp_top.pose.orientation.w]
        euler = euler_from_quaternion(quat)
        yaw = -(euler[2]+pi/2)
        # If the object is small enough to be grasped
        if collisionObject.primitives[0].dimensions[0] <= GRIP_SIZE:
            # top
            quat_top = quaternion_from_euler(pi, 0.0, yaw, axes='rxyz')
            top_box.x = quat_top[0]
            top_box.y = quat_top[1]
            top_box.z = quat_top[2]
            top_box.w = quat_top[3]
            grasp_top.pose.position.z += 0.5*collisionObject.primitives[0].dimensions[2] - 0.08#account for object being too tall most of the time
            grasp_top.pose.position.x = 0.97*grasp_top.pose.position.x
            grasp_top.pose.position.y = 0.97*grasp_top.pose.position.y
            grasp_top.pose.orientation = top_box
            approach_top = deepcopy(grasp_top)
            approach_top.pose.position.z += SHIFT
            graspList.append(grasp_top)
            approachList.append(approach_top)
            # side
            quat_side = quaternion_from_euler(pi, yaw, pi/2, axes='rxzy')#
            box_side.x = quat_side[0]
            box_side.y = quat_side[1]
            box_side.z = quat_side[2]
            box_side.w = quat_side[3]
            grasp_side.pose.orientation = box_side
            grasp_top.pose.position.x = 0.95*grasp_top.pose.position.x
            grasp_top.pose.position.y = 0.95*grasp_top.pose.position.y
            approach_side = deepcopy(grasp_side)
            euler_side = euler_from_quaternion(quat_side)
            angle_side = euler_side[0]
            approach_side.pose.position.x += SHIFT*sin(angle_side) 
            approach_side.pose.position.y += SHIFT*cos(angle_side)
            graspList.append(grasp_side) 
            approachList.append(approach_side)
        if collisionObject.primitives[0].dimensions[1] <= GRIP_SIZE:
            # front
            quat_front = quaternion_from_euler(pi, yaw-pi/2, pi/2, axes='rxzy')
            box_front.x = quat_front[0]
            box_front.y = quat_front[1]
            box_front.z = quat_front[2]
            box_front.w = quat_front[3]
            grasp_front.pose.orientation = box_front
            grasp_top.pose.position.x = 0.96*grasp_top.pose.position.x
            grasp_top.pose.position.y = 0.96*grasp_top.pose.position.y
            approach_front = deepcopy(grasp_front)
            euler_front = euler_from_quaternion(quat_front)
            angle_front = euler_front[0]
            approach_front.pose.position.x += SHIFT*sin(angle_front) 
            approach_front.pose.position.y += SHIFT*cos(angle_front)
            graspList.append(grasp_front) 
            approachList.append(approach_front)
           
    return graspList, approachList

def findBestPose(graspList, approachList, arm, grip):
    globalCost = []
    idx = 1

    for grasp, approach in zip(graspList, approachList):
        print "TESTING FOR GRASP", idx
        graspCost = testPose(grasp, arm, grip)
        if graspCost != 10000: # grasp is reachable without collision
            print "TESTING FOR APPROACH", idx
            approachCost = testPose(approach, arm, grip)
        else:
            approachCost = 10000
        globalCost.append(graspCost+approachCost)
        idx += 1
    print "GLOBAL COST", globalCost
    # If it exists at least one reachable grasp
    if min(globalCost) != 20000:
        idx = np.where(globalCost == min(globalCost))[0][0]
        grasp = graspList[idx]
        approach = approachList[idx]
    else:
        grasp = None
        approach = None

    return grasp, approach

def testPose(pose, arm, grip, **kwargs):
    test = arm.moveToPose(pose, grip, max_velocity_scaling_factor = robotSpd, plan_only = True,  planning_time = compTime)

    # if plan failed (-1: No solution / -2: Solution with collision)
    if test.error_code.val == -1 or test.error_code.val == -2:
        return 10000
    # if plan success
    else:
        cost = costFunction(test, pose, arm)
        return cost

def costFunction(trajectory, pose, arm):
    poseFct = 200 # factor for the influence of the cost pose
    jointFct = 1 # factor for the influence of the cost joint
    costPose = 0
    costJoints = 0
    finalPose = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
    for point in trajectory.planned_trajectory.joint_trajectory.points:
        joint = point.positions
        currentPose = endPose(joint, arm)
        currentPose = currentPose[0:3]
        # How far away from the final pose do you go during your movement (order of 1)
        costPose += np.sum((finalPose - currentPose)**2)
        # How close to the joint limits do you get (order of 100)
        costJoints += np.sum((np.asarray(joint)-np.asarray(jts_zero))**2)
    # Final cost (order of 100 to 1000)
    cost = poseFct*costPose + jointFct*costJoints
    print "COST POSE", poseFct*costPose, "COST JOINTS", jointFct*costJoints, "COST TOTAL", cost
    return cost

def endPose(joints, arm):
    # Setup a dictionary of the joints value to communicate with pyKDL
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
    # Return the pose in base frame for the corresponding joint angles
    correspondingPose = solver.forward_position_kinematics(jointsdico)
    return correspondingPose

# # For future work
# def attachObject2Robot(name, collisionObject, link_name):
#     collisionObject.primitives[0].dimensions[0] = collisionObject.primitives[0].dimensions[0]/2
#     collisionObject.header.frame_id = link_name
#     collisionObject.primitive_poses[0].position.x = 0
#     collisionObject.primitive_poses[0].position.y = 0
#     collisionObject.primitive_poses[0].position.z = collisionObject.primitives[0].dimensions[0]+0.02
#     collisionObject.primitive_poses[0].orientation.x = 0
#     collisionObject.primitive_poses[0].orientation.y = 0
#     collisionObject.primitive_poses[0].orientation.z = 0    
#     collisionObject.primitive_poses[0].orientation.w = 1    
#     a = p.makeAttached(link_name, collisionObject, touch_links=None, detach_posture=None, weight=0.0)
#     p._attached_objects[name] = a
#     p.sendUpdate(None, a, use_service=True)

def place(side, currentPose):   
    # Defines the move group depending on which arm should be used
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

    # Retreat is a upper positon to avoid collision between the grasped object and the rest of the planning scene
    retreat =deepcopy(currentPose)
    retreat.pose.position.z = retreat.pose.position.z + 0.20
    retreatTest = arm.moveToPose(retreat, grip, max_velocity_scaling_factor = robotSpd, plan_only=False,  planning_time = compTime)
    print "RETREAT", retreatTest.error_code, "COST", costFunction(retreatTest, retreat, arm)
    # Basket can be any pose set at the beggining of the script    
    bskTest = arm.moveToPose(bskpos, grip, max_velocity_scaling_factor = robotSpd, plan_only=False,  planning_time = compTime)  
    gripper.open()
    print "RETURN", bskTest.error_code, "COST", costFunction(bskTest, bskpos, arm)

#################################### MAIN ####################################

if __name__=='__main__':
    try:
        global objectName, closeRC, closeLC
        object_topic       = rospy.get_param('~object_topic')
        rospy.Subscriber(object_topic, obj, addObj)
        
        # cProfile to measure the performance (time) of the task.
        pr = cProfile.Profile()
        pr.enable()    
        rightgripper.calibrate()
        leftgripper.calibrate()
        rightgripper.open()
        leftgripper.open()
        g.moveToJointPosition(jts_both, pos_high,  max_velocity_scaling_factor = 0.5, plan_only=False)     
        # Awaits for the planning scene to build correctly
        time.sleep(10)
        print "There is", len(objectName), "object(s) detected on the table"
        # Sort the object list
        objectName, collisionObjects = sortObjects(objectName)
        # Pick and place loop    
        i = 0        
        for collisionObject in collisionObjects:
            name = objectName[i]
            if collisionObject.primitive_poses[0].position.y < 0:
                side = 'right'
            else:
                side = 'left'
            print "READY TO PICK UP OBJECT", name
            picked, grasp = pick(name, collisionObject, side)
            if picked:
                print "READY TO PLACE OBJECT", name
                place(side, grasp)
                g.moveToJointPosition(jts_both, pos_high,  max_velocity_scaling_factor = 0.8, plan_only=False)
            rightgripper.open()
            leftgripper.open()
            i += 1
        # Profiling
        pr.disable()
        sortby = 'cumulative'
        ps=pstats.Stats(pr).sort_stats(sortby).print_stats(0.0)

    except rospy.ROSInterruptException:
        pass