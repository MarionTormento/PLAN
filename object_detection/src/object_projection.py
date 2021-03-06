#!/usr/bin/env python  
# ------------------------------------------------------------------------
# Listens to the corner points of the obj in the kinect frame and project them
# in the base frame, so that the obj is ready to be added to moveit
#
# Prepared by : M Tormento
# 2018
# ------------------------------------------------------------------------

import roslib
import rospy
import math
import tf
import std_msgs.msg
from sensor_msgs.msg import PointCloud2
from pcl_detection.msg import obj, box
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped

def projectObject(msg):
    # Listen to the transform and create the transform Matrix
    h = std_msgs.msg.Header()
    h.frame_id = g_kinect_frame 
    h.stamp = rospy.Time.now()

    tfListener.waitForTransform(g_base_frame, g_kinect_frame, rospy.Time.now(), rospy.Duration(2.0))

    try:      
        # M routine
        M = tfListener.asMatrix(g_base_frame, h)  
        M = np.asarray(M)
        R = M[0:3,0:3]
        val = [msg.center[0], msg.center[1], msg.center[2], 1]
        val = M.dot(val)
        msg.center = val[0:3]
        val = tf.transformations.euler_from_quaternion(msg.quaternion)
        val = R.dot(val)
        yaw = -(val[1]-1.5708)
        msg.quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        objPub.publish(msg)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print e

def projectBox(msg):
    # Listen to the transform and create the transform Matrix
    h = std_msgs.msg.Header()
    h.frame_id = g_kinect_frame
    h.stamp = rospy.Time.now()
    
    tfListener.waitForTransform(g_base_frame, g_kinect_frame, rospy.Time.now(), rospy.Duration(2.0))

    try:      
        # M routine
        M = tfListener.asMatrix(g_base_frame, h)  
        M = np.asarray(M)
        R = M[0:3,0:3]
        val = [msg.center1[0], msg.center1[1], msg.center1[2], 1]
        val = M.dot(val)
        msg.center1 = val[0:3]
        val = [msg.center2[0], msg.center2[1], msg.center2[2], 1]
        val = M.dot(val)
        msg.center2 = val[0:3]
        
        box = obj()
        box.name = msg.name
        
        if msg.center1[2] > msg.center2[2]:
            # Center 1 is on the top plane
            box.center = [msg.center1[0], msg.center1[1], msg.center2[2]]
            val = tf.transformations.euler_from_quaternion(msg.quaternion1)
            val = R.dot(val)
            yaw = -(val[1]-1.5708)
            box.quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
            w = msg.dimensions1[0]
            h = msg.dimensions1[1]
            delta= []
            for i in range(2):
                delta.append(abs(msg.dimensions2[i]-w))
                delta.append(abs(msg.dimensions2[i]-h))         
            minIdx = np.where(delta == np.amin(delta))
            if minIdx==0 or minIdx==1:
                box.dimensions = [w, h, msg.dimensions2[1]]
            else:
                box.dimensions = [w, h, msg.dimensions2[0]]
        else:
            # Center 2 is on the top plane
            box.center = [msg.center2[0], msg.center2[1], msg.center1[2]]
            val = tf.transformations.euler_from_quaternion(msg.quaternion2)
            val = R.dot(val)
            yaw = -(val[1]-1.5708)
            box.quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
            w = msg.dimensions2[0]
            h = msg.dimensions2[1]
            delta= []
            for i in range(2):
                delta.append(abs(msg.dimensions1[i]-w))
                delta.append(abs(msg.dimensions1[i]-h))          
            minIdx = np.where(delta == np.amin(delta))
            if minIdx==0 or minIdx==1:
                box.dimensions = [w, h, msg.dimensions1[1]]
            else:
                box.dimensions = [w, h, msg.dimensions1[0]]
        objPub.publish(box)

            
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print e

if __name__ == '__main__':
    rospy.init_node('obj_constructor')

    g_kinect_frame  = rospy.get_param('~kinect_frame')
    g_base_frame       = rospy.get_param('~robot_base')

    tfListener = tf.TransformListener()

    objSub = rospy.Subscriber("/pcl/object/kinect_frame", obj, projectObject)
    boxSub = rospy.Subscriber("/pcl/box/kinect_frame", box, projectBox)
    objPub = rospy.Publisher("/pcl/object/base_frame", obj, queue_size=1)

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        rate.sleep()