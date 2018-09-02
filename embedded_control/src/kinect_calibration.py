#!/usr/bin/env python  
# ------------------------------------------------------------------------
# Listens to the corner points of the obj in the kinect frame and project them
# in the base frame, so that the obj is ready to be added to moveit
#
# Prepared by : M Tormento
# 2018
# ------------------------------------------------------------------------

import rospy
import numpy as np 
from std_msgs.msg import Float64MultiArray
from pcl_detection.msg import point
from baxter_core_msgs.msg import EndpointState
import time
import tf
from math import atan2, sqrt

# Parameters
Nmax = 10
N = 0
robot_points_3d = None
kinect_points_3d = None

def get_position(msg):
	# Get kinect position
	global kinect_points_3d, new_pos_robot, robot_points_3d, N

	x_k = msg.coords[0]
	y_k = msg.coords[1]
	z_k = msg.coords[2]

	new_pos_kinect = np.matrix([[x_k],[y_k],[z_k]])
	if kinect_points_3d is None:
		kinect_points_3d = new_pos_kinect
	else:
		kinect_points_3d = np.concatenate((kinect_points_3d, new_pos_kinect), 0)

	# Get robot position
	if N < 5:
		(trans, rot) = tfListener.lookupTransform('/base', '/left_null_gripper', rospy.Time(0))
	else:
		(trans, rot) = tfListener.lookupTransform('/base', '/right_null_gripper', rospy.Time(0))


	new_pos_robot = np.matrix([[trans[0]],[trans[1]],[trans[2]]])

	if robot_points_3d is None:
		robot_points_3d = new_pos_robot
	else:
		robot_points_3d = np.concatenate((robot_points_3d, new_pos_robot), 0)

	N += 1

def computeMatrix(points3d):
	matrix = None
	zeros = np.matrix([[0, 0, 0, 0]])
	ones = np.matrix([[1]])
	nbPoints = int(kinect_points_3d.shape[0]/3)

	for i in range(nbPoints):
		point = points3d[3*i:3*(i+1)][:].T
		line1 = np.concatenate((point, ones, zeros, zeros), 1)
		line2 = np.concatenate((zeros, point, ones, zeros), 1)
		line3 = np.concatenate((zeros, zeros, point, ones), 1)
		new_matrix = np.concatenate((line1, line2, line3), 0)
		if matrix is None:
			matrix = new_matrix
		else:
			matrix = np.concatenate((matrix, new_matrix), 0)
	return matrix

def run():
	global N, kinect_points_3d, robot_points_3d
	
	while N < Nmax:
		time.sleep(1)

	Kmatrix = computeMatrix(kinect_points_3d)
	coeff = np.linalg.pinv(Kmatrix)*robot_points_3d#

	coeff = np.reshape(coeff, (3, 4))
	R = coeff[0:3,0:3]
	print(coeff)

	yaw = atan2(R[2,1], R[2,2])
	pitch = atan2(-R[2,0], sqrt(R[2,1]**2+R[2,2]**2))
	roll = atan2(R[1,0], R[0,0])

	print(roll, pitch, yaw)
	
	N = 0
	robot_points_3d = None
	kinect_points_3d = None	


if __name__ == '__main__':

	rospy.init_node('Kinect_calibration')

	tfListener = tf.TransformListener()
	subPCL = rospy.Subscriber('/kinect_calibration/PCLPointXYZ', point, get_position)

	rate = rospy.Rate(10)

   	while not rospy.is_shutdown():
		run()