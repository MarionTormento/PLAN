#!/usr/bin/env python
# ------------------------------------------------------------------------
# Node to collect and publish the lidar 2d scan and motor states to prepare the 3d point cloud
# Prepared by : Marion
# 2018
# ------------------------------------------------------------------------

# BEGIN ALL
import rospy
import rosbag
from dynamixel_msgs.msg import JointState
from sensor_msgs.msg import LaserScan
import thread
import time
import numpy as np
from save_logfile import *

motor_roll = 0.0
ranges = []
angle_min = 0.0
angle_max = 1.0
angle_increment = 0.1
offset = 0.03

name_file = time.strftime("%Y%m%d-%H%M%S") + "_3d.log"

#Callback
def motor_callback(msg):
    global motor_roll
    motor_roll = msg.current_pos

def cloud_callback(msg):
    global ranges, angle_max, angle_min, angle_increment
    ranges = msg.ranges
    angle_max = msg.angle_max
    angle_min = msg.angle_min
    angle_increment = msg.angle_increment

def spin_loop(msg):
    print("ROS is spinning")
    rospy.spin()

if (__name__ == "__main__"):
    global ranges, motor_roll, angle_max, angle_min, angle_increment

    rospy.init_node('lidar_3dmapping')
    controller_name         = rospy.get_param('~controller_name')

    motor_sub = rospy.Subscriber("/" + controller_name + "/state", JointState, motor_callback) # to know the joint state at time t
    cloud_sub = rospy.Subscriber("/robot/laser/scan", LaserScan, cloud_callback) # to know the point cloud at time t
    # robot_sub = ... to know the position of deNiro
    # octo_pub = rospy.Publisher("/octo") # send the point cloud in the correct way to octomap

    spin_thread = thread.start_new_thread(spin_loop, (" ",))

    create_logfile(name_file)
    
    cloud = np.zeros((3,1081))
    pos_lidar = np.zeros(6)

    while not rospy.is_shutdown():
        if len(ranges)>0: # Security to avoid collecting empty data during init
            # Project from polar coordinate to cartesian coordinates in the lidar plane
            ranges = np.asarray(ranges)
            theta = np.arange(angle_min, angle_max, angle_increment)
            cloud[0] = ranges*np.cos(theta)
            cloud[1] = ranges*np.sin(theta)
            # Position of the lidar center in the world frame
            pos_lidar[1] = offset*np.cos(motor_roll)
            pos_lidar[2] = offset*np.sin(motor_roll)
            pos_lidar[3] = motor_roll

            if pos_lidar[3] != 0.0: # Security to avoid collecting empty data during init
                write_log(name_file, pos_lidar, cloud)
        else:
            pass



