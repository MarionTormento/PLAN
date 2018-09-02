#!/usr/bin/env python
# ------------------------------------------------------------------------
# Saves only the plan lidar in polar without projection
# Prepared by : Marion
# 2018
# ------------------------------------------------------------------------

# BEGIN ALL
import rospy
from dynamixel_msgs.msg import JointState
from sensor_msgs.msg import LaserScan
import thread
import numpy as np
from save_logfile import *

motor_roll = 0.0
ranges = []
angle_min = 0.0
angle_max = 1.0
angle_increment = 0.1
offset = 1

name_file = time.strftime("%Y%m%d-%H%M%S") + "_static.log"

def cloud_callback(msg):
    global ranges, angle_max, angle_min, angle_increment
    ranges = msg.ranges
    angle_max = msg.angle_max
    angle_min = msg.angle_min
    angle_increment = msg.angle_increment

if (__name__ == "__main__"):
    global ranges, motor_roll, angle_max, angle_min, angle_increment

    rospy.init_node('lidar_staticmapping')

    cloud_sub = rospy.Subscriber("/scan", LaserScan, cloud_callback) # to know the point cloud at time t
    # robot_sub = ... to know the position of deNiro
    # octo_pub = rospy.Publisher("/octo") # send the point cloud in the correct way to octomap

    create_logfile(name_file)

    cloud = np.zeros((3,1081))
    pos_lidar = np.zeros(6)

    while not rospy.is_shutdown():
        if len(ranges)>0:
            ranges = np.asarray(ranges)
            cloud[0] = ranges#*np.cos(theta)
            cloud[1] = ranges#*np.sin(theta)

            write_log(name_file, pos_lidar, cloud)
        else:
            pass
