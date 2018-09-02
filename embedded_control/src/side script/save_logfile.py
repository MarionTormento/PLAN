#!/usr/bin/env python
# ------------------------------------------------------------------------
# Simple script to save the lidar 3d scan as a log file
# Prepared by : Marion
# 2018
# ------------------------------------------------------------------------

import os
import numpy as np

path = "/home/robin/catkin_ws_hands/src/lidar_3d_scan/data"

def create_logfile(name):
    os.chdir(path)
    logfile = open(name, 'w')
    # header = "# Lidar 3D scan log file \n # motor speed: rpm \n # space:"
    # logfile.write(header)

def write_log(name, pos_lidar, cloud):
    # NODE x y z roll pitch yaw
    txt_pos = "NODE " + str(pos_lidar[0]) + ' ' + str(pos_lidar[1]) + ' ' + str(pos_lidar[2]) + ' ' + str(pos_lidar[3]) + ' ' + str(pos_lidar[4]) + ' ' + str(pos_lidar[5]) + '\n'
    txt_cloud = np.transpose(cloud)

    logfile = open(name, 'a')
    logfile.write(txt_pos)

    for i in range(len(txt_cloud)):
        line = txt_cloud[i]
        line = str(line[0]) + ' ' + str(line[1]) + ' ' + str(line[2]) + '\n'
        logfile.write(line)
    logfile.write('\n')
