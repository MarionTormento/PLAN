#!/usr/bin/env python  
# ------------------------------------------------------------------------
# Tf broadcaster to transform from the servo frame to the robot base frame
# Prepared by : R Saputra
# 2017
# ------------------------------------------------------------------------

import rospy
import tf
from math import cos, sin
import tf2_ros
import geometry_msgs.msg
from dynamixel_msgs.msg import JointState

LIDAR_OFFSET = 0.03
FRAME_BIAS = 0.45

def publishMotorTf(msg):
    # print "Current position controller_name/state", msg.current_pos
    br  = tf2_ros.TransformBroadcaster()
    t   = geometry_msgs.msg.TransformStamped()
    q   = tf.transformations.quaternion_from_euler(msg.current_pos+FRAME_BIAS, 0, 0)
    
    # Transform from laser frame to joint on motor frame
    t.header.stamp              = msg.header.stamp
    t.header.frame_id           = g_servo_frame
    t.child_frame_id            = g_lidar_frame
    t.transform.translation.x   = 0.10 #0.10
    t.transform.translation.y   = -LIDAR_OFFSET*sin(msg.current_pos+FRAME_BIAS)
    t.transform.translation.z   = LIDAR_OFFSET*cos(msg.current_pos+FRAME_BIAS) # CORRESPONDS TO T
    t.transform.rotation.x      = q[0]
    t.transform.rotation.y      = q[1]
    t.transform.rotation.z      = q[2]
    t.transform.rotation.w      = q[3]    
    br.sendTransform(t)


if __name__ == '__main__':
    global g_servo_frame
    global g_lidar_frame

    rospy.init_node('tf_laser_scan_broadcaster')

    g_servo_frame       = rospy.get_param('~servo_frame')
    g_lidar_frame  = rospy.get_param('~lidar_frame')
    controller_name     = rospy.get_param('~controller_name')

    rospy.Subscriber("/" + controller_name + "/state", JointState, publishMotorTf)
    rospy.spin()