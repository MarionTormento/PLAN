#!/usr/bin/env python  
# ------------------------------------------------------------------------
# Tf broadcaster to transform from the kinect servo frame to the kinect frame
# Prepared by : R Saputra
# Modified by : M Tormento
# 2018
# ------------------------------------------------------------------------

import rospy
import tf
from math import cos, sin
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import JointState
from dynamixel_workbench_msgs.srv import JointCommand
from std_msgs.msg import Float64

MOTOR_OFFSET = 1.5708

def moveMotor(msg):
    cmd = MOTOR_OFFSET - msg.data
    try:
        joint_command('rad', 1, cmd)
    except rospy.ServiceException, e:
        print "service call failed"

def publishMotorTf(msg):
    angle = msg.position[0] - MOTOR_OFFSET
    q   = tf.transformations.quaternion_from_euler(angle, 0, 0)
    br = tf.TransformBroadcaster()
    br.sendTransform((0, 0, 0), 
                     q,
                     rospy.Time.now(),
                     g_kinect_frame,
                     g_servo_frame)


if __name__ == '__main__':
    global g_servo_frame
    global g_robot_base_frame

    rospy.init_node('tf_kinect_broadcaster')
    rospy.wait_for_service('joint_command')

    g_servo_frame       = rospy.get_param('~servo_frame')
    g_kinect_frame  = rospy.get_param('~kinect_frame')

    rospy.Subscriber("/joint_states", JointState, publishMotorTf)
    rospy.Subscriber("/kinect/position/command", Float64, moveMotor)
    joint_command = rospy.ServiceProxy('joint_command', JointCommand)

    rospy.spin()