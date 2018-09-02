#!/usr/bin/env python
# ------------------------------------------------------------------------
# A simple position controler for dynamixel motor to spin from -pi/2 to pi/2
# Prepared by : R Saputra
# 2017
# ------------------------------------------------------------------------

# BEGIN ALL
import math
import rospy
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
from lidar_3d_scan.msg import StartEnd

LOOP_RATE = 10                  # In Hz
# ROTATION_ANGLE = math.pi/2*1.2  # In rad
ROTATION_ANGLE = math.pi/2  # In rad
ERROR_THRESH = 0.3              # In rad

feedback = JointState()

def init():
    feedback.current_pos = ROTATION_ANGLE
    feedback.goal_pos = ROTATION_ANGLE
    feedback.error = 0.0


#Callback
def feedback_callback(msg):
    global feedback    
    feedback = msg

def scan():
    rospy.init_node('motor_control')
    controller_name         = rospy.get_param('~controller_name')
    assemble_times_topic    = rospy.get_param('~times_out')
    
    motor_sub = rospy.Subscriber("/" + controller_name + "/state", JointState, feedback_callback)
    motor_pub = rospy.Publisher("/" + controller_name + "/command", Float64, queue_size=1)
    times_pub = rospy.Publisher(assemble_times_topic, StartEnd, queue_size=10)
    
    command_complete = False
    target = ROTATION_ANGLE
    assemble_time = StartEnd()
    assemble_time.start = assemble_time.end = rospy.Time(0)
    
    rate = rospy.Rate(LOOP_RATE)
    while not rospy.is_shutdown():

        if not command_complete:
            # print("IF ERROR target=%f pos=%f" %(target, feedback.current_pos))
            if abs(target - feedback.current_pos) <= ERROR_THRESH:
                command_complete = True
        else:
            assemble_time.start = assemble_time.end
            assemble_time.end   = feedback.header.stamp
            times_pub.publish(assemble_time)

            target = -target
            command_complete = False
            #motor_pub.publish(target)
        
        motor_pub.publish(target)
        rate.sleep()

init()


if (__name__ == "__main__"):
    scan()
