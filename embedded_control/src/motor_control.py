#!/usr/bin/env python
# ------------------------------------------------------------------------
# A simple speed controler for dynamixel motor to rotate continuously
# Prepared by : Petar and Nicholas
# Modified by : Marion
# 2018
# ------------------------------------------------------------------------

# BEGIN ALL
import math
import rospy
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
from embedded_control.msg import StartEnd
import thread
from dynamixel_msgs.msg import MotorStateList
import gui_lidar

LOOP_RATE = 10                  # In Hz
# ROTATION_ANGLE = math.pi/2      # In rad
# ERROR_THRESH = 0.3              # In rad
MOTOR_SPEED = 0.3
feedback = JointState()


#Callback
def feedback_callback(msg):
    global feedback
    feedback = msg

def speed_callback(msg):
    global MOTOR_SPEED
    MOTOR_SPEED = msg.data
    
def scan():
    global feedback

    rospy.init_node('motor_control')
    assemble_times_topic    = rospy.get_param('~times_out')

    rospy.wait_for_service('joint_command')

    rospy.Subscriber("/kinect/position/command", Float64, moveMotor)
    joint_command = rospy.ServiceProxy('joint_command', JointCommand)
    times_pub = rospy.Publisher(assemble_times_topic, StartEnd, queue_size=10)

    assemble_time = StartEnd()
    assemble_time.start = assemble_time.end = rospy.Time(0)

    rate = rospy.Rate(LOOP_RATE)

    while not rospy.is_shutdown():
        # Follow the motor speed command
        motor_pub.publish(MOTOR_SPEED)
        # Send time for the laser assembler
        assemble_time.start = assemble_time.end
        assemble_time.end   = feedback.header.stamp
        times_pub.publish(assemble_time)
        rate.sleep()

if (__name__ == "__main__"):

    # init()
    scan()

#     LOOP_RATE = 10                  # In Hz
# # ROTATION_ANGLE = math.pi/2      # In rad
# # ERROR_THRESH = 0.3              # In rad
# MOTOR_SPEED = 0.3
# feedback = JointState()


# #Callback
# def feedback_callback(msg):
#     global feedback
#     feedback = msg

# def speed_callback(msg):
#     global MOTOR_SPEED
#     MOTOR_SPEED = msg.data
    
# def scan():
#     global feedback

#     rospy.init_node('motor_control')
#     controller_name         = rospy.get_param('~controller_name')
#     assemble_times_topic    = rospy.get_param('~times_out')
#     rospy.Subscriber("/kinect/position/command", Float64, moveMotor)

#     motor_sub = rospy.Subscriber("/" + controller_name + "/state", JointState, feedback_callback)
#     motorSpeed_sub = rospy.Subscriber("/speedcommand", Float64, speed_callback)
#     motor_pub = rospy.Publisher("/" + controller_name + "/command", Float64, queue_size=1)
#     times_pub = rospy.Publisher(assemble_times_topic, StartEnd, queue_size=10)

#     # command_complete = False
#     # target = ROTATION_ANGLE
#     assemble_time = StartEnd()
#     assemble_time.start = assemble_time.end = rospy.Time(0)

#     rate = rospy.Rate(LOOP_RATE)

#     while not rospy.is_shutdown():
#         # Follow the motor speed command
#         motor_pub.publish(MOTOR_SPEED)
#         # Send time for the laser assembler
#         assemble_time.start = assemble_time.end
#         assemble_time.end   = feedback.header.stamp
#         times_pub.publish(assemble_time)
#         rate.sleep()

# if (__name__ == "__main__"):

#     # init()
#     scan()

