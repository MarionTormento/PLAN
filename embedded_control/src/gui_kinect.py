#!/usr/bin/env python
# ------------------------------------------------------------------------
# GUI to control the motor speed and monitor the motor temperature
# Prepared by : Marion
# 2018
# ------------------------------------------------------------------------

# BEGIN ALL
import math
import rospy
from std_msgs.msg import Float64
# from dynamixel_msgs.msg import MotorStateList
from Tkinter import *
import tkMessageBox
from math import pi

# Fonts
LARGE_FONT= ("Verdana", 12)
MEDIUM_FONT= ("Verdana", 10)
    
class GUI():
    def __init__(self, master):      
        master.title("Kinect motor control")
        frame = Frame(master) #set main window
        root.geometry('600x250')
        frame.pack()

        ## Global structure
        label_space=Label(frame,text='      \n')
        label_space.grid(row=0, column=0)
        
        ## Motor speed
        frame_speed= LabelFrame(frame, text="Kinect motor position control (degree)", font=LARGE_FONT, padx=50, pady=50)
        frame_speed.grid(row=1,column=1)   
        self.label_spd = Label(frame_speed, text = "Move the slider to the desired angle for the kinect", font=MEDIUM_FONT)
        self.label_spd.grid(row = 0, column = 1)
        # Speed slider
        self.speed_slider = Scale(frame_speed, from_=-30, to=90, font=LARGE_FONT, length=300, orient=HORIZONTAL, command=self.sendSpeed)
        self.speed_slider.grid(columnspan=3, row = 3, column = 1)
       
    def sendSpeed(self, event):
        # Publish speed to the node connected to the motor
        degToRad = self.speed_slider.get()*pi/180
        motorPos_pub.publish(degToRad)

    def quit_callback(self):
        motorPos_pub.publish(0)

        if tkMessageBox.askokcancel("Quit", "Do you really wish to quit?"):
            root.destroy()


if (__name__ == "__main__"):
    root = Tk()
    my_gui = GUI(root)
    
    rospy.init_node('kinect_motor_GUI')

    motorPos_pub = rospy.Publisher("/kinect/position/command", Float64, queue_size=1)

    root.protocol("WM_DELETE_WINDOW", my_gui.quit_callback)

    while not rospy.is_shutdown():
        root.mainloop()