#!/usr/bin/env python
# ------------------------------------------------------------------------
# GUI to control the motor speed and monitor the motor temperature
# Prepared by : Marion
# 2018
# ------------------------------------------------------------------------

import math
import rospy
from std_msgs.msg import Float64
from dynamixel_msgs.msg import MotorStateList
from Tkinter import *
import tkMessageBox

# Fonts
LARGE_FONT= ("Verdana", 12)
MEDIUM_FONT= ("Verdana", 10)

def temp_callback(msg):
    global motorTemp    
    data = msg.motor_states[0].temperature
    if data > 0:
        motorTemp = float(data)
     
class GUI():
    def __init__(self, master):      
        master.title("Lidar motor control")
        frame = Frame(master) #set main window
        root.geometry('1200x650')
        frame.pack()

        ## Variables      
        self.initSpeed = 0.3
        self.speed = 0
        self.maxSpeedCommand = 1.0
        self.maxTemperature = 70
        self.temperature = StringVar()
        self.temperature.set('--')
        self.currentSpeed = StringVar()
        self.currentSpeed.set(0)
        self.go = False
        self.commandRatio = 80
        self.maxSpeedInit = self.commandRatio*self.maxSpeedCommand
        self.maxSpeed = StringVar()
        self.maxSpeed.set(self.maxSpeedInit)

        ## Global structure
        label_space=Label(frame,text='      \n')
        label_space.grid(row=0, column=0)
        label_space=Label(frame,text=' \n')
        label_space.grid(row=1, column=1)
        label_space=Label(frame,text='\n ')
        label_space.grid(row=2, column=1)
        
        ## Motor speed
        frame_speed= LabelFrame(frame, text="Motor speed control (% MAX)", font=LARGE_FONT, padx=100, pady=50)
        frame_speed.grid(row=1,column=1)   
        subframe_speed = LabelFrame(frame_speed, padx=20, pady=20) 
        subframe_speed.grid(row=1, column = 5, rowspan=3)
        # Space holder
        label_space=Label(frame_speed,text='      \n')
        label_space.grid(row=2, column=0)
        label_space=Label(frame_speed,text='      \n')
        label_space.grid(row=1, column=4)
        label_space=Label(subframe_speed, text='      \n')
        label_space.grid(row=2, column=1)
        # Start
        self.start_button = Button(frame_speed, text="START", bg='green', font=LARGE_FONT, width = 7, command=self.start)
        self.start_button.grid(row = 1, column = 1)
        # Stop
        self.stop_button = Button(frame_speed, text="STOP", bg='red', font=LARGE_FONT, width =7, command=self.stop)
        self.stop_button.grid(row = 1, column = 3)
        # Speed slider
        self.speed_slider = Scale(frame_speed, from_=0, to=100, font=LARGE_FONT, length=330, orient=HORIZONTAL, command=self.sendSpeed)
        self.speed_slider.grid(columnspan=3, row = 3, column = 1)
        # Motor max speed
        self.maxSpeedLbl = Label(subframe_speed, text="Max motor Speed (rpm) ", font=MEDIUM_FONT)
        self.maxSpeedLbl.grid(columnspan=3, row=1, column=1)
        self.maxSpeedBox = Spinbox(subframe_speed, from_=0, to=120, textvariable=self.maxSpeed, width=5, font=MEDIUM_FONT, command=self.updateMaxSpeed)
        self.maxSpeedBox.grid(columnspan = 2, row=1, column=5)
        # Motor speed command
        self.current_spd = Label(subframe_speed, text="Current motor speed is", font=MEDIUM_FONT )
        self.current_spd.grid(columnspan = 4, row = 3, column = 1)
        self.label_spd = Label(subframe_speed, textvariable = self.currentSpeed, font=MEDIUM_FONT)
        self.label_spd.grid(row = 3, column = 5)
        self.current_spd2 = Label(subframe_speed, text="Hz", font=MEDIUM_FONT )
        self.current_spd2.grid(row = 3, column = 6)

        ## Motor temperature
        frame_temp= LabelFrame(frame, text="Motor temperature control", font=LARGE_FONT, padx=100, pady=50)
        frame_temp.grid(row=3,column=1)
        # space holder
        label_space=Label(frame_temp,text='      \n')
        label_space.grid(row=2, column=0)
        # Current Temperature
        self.current_temp = Label(frame_temp, text="Current motor temperature is ", font=MEDIUM_FONT )
        self.current_temp.grid(columnspan = 4, row = 1, column = 2)
        self.label_temp = Label(frame_temp, textvariable = self.temperature, font=MEDIUM_FONT)
        self.label_temp.grid(row = 1, column = 6)
        self.current_temp2 = Label(frame_temp, text="C", font=MEDIUM_FONT )
        self.current_temp2.grid(row = 1, column = 7)
        # Warning message
        self.warning = StringVar()
        self.warning.set(' ')
        self.warning_label = Label(frame_temp, textvariable = self.warning, fg = 'red', font= LARGE_FONT)
        self.warning_label.grid(columnspan = 5, row = 3, column = 2)

    def start(self):
        self.go = True
        self.start_button.config(fg='white', text='ON')
        self.stop_button.config(fg='grey', text='STOP')
        self.maxSpeedBox.config(bg='grey')
        # Update Speed
        self.updateMaxSpeed()
        perc = self.initSpeed/float(self.maxSpeedCommand)*100
        self.speed_slider.set(int(perc))
        # Update Temp
        self.updateTemp()

    def stop(self):
        self.go = False
        self.start_button.config(fg='grey', text='START')
        self.stop_button.config(fg='white', text='OFF')
        self.maxSpeedBox.config(bg='white')
        # Update speed
        self.speed_slider.set(0)
        self.speed = 0

    def updateTemp(self):
        global motorTemp

        if self.go == True :
            self.temperature.set(motorTemp)
            # If overheating, motor is stopped and warning message
            if motorTemp > self.maxTemperature:
                self.warning.set("WARNING ! The motor is overheating !")
                self.warning_label.bg = 'red'
                self.stop()
            root.after(500, self.updateTemp)
        else:
            self.temperature.set('--')

    def updateMaxSpeed(self):
        if self.go == False:
            self.maxSpeedCommand = float(self.maxSpeed.get())/self.commandRatio
        else:
            val = self.maxSpeedCommand*self.commandRatio
            self.maxSpeed.set(val)

    
    def sendSpeed(self, event):
        # Publish speed to the node connected to the motor
        if self.go == True:
            perc = float(self.speed_slider.get())
            self.speed = perc*self.maxSpeedCommand/100
            val = float(int(100*self.speed*self.commandRatio/60))/100
            if self.speed < 1.5: # security to protect motor and lidar
                motorSpeed_pub.publish(self.speed)
            self.currentSpeed.set(val)
        else:
            self.speed_slider.set(0)
            self.speed = 0
            self.currentSpeed.set(0)
            motorSpeed_pub.publish(0)

    def quit_callback(self):
        self.stop()
        if tkMessageBox.askokcancel("Quit", "Do you really wish to quit?"):
            root.destroy()


if (__name__ == "__main__"):
    root = Tk()
    my_gui = GUI(root)
    
    rospy.init_node('lidar_motor_GUI')

    motorTemp_sub = rospy.Subscriber("/motor_states/pan_tilt_port", MotorStateList, temp_callback)
    motorSpeed_pub = rospy.Publisher("/speedcommand", Float64, queue_size=1)

    root.protocol("WM_DELETE_WINDOW", my_gui.quit_callback)

    while not rospy.is_shutdown():
        root.mainloop()