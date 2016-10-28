#!/usr/bin/env python

import yarp as y
import pygame as pg
from time import sleep
from math import atan2, cos, pi, sin, sqrt

#---------------------------Global variables----------------------------------------

#radius: diagonal distance between wheels split in two
radius = 0.35/2 #carrito 

#---------------------------Yarp ports connection-----------------------------------

#starting yarp network
y.Network.init()

#creating output port 1
joystick_output_port_1 = y.BufferedPortBottle()
joystick_output_port_1_name = "/ds3/stick1_y/out1"
joystick_output_port_1.open(joystick_output_port_1_name)
#creating output port 2
joystick_output_port_2 = y.BufferedPortBottle()
joystick_output_port_2_name = "/ds3/stick1_y/out2"
joystick_output_port_2.open(joystick_output_port_2_name)
#creating output port 3
joystick_output_port_3 = y.BufferedPortBottle()
joystick_output_port_3_name = "/ds3/stick1_y/out3"
joystick_output_port_3.open(joystick_output_port_3_name)
#creating output port 4
joystick_output_port_4 = y .BufferedPortBottle()
joystick_output_port_4_name = "/ds3/stick1_y/out4"
joystick_output_port_4.open(joystick_output_port_4_name)


#connecting joystick data to motors
style = y.ContactStyle()
style.persistent = True
stm32_1_input_port_speed_1_name = "/stm32_1/speed/in_1"
stm32_2_input_port_speed_1_name = "/stm32_2/speed/in_1"
stm32_3_input_port_speed_1_name = "/stm32_3/speed/in_1"
stm32_4_input_port_speed_1_name = "/stm32_4/speed/in_1"

y.Network.connect(joystick_output_port_1_name, stm32_1_input_port_speed_1_name, style)
y.Network.connect(joystick_output_port_2_name, stm32_2_input_port_speed_1_name, style)
y.Network.connect(joystick_output_port_3_name, stm32_3_input_port_speed_1_name, style)
y.Network.connect(joystick_output_port_4_name, stm32_4_input_port_speed_1_name, style)


#-------------joystick connection----------------------

pg.init()
pg.joystick.init()
js = pg.joystick.Joystick(0)
js.init()

if True:
    while True:

        #getting position from joystick
        pg.event.get()

        #-------------------------Ommni-directional base algorithm---------------------
        #------------------------------------------------------------------------------

        #velocity vector
        left_x = -js.get_axis(1)
        left_y = js.get_axis(0)

        #rotation vector
        right_x = js.get_axis(2)
        right_y = -js.get_axis(3)

	    #getting speed from velocity vector
        speed= sqrt(left_x**2+left_y**2)

	    #getting angle form velocity vector
        direction = atan2(left_y,left_x) #TODO make sure in which quadrant the angle is 
        angularSpeed = sqrt(right_x**2+right_y**2) 
	
	    #tangencial speed for the wheels
        vTheta = angularSpeed*radius
	
	    #speed for the four wheels
        vel_upper_right = -(speed*cos(direction+(pi/4))-vTheta)
        vel_down_right = -(speed*sin(direction+(pi/4))-vTheta)
        vel_upper_left = speed*sin(direction+(pi/4))+vTheta
        vel_down_left = speed*cos(direction+(pi/4))+vTheta
        #------------------------------------------------------------------------------
        #------------------------------------------------------------------------------

        #stm32 1 -> upper-right
        joystick_output_bottle_1 = joystick_output_port_1.prepare()
        joystick_output_bottle_1.clear()
        joystick_output_bottle_1.addDouble(vel_upper_right)
        joystick_output_port_1.write()

        #stm32 2 -> upper-left
        joystick_output_bottle_2 = joystick_output_port_2.prepare()
        joystick_output_bottle_2.clear()
        joystick_output_bottle_2.addDouble(vel_upper_left)
        joystick_output_port_2.write()

        #stm32 3 -> down_right
        joystick_output_bottle_3 = joystick_output_port_3.prepare()
        joystick_output_bottle_3.clear()
        joystick_output_bottle_3.addDouble(vel_down_right)
        joystick_output_port_3.write()

        #stm32 4 -> down-left
        joystick_output_bottle_4 = joystick_output_port_4.prepare()
        joystick_output_bottle_4.clear()
        joystick_output_bottle_4.addDouble(vel_down_left)
        joystick_output_port_4.write()

        sleep(0.01)

