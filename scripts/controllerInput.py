#!/usr/bin/env python

from __future__ import print_function
import pygame

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int16MultiArray

def pub_control_cmd():
	pygame.init()
	pygame.joystick.init()

	clock = pygame.time.Clock()

	axis_control_pub = rospy.Publisher('/Gamepad/Axis', Float64MultiArray)
	
	button_control_pub = rospy.Publisher('/Gamepad/Buttons', Int16MultiArray)

	print('Number of joysticks found: ', pygame.joystick.get_count())
	gamepad = pygame.joystick.Joystick(0)       # Use first joystick
	gamepad.init()
	print('Name of first Joystick: ', gamepad.get_name())
	print('Detected axis: ', gamepad.get_numaxes())
	print('Detected buttons: ', gamepad.get_numbuttons())

	axisDataGamepad = Float64MultiArray()
	buttonDataGamepad = Int16MultiArray()
	pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
	button = np.array([0, 0, 0, 0, 0, 0, 0, 0])
	rospy.init_node('kit_head_control_joystick')

	while not rospy.is_shutdown():
	    pygame.event.pump()                   # Needed to handle pygame events
	    lh = gamepad.get_axis(0)                # Left horizontal
	    lv = gamepad.get_axis(1)                # Left vertical
	    rh = gamepad.get_axis(3)                # Right horizontal
	    rv = gamepad.get_axis(4)                # Right vertical
	    sl = gamepad.get_axis(2)                # Shoulder left
	    sr = gamepad.get_axis(5)                # Shoulder right
# 	    print(lh, lv, rh, rv, sl, sr)
	    ab = gamepad.get_button(0)				# A button pressed
	    bb = gamepad.get_button(1)				# B button pressed
	    xb = gamepad.get_button(2)				# X button pressed
	    yb = gamepad.get_button(3)				# Y button pressed
	    lb = gamepad.get_button(4)				# left shoulder button pressed
	    rb = gamepad.get_button(5)				# right shoulder button pressed
	    backb = gamepad.get_button(6)			# back button pressed
	    startb = gamepad.get_button(7)			# startbutton pressed
# 	    print(ab, bb, xb, yb, lb, rb, sb, tb)
	    
	    pose[0] = lh
	    pose[1] = lv
	    pose[2] = rh
	    pose[3] = rv
	    pose[4] = sl
	    pose[5] = sr
	    button[0] = ab
	    button[1] = bb
	    button[2] = xb
	    button[3] = yb
	    button[4] = lb
	    button[5] = rb
	    button[6] = backb
	    button[7] = startb  
	    
	    axisDataGamepad.data = pose
	    buttonDataGamepad.data = button
	    axis_control_pub.publish(axisDataGamepad)
	    button_control_pub.publish(buttonDataGamepad)
	    
	    clock.tick(10)                       # Limit execution to 10hz

if __name__ == '__main__':
	try:
		pub_control_cmd()
	except rospy.ROSInterruptException:
		pass
