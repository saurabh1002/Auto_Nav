#!/usr/bin/env python

##############################################################################################
# CODE TO MAP ROBOT VELOCITIES TO MOTOR PWM
# THE PWM VALUES ARE SENT VIA ROSSERIAL TO THE BASE MICROCONTROLLER
##############################################################################################

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray
pwmL = 0
pwmR = 0

##############################################################################################

def sign(x):
	if x > 0:
		return 1
	elif x == 0:
		return 0
	else:
		return -1

def cmd_vel_cb(cmd):
	lin_x = cmd.linear.x													# robot velocities
	ang_z = cmd.angular.z
	pwmR_int = (lin_x + 0.35*ang_z)*79.1									# computing individual motor speeds
	pwmL_int = (lin_x - 0.35*ang_z)*79.1
	pwmR = 85*sign(pwmR_int) + 0.3*(pwmR_int)								# pwm mapping depending on architecture and requirement
	pwmL = 85*sign(pwmL_int) + 0.3*(pwmL_int)
	pwm_values.data = [pwmL,pwmR]
	pub.publish(pwm_values)

################################################################################################

if __name__ == '__main__':
	rospy.init_node('auto_nav', anonymous=True)
	rospy.Subscriber("/cmd_vel", Twist , cmd_vel_cb)						# subscribing to command velocity topic published by move_base
	pub = rospy.Publisher("/motor_pwm", Int16MultiArray , queue_size = 10)
	pwm_values = Int16MultiArray()
	rospy.spin()
