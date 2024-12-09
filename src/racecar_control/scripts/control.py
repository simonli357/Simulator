#!/usr/bin/env python

import rospy

from utils.msg import drive_param
from utils.msg import pid_input
import math
import numpy as np

pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

kp = 10.
kd = 0.01
kp_vel = 42.0
kd_vel = 0.0

ki = 0.0
servo_offset = 18.0*math.pi/180
prev_error = 0.0 
error = 0.0
integral = 0.0
vel_input = 1.0


def control(data):
	global integral
	global prev_error
	global vel_input
	global kp
	global ki
	global kd
	global kd_vel
	global kp_vel
	velocity = data.pid_vel
	angle = servo_offset
	error = 5*data.pid_error
	if error != 0.0:
		control_error = kp*error + kd*(error - prev_error)
		angle = angle + control_error*np.pi/180

		control_error_vel = kp_vel*error + kd_vel*(error - prev_error)
		velocity = velocity + abs(control_error_vel)

	prev_error = error

	if angle > 30*np.pi/180:
		angle = 30*np.pi/180
	if angle < -30*np.pi/180:
		angle = -30*np.pi/180

	msg = drive_param()

	if angle >= 10*np.pi/180 or angle <= -10*np.pi/180:
		velocity = 0.4

	if angle > 20*np.pi/180 or angle < -20*np.pi/180:
		velocity = 0.4

	if angle >= -1*np.pi/180 and angle <= 1*np.pi/180:
		velocity = 0.4

	if velocity < 0:
		velocity = 1

	if velocity > 0.4:
		velocity = 0.4

	msg.velocity = velocity
	msg.angle = angle
	pub.publish(msg)

def listener():
	rospy.init_node('pid_controller', anonymous=True)
	rospy.Subscriber("error", pid_input, control)
	global kp
	global ki
	global kd
	global vel_input
	kp = input("Enter Kp Value: ")
	ki = input("Enter Ki Value: ")
	kd = input("Enter Kd Value: ")
	vel_input = input("Enter Velocity: ")
	rospy.spin()


if __name__ == '__main__':
	print("Listening to error for PID")
	listener()
