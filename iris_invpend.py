#!/usr/bin/env python


""" Configure a cart-pole system spawned in Gazebo to be a qualified environment for reinforcement learning """
from __future__ import print_function

import numpy as np
import math
import random
import time

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, Vector3, PoseStamped, TwistStamped
from mavros_msgs.msg import AttitudeTarget, State
from mavros_msgs.srv import SetMode, CommandBool
#from pymavlink import mavutil
#from six.moves import xrange
from std_msgs.msg import Header
#from threading import Thread
from tf.transformations import euler_from_quaternion



# Simulation geometry
MASS_POLE = 2
LENGTH_POLE = 0.5
RADIUS = 0.01
RADIUS_POLE = 0.025
G = 9.8

PKG = 'px4'


class MavrosOffboardAttctl():
	"""
	Tests flying in offboard control by sending attitude and thrust setpoints
	via MAVROS.
	For the test to be successful it needs to cross a certain boundary in time.
	"""
	def __init__(self):
		rospy.init_node('AttControl', anonymous=True)
		self.att = AttitudeTarget()
		self.att_setpoint_pub = rospy.Publisher(
			'mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
		self._sub_invpend_state = rospy.Subscriber('/iris/joint_states', JointState, self.jstates_callback)
		self.drone_pose_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.drone_pose_cb)
		self.drone_vel_subscriber = rospy.Subscriber('/mavros/local_position/velocity_body', TwistStamped, self.drone_vel_cb)
		self. curr_drone_pose = PoseStamped()
		self. curr_drone_vel = TwistStamped()
		self. vel_pole = 0
		self. pos_pole = 0
		# desired yawrate target
		self.des_yawrate = 0.1
		self.yawrate_tol = 0.02
		self.att.body_rate = Vector3()
		self.att.header = Header()
		#self.att.header.frame_id = "base_footprint"
		#self.att.orientation = self.curr_drone_pose.pose.orientation

	def jstates_callback(self, data):
		""" Callback function for subscribing /invpend/joint_states topic """
		self.pos_pole = data.position[0]
		self.vel_pole = data.velocity[0]

	def drone_pose_cb(self, msg):
		self.curr_drone_pose = msg


	def drone_vel_cb(self, msg):
		self.curr_drone_vel = msg
		# print("Drone Velocity", self.curr_drone_vel)
		#has .linear.x .y .z angular

	def set_offboard_mode(self):
		rospy.wait_for_service('/mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
			isModeChanged = flightModeService(custom_mode='OFFBOARD')
		except rospy.ServiceException as e:
			print("service set_mode call failed: %s. OFFBOARD Mode could not be set. Check that GPS is enabled" % e)
		
	def set_arm(self):
		rospy.wait_for_service('/mavros/cmd/arming')
		try:
			armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
			armService(True)
			self.arm = True
		except rospy.ServiceException as e:
			print("Service arm call failed: %s" % e)

	def take_off(self):
		self.set_offboard_mode()
		self.set_arm()

	def statefeedback(self):
		self.x = self.curr_drone_pose.pose.position.x
		self.y = self.curr_drone_pose.pose.position.y
		self.z = self.curr_drone_pose.pose.position.z
		o = self.curr_drone_pose.pose.orientation
		(self.roll, self.pitch, self.yaw)=euler_from_quaternion([o.x, o.y, o.z, o.w])
		self.xdot = self.curr_drone_vel.twist.linear.x
		self.ydot = self.curr_drone_vel.twist.linear.y
		self.zdot = self.curr_drone_vel.twist.linear.z
		self.r = LENGTH_POLE * math.cos(self.pos_pole)
		self.s = LENGTH_POLE * math.sin(self.pos_pole)
		self.rdot = LENGTH_POLE * math.cos(self.vel_pole)
		self.sdot = LENGTH_POLE * math.sin(self.vel_pole)


	def LQR(self):
		self.Ky = np.array([3.162277660168370,4.110266480060687,13.966488802439859,50.213969275831396,14.052305698647750])
		self.Kx = np.array([-3.162277660168276,-4.110266480060570,13.966488802439772,-50.213969275830540,-14.052305698647515])
		self.Kz = np.array([10.0, 4.4721])

		att_rate_x = np.dot(-self.Kx, np.array([self.x - 20, self.xdot, self.roll, self.r, self.rdot]))
		att_rate_y = np.dot(-self.Ky, np.array([self.y - 20, self.ydot, self.pitch, self.s, self.sdot]))
		a = np.dot(-self.Kz, np.array([self.z -20, self.zdot])) + G

		print("x =", att_rate_x, "y =", att_rate_y, "a=",a)
		return att_rate_x, att_rate_y, a


	def Fly(self):
		rate = rospy.Rate(10)  # Hz
		self.take_off()
		while not rospy.is_shutdown():
			self.statefeedback()
			x,y,a = self.LQR()
			self.att.body_rate.x = 10*x
			self.att.body_rate.y = 10*y
			self.att.body_rate.z = 50

			self.att.thrust = 10*a
			self.att_setpoint_pub.publish(self.att)
			try:  # prevent garbage in console output when thread is killed
				rate.sleep()
			except rospy.ROSInterruptException:
				pass


if __name__ == '__main__':
	iris = MavrosOffboardAttctl()
	iris.Fly()