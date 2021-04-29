#!/usr/bin/env python

import rospy
from mavros_msgs.msg import State, AttitudeTarget
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TwistStamped
import math
import numpy as np
from sensor_msgs.msg import JointState
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import String, UInt16, Float64
from tf.transformations import quaternion_from_euler, euler_from_quaternion


MASS_POLE = 2
LENGTH_POLE = 0.5
RADIUS = 0.01
RADIUS_POLE = 0.025
G = 9.8


class OffbPosCtl:
	def __init__(self):
		rospy.init_node('offboard_test', anonymous=True)
		##################################subscribers ########################################
		self.drone_pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.drone_pose_cb)
		self.drone_state_sub = rospy.Subscriber('/mavros/state', State, self.drone_state_cb)
		self.invpend_state_sub = rospy.Subscriber('/iris/joint_states', JointState, self.jstates_callback)
		self.drone_vel_sub = rospy.Subscriber('/mavros/local_position/velocity_body', TwistStamped, self.drone_vel_cb)
		#################################publishers######################################
		self.drone_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd', TwistStamped, queue_size =1)
		self.drone_pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
		self.drone_att_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)


		###############commands#############################
		self.att_cmd = AttitudeTarget()
		self.vel_cmd = TwistStamped()
		self.pose_cmd = PoseStamped()
		######################################################
		self.curr_drone_pose = PoseStamped()
		self.curr_drone_vel = TwistStamped()
		self.is_ready_to_fly = False
		self.hover_loc = [3.0,0.0, 3, 0, 0, 0, 0] # Hovers 3meter above at this location
		self.dist_threshold = 0.1
		self.arm = False

		self.pole_pose = 0
		self.pole_vel = 0

		self.locations = np.matrix([[0.0, 0.0, 3.0, 0, 0, 0, 0],
							  [0.0, 01.0, 03.0, 0, 0, 0, 0],
							  [0.0, 02.0, 03.0, 0, 0, 0, 0],])


	def drone_pose_cb(self, msg):
		self.curr_drone_pose = msg


	def drone_vel_cb(self,msg):
		self.curr_drone_vel = msg


	def drone_state_cb(self, msg):
		print (msg.mode)
		# if (msg.mode == 'OFFBOARD'):
		# 	self.isReadyToFly = True
	 #    	print("readyToFly")

		# if (msg.mode == 'HOVER'):
		# 	self.hover()

	def jstates_callback(self, msg):
		self.pole_pose = msg.position[0]
		self.pole_vel = msg.velocity[0]


	def set_offboard_mode(self):
		rospy.wait_for_service('/mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
			isModeChanged = flightModeService(custom_mode='OFFBOARD')
		except rospy.ServiceException as e:
			print("service set_mode call failed: %s. OFFBOARD Mode could not be set. Check that GPS is enabled" % e)

	def set_autoland_mode(self):
		rospy.wait_for_service('/mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
			isModeChanged = flightModeService(custom_mode='AUTO.LAND')
		except rospy.ServiceException as e:
			print("service set_mode call failed: %s. OFFBOARD Mode could not be set. Check that GPS is enabled" % e)


	def set_hover_mode(self):
		rospy.wait_for_service('/mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
			isModeChanged = flightModeService(custom_mode='HOVER')
		except rospy.ServiceException as e:
			print("service set_mode call failed: %s. HOVER Mode could not be set. Check that GPS is enabled" % e)   


	def set_arm(self):
		rospy.wait_for_service('/mavros/cmd/arming')
		try:
			armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
			armService(True)
			self.arm = True
		except rospy.ServiceException as e:
			print("Service arm call failed: %s" % e)	


	#function to get all the states
	def statefeedback(self):
		self.x = self.curr_drone_pose.pose.position.x
		self.y = self.curr_drone_pose.pose.position.y
		self.z = self.curr_drone_pose.pose.position.z
		o = self.curr_drone_pose.pose.orientation
		(self.roll, self.pitch, self.yaw)=euler_from_quaternion([o.x, o.y, o.z, o.w])
		self.xdot = self.curr_drone_vel.twist.linear.x
		self.ydot = self.curr_drone_vel.twist.linear.y
		self.zdot = self.curr_drone_vel.twist.linear.z
		self.r = LENGTH_POLE * math.cos(self.pole_pose)
		self.s = 0 #LENGTH_POLE * self.pole_pose
		self.rdot = LENGTH_POLE * - math.sin(self.pole_pose)* self.pole_vel
		self.sdot = 0 #LENGTH_POLE * math.sin(self.pole_vel)


	def LQR(self):
		self.Ky = np.array([3.162277660168361,3.604814995967241,15.580214944933351, 71.472487821312580,15.977042199347748]) #np.array([3.162277660168370,4.110266480060687,13.966488802439859,50.213969275831396,14.052305698647750])
		self.Kx = np.array([-3.162277660168527,-3.604814995967372,15.580214944933577,-71.472487821314080,-15.977042199348098]) #np.array([-3.162277660168276,-4.110266480060570,13.966488802439772,-50.213969275830540,-14.052305698647515])
		self.Kz = np.array([10.0, 4.4721])


		#print("Position", self.x, " ",self.y, " ",self.z)
		att_rate_x = np.dot(-self.Kx, np.array([self.x_diff, self.xdot, self.roll, self.r, self.rdot]))
		att_rate_y = np.dot(-self.Ky, np.array([self.y_diff, self.ydot, self.pitch, self.s, self.sdot]))
		a = np.dot(-self.Kz, np.array([self.z_diff, self.zdot])) - G

		print("x =", att_rate_x, "y =", att_rate_y, "a=",a)
		return att_rate_x, att_rate_y, a



	def goToPosition(self):
		""" hover at height mentioned in location
			set mode as HOVER to make it work
		"""
		index=0
		rate = rospy.Rate(20)
		des_pose = PoseStamped()
		#print(self.mode)
		while not rospy.is_shutdown():
			self.set_arm()
			self.set_offboard_mode()
			des_pose.pose.position.x = self.locations[index, 0]
			des_pose.pose.position.y = self.locations[index, 1]
			des_pose.pose.position.z = self.locations[index, 2]
			des_pose.pose.orientation.x = self.locations[index, 3]
			des_pose.pose.orientation.y = self.locations[index, 4]
			des_pose.pose.orientation.z = self.locations[index, 5]
			des_pose.pose.orientation.w = self.locations[index, 6]

			curr_x = self.curr_drone_pose.pose.position.x
			curr_y = self.curr_drone_pose.pose.position.y
			curr_z = self.curr_drone_pose.pose.position.z

			des_x = des_pose.pose.position.x
			des_y = des_pose.pose.position.y
			des_z = des_pose.pose.position.z

			dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) + (curr_z - des_z)*(curr_z - des_z))

			self.drone_pose_pub.publish(des_pose)
			if dist < 0.5:
				#index+=1
				if index > 2:
					index = 2
				self.attitudeControl(curr_x, curr_y, curr_z, des_x, des_y, des_z)

			rate.sleep()

	def attitudeControl(self, curr_x, curr_y, curr_z, des_x, des_y, des_z):
		self.statefeedback()
		self.x_diff = curr_x - des_x
		self.y_diff = curr_y - des_y
		self.z_diff = curr_z - des_z
		x,y,a = self.LQR()
		self.att_cmd.type_mask = 128
		self.att_cmd.header.frame_id = "base_footprint"  
		self.att_cmd.body_rate.x = x *0.01
		self.att_cmd.body_rate.y = y *0.05
		# if self.pole_pose < 0:
		# 	self.att_cmd.body_rate.y = -0.1
		# else:
		# 	self.att_cmd.body_rate.y = 0.1
		self.att_cmd.body_rate.z = 0

		self.drone_att_pub.publish(self.att_cmd)


	def controller(self):
		rate = rospy.Rate(20)
		self.goToPosition()

		while not rospy.is_shutdown():
			rate.sleep()




if __name__ == "__main__":
	iris = OffbPosCtl()
	iris.controller()
