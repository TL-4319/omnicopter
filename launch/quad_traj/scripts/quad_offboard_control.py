#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Vector3Stamped

from gazebo_msgs.msg import ModelStates
from mavros_msgs.msg import *
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode

attonly = False # Set to True for to send sp based on time rather than position
attcount = 0
omnipose = None
linecount = 0
acc_rad = 0.1
sp = PoseStamped()
err = PoseStamped()

class flight_controller():
	def __init__(self):
		self.sp = PoseStamped()
		# A Message for the current local position of the drone
		self.local_pos = Point(0.0, 0.0, 0.0)
		self.initial_z = 0
		### Trajectory Publishers ###
		# Position and acceleration setpoint publisher
		self.pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)

		# Local pose subscriber, to check when vehicle reaches a setpoint
		self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.posCb)


		# Set publish/subscribe rate
		self.rate = rospy.Rate(10)  # 50hz

	def launch(self):
		rospy.loginfo("Launching Drone")
		self.initial_z = self.local_pos.z
		self.arm()
		self.sp.pose.position.x = 0
		self.sp.pose.position.y = 0
		self.sp.pose.position.z = 0.5
		self.sp.pose.orientation.x = 0
		self.sp.pose.orientation.y = 0
		self.sp.pose.orientation.z = 0 #-0.7071
		self.sp.pose.orientation.w = -1 #0.7071
		self.pose_pub.publish(self.sp)
		at_sp = False
		while not at_sp:
			errx = self.local_pos.x - self.sp.pose.position.x
			erry = self.local_pos.y - self.sp.pose.position.y
			errz = self.local_pos.z - self.sp.pose.position.z
			rospy.loginfo("X error: %s  Y error: %s  Z error: %s", errx, erry, errz)
			if ((abs(errx) < 0.1) and (abs(erry) < 0.1) and (abs(errz) < 0.05)):
				at_sp = True
			self.rate.sleep()
		rospy.loginfo("Drone has reached launch point.")

	def land(self):
		rospy.loginfo("Landing Drone")
		self.sp.pose.position.x = 0
		self.sp.pose.position.y = 0
		self.pose_pub.publish(self.sp)
		at_sp = False
		while not at_sp:
			errx = self.local_pos.x - self.sp.pose.position.x
			erry = self.local_pos.y - self.sp.pose.position.y
			errz = self.local_pos.z - self.sp.pose.position.z
			if ((abs(errx) < 0.01) and (abs(erry) < 0.01) and (abs(errz) < 0.01)):
				at_sp = True
			self.rate.sleep()
		self.sp.pose.position.z = self.initital_z
		self.pose_pub.publish(self.sp)
		at_sp = False
		while not at_sp:
			errz = self.local_pos.z - self.sp.pose.position.z
			if (abs(errz) < 0.06):
				at_sp = True
			self.rate.sleep()
		self.disarm()
		rospy.loginf("Drone has landed and been disarmed")

	def fly(self, traj_points, time_based, delta):
		traj_len = len(traj_points)
		for i in range(traj_len):
			data = traj_points[i]
			datasplit = data.split(",")
			datasplit[2] = datasplit[2].rstrip()
			datarray = np.array(map(float, datasplit))
			self.sp.pose.position.x = datarray[0]
			self.sp.pose.position.y = datarray[1]
			self.sp.pose.position.z = datarray[2]
			self.sp.pose.orientation.x = datarray[3]
			self.sp.pose.orientation.y = datarray[4]
			self.sp.pose.orientation.z = datarray[5]
			self.sp.pose.orientation.w = datarray[6]
			self.pose_pub.publish(self.sp)
			errx = self.local_pos.x - self.sp.pose.position.x
			erry = self.local_pos.y - self.sp.pose.position.y
			errz = self.local_pos.z - self.sp.pose.position.z
			if not time_based:
				mtns = False
				while not mtns:
					if ((abs(errx) < delta) and (abs(erry) < delta) and (abs(errz) < delta)):
						mtns = True
					self.rate.sleep()
			else:
				for k in range(delta):
					self.rate.sleep()

	def arm(self):
		rospy.wait_for_service('mavros/cmd/arming')
		try:
			armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
			armService(True)
		except rospy.ServiceException:
			rospy.loginfo("Service arming call failed")

	def disarm(self):
		rospy.wait_for_service('mavros/cmd/arming')
		try:
			armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
			armService(False)
		except rospy.ServiceException:
			rospy.loginfo("Service disarming call failed")

	def setOffboardMode(self):
		self.sp.pose.position.x = 0
		self.sp.pose.position.y = 0
		self.sp.pose.position.z = 0.0
		self.sp.pose.orientation.x = 0
		self.sp.pose.orientation.y = 0
		self.sp.pose.orientation.z = 0 # -0.7071
		self.sp.pose.orientation.w = -1 # -0.7071
		for k in range(20):
        		self.pose_pub.publish(self.sp)
        		self.rate.sleep()
		change_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
		rospy.wait_for_service('/mavros/set_mode')
		try:
			# base_mode = 0
			# custom_mode = 'OFFBOARD'
			out = change_mode(custom_mode='OFFBOARD')
			if out:
				rospy.loginfo("Successfully changed to offboard mode")
			else:
				rospy.loginfo("Failed to change to offboard mode")
				rospy.spin()
		except rospy.ServiceException:
			rospy.loginfo("Service call failed")

	def posCb(self, msg):
		self.local_pos.x = msg.pose.position.x
		self.local_pos.y = msg.pose.position.y
		self.local_pos.z = msg.pose.position.z


def main():
	rospy.init_node('quad_trajectory', anonymous=True)

	# Initiate Flight Controller for drone
	drone1 = flight_controller()

	# Load trajectory/setpoint file
	#traj_file = open(r'/home/omni/PX4-Autopilot/launch/quad_traj/scripts/traj_move_up.txt')
	#lines = traj_file.readlines()

	drone1.setOffboardMode()
	# Launch
	drone1.launch()

	# Trajectory
	#drone1.fly(lines, True, 0.1)

	# Land
	drone1.land()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
