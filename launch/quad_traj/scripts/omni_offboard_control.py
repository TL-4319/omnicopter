#!/usr/bin/env python

import rospy
import time
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
		self.sp_pos = PositionTarget()
		self.sp_pos.type_mask = self.sp_pos.IGNORE_YAW | self.sp_pos.IGNORE_YAW_RATE
		self.sp_pos.coordinate_frame = self.sp_pos.FRAME_LOCAL_NED
		self.sp_att = AttitudeTarget()
		self.takeoff_rate = 0.1/50

		# A Message for the current local position of the drone
		self.local_pos = Point(0.0, 0.0, 0.0)
		self.initial_pos = Point(0, 0, 0)
		### Trajectory Publishers ###
		# Position and acceleration setpoint publisher
		self.pose_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
		self.att_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)


		# Local pose subscriber, to check when vehicle reaches a setpoint
		self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.posCb)


		# Set publish/subscribe rate
		self.rate = rospy.Rate(50)  # 50hz
		time.sleep(2)

	def launch(self):
		rospy.loginfo("Launching Drone")
		self.arm()
		# Position (through velocity structure) and acceleration setpoints
		self.sp_pos.position.x = self.initial_pos.x
		self.sp_pos.position.y = self.initial_pos.y
		self.sp_pos.position.z = self.initial_pos.z
		self.sp_pos.velocity.x = 0
		self.sp_pos.velocity.y = 0
		self.sp_pos.velocity.z = 0
		self.sp_pos.acceleration_or_force.x = 0
		self.sp_pos.acceleration_or_force.y = 0
		self.sp_pos.acceleration_or_force.z = 0
		# Attitude and angular rate setpoints
		self.sp_att.orientation.x = 0
		self.sp_att.orientation.y = 0
		self.sp_att.orientation.z = 0
		self.sp_att.orientation.w = 1
		self.sp_att.body_rate.x = 0
		self.sp_att.body_rate.y = 0
		self.sp_att.body_rate.z = 0

		while self.sp_pos.position.z < self.pos_z_launch:
			self.sp_pos.position.z += self.takeoff_rate
			self.sp_pos.velocity.z = self.takeoff_rate*50
			self.pose_pub.publish(self.sp_pos)
			self.rate.sleep()

		self.sp_pos.position.z = self.pos_z_launch
		self.sp_pos.velocity.z = 0

		self.pose_pub.publish(self.sp_pos)
		self.att_pub.publish(self.sp_att)

		at_sp = False
		while not at_sp:
			errx = self.local_pos.x - self.sp_pos.position.x
			erry = self.local_pos.y - self.sp_pos.position.y
			errz = self.local_pos.z - self.sp_pos.position.z
			rospy.loginfo("X error: %s  Y error: %s  Z error: %s", errx, erry, errz)
			if ((abs(errx) < 0.1) and (abs(erry) < 0.1) and (abs(errz) < 0.05)):
				at_sp = True
			self.rate.sleep()
		rospy.loginfo("Drone has reached launch point.")

	def land(self):
		rospy.loginfo("Return to Origin")
		self.sp_pos.position.x = self.initial_pos.x
		self.sp_pos.position.y = self.initial_pos.y
		self.sp_pos.velocity.x = 0
		self.sp_pos.velocity.y = 0
		self.sp_pos.acceleration_or_force.x = 0
		self.sp_pos.acceleration_or_force.y = 0
		self.sp_pos.acceleration_or_force.z = 0
		self.pose_pub.publish(self.sp_pos)
		at_sp = False
		while not at_sp:
			errx = self.local_pos.x - self.sp_pos.position.x
			erry = self.local_pos.y - self.sp_pos.position.y
			errz = self.local_pos.z - self.sp_pos.position.z
			if ((abs(errx) < 0.1) and (abs(erry) < 0.1) and (abs(errz) < 0.1)):
				at_sp = True
			self.rate.sleep()
		rospy.loginfo("Landing Drone")

		# slowly lower z setpoint
		z_sp = self.local_pos.z - self.takeoff_rate #0.001
		self.sp_pos.position.z = z_sp
		self.sp_pos.velocity.z = -self.takeoff_rate*50
		self.pose_pub.publish(self.sp_pos)
		while z_sp > (self.initial_pos.z + 0.1):
			self.rate.sleep()
			z_sp -= self.takeoff_rate
			self.sp_pos.position.z = z_sp
			self.sp_pos.velocity.z = -self.takeoff_rate*50
			self.pose_pub.publish(self.sp_pos)

		# Wait till vehicle is within 8 centimeters of ground to kill motors
		at_sp = False
		while not at_sp:
			errz = self.local_pos.z - self.initial_pos.z
			if (errz < 0.15):
				at_sp = True
			self.rate.sleep()
		self.disarm()
		rospy.loginfo("Drone has landed and been disarmed")

	def set_init_offset(self, traj_file):
		self.initial_pos.x = self.local_pos.x
		self.initial_pos.y = self.local_pos.y
		self.initial_pos.z = self.local_pos.z

		traj_lines = traj_file.readlines()
		data = traj_lines[0]
		datasplit = data.split(",")
		datarray = np.array(map(float, datasplit))
		self.pos_x_offset = self.initial_pos.x - datarray[0]
		self.pos_y_offset = self.initial_pos.y - datarray[1]
		self.pos_z_launch = datarray[2]
		print('X offset: %s'%self.pos_x_offset)
		print('Y offset: %s'%self.pos_y_offset)

	def fly_trajectory(self, traj_file):
		traj_lines = traj_file.readlines()
		traj_len = len(traj_lines)
		for i in range(traj_len):
			data = traj_lines[i]
			datasplit = data.split(",")
			datarray = np.array(map(float, datasplit))

			######################## Velocity and Acceleration Publishing
			# self.sp_pos.position.x = datarray[0] + self.pos_x_offset
			# self.sp_pos.position.y = datarray[1] + self.pos_y_offset
			# self.sp_pos.position.z = datarray[2]

			# self.sp_pos.velocity.x = datarray[3] #Keep this for now...may need to update trajectory files
			# self.sp_pos.velocity.y = datarray[4]
			# self.sp_pos.velocity.z = datarray[5]

			# self.sp_pos.acceleration_or_force.x = datarray[6] #Keep this for now...may need to update trajectory files
			# self.sp_pos.acceleration_or_force.y = datarray[7]
			# self.sp_pos.acceleration_or_force.z = datarray[8]

			# self.sp_att.orientation.w = datarray[9]
			# self.sp_att.orientation.x = datarray[10]
			# self.sp_att.orientation.y = datarray[11]
			# self.sp_att.orientation.z = datarray[12]

			# self.sp_att.body_rate.x = datarray[13]
			# self.sp_att.body_rate.y = datarray[14]
			# self.sp_att.body_rate.z = datarray[15]

			##### JUST Vel or Accel Publishing
			self.sp_pos.position.x = datarray[0] + self.pos_x_offset
			self.sp_pos.position.y = datarray[1] + self.pos_y_offset
			self.sp_pos.position.z = datarray[2]

			self.sp_pos.velocity.x = datarray[3] #Keep this for now...may need to update trajectory files
			self.sp_pos.velocity.y = datarray[4]
			self.sp_pos.velocity.z = datarray[5]

			self.sp_pos.acceleration_or_force.x = datarray[3] #Keep this for now...may need to update trajectory files
			self.sp_pos.acceleration_or_force.y = datarray[4]
			self.sp_pos.acceleration_or_force.z = datarray[5]

			self.sp_att.orientation.w = datarray[6]
			self.sp_att.orientation.x = datarray[7]
			self.sp_att.orientation.y = datarray[8]
			self.sp_att.orientation.z = datarray[9]

			self.sp_att.body_rate.x = datarray[10]
			self.sp_att.body_rate.y = datarray[11]
			self.sp_att.body_rate.z = datarray[12]

			#######################################
			self.pose_pub.publish(self.sp_pos)
			self.att_pub.publish(self.sp_att)

			errx = self.local_pos.x - self.sp_pos.position.x
			erry = self.local_pos.y - self.sp_pos.position.y
			errz = self.local_pos.z - self.sp_pos.position.z

			rospy.loginfo("X error: %s  Y error: %s  Z error: %s", errx, erry, errz)
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
		# Position (through velocity structure) and acceleration setpoints
		self.sp_pos.position.x = 0
		self.sp_pos.position.y = 0
		self.sp_pos.position.z = 0
		self.sp_pos.velocity.x = 0
		self.sp_pos.velocity.y = 0
		self.sp_pos.velocity.z = 0
		self.sp_pos.acceleration_or_force.x = 0
		self.sp_pos.acceleration_or_force.y = 0
		self.sp_pos.acceleration_or_force.z = 0
		# Attitude and angular rate setpoints
		self.sp_att.orientation.x = 0
		self.sp_att.orientation.y = 0
		self.sp_att.orientation.z = 0
		self.sp_att.orientation.w = 1
		self.sp_att.body_rate.x = 0
		self.sp_att.body_rate.y = 0
		self.sp_att.body_rate.z = 0

		for k in range(20):
        		self.pose_pub.publish(self.sp_pos)
			self.att_pub.publish(self.sp_att)
			self.rate.sleep()
		change_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
		rospy.wait_for_service('/mavros/set_mode')
		try:
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

	# Load trajectory file
	trajectory = raw_input("Desired trajectory 1: ")
	traj_file_name = '/home/gs2/omni/quadcopter-lqr-controller/launch/quad_traj/scripts/' + trajectory

	# Initiate Flight Controller for drone
	drone1 = flight_controller()
	with open(traj_file_name, 'r') as f: drone1.set_init_offset(f)

	drone1.setOffboardMode()


	# Launch
	drone1.launch()

	time.sleep(20)

	# Trajectory
	# rospy.loginfo("Flying: %s", trajectory)
	# with open(traj_file_name, 'r') as f: drone1.fly_trajectory(f)

	# Land
	drone1.land()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
