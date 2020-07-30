#!/usr/bin/env python

import sys
import rospy
import copy
import time

import moveit_commander
import moveit_msgs.msg

from geometry_msgs.msg import PoseStamped, Pose

from geometry_msgs.msg import Quaternion
import tf
import numpy as np

class FromTerminal():
	def __init__(self):
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('input', anonymous=True)

		self.move_group = moveit_commander.MoveGroupCommander("panda_arm")

	def home_position(self):
		grasp_pose = self.move_group.get_current_pose()

		# Home coordinates
		grasp_pose.pose.position.x = 0.307019570052
		grasp_pose.pose.position.y = 0
		grasp_pose.pose.position.z = 0.590269558277

		grasp_pose.pose.orientation.x = 0.923955699469
		grasp_pose.pose.orientation.y = -0.382499497279
		grasp_pose.pose.orientation.z = 0
		grasp_pose.pose.orientation.w = 0

		self.move_group.set_pose_target(grasp_pose)		# Assign the goal pose
		self.move_group.go()	# Plan and execute the manipulator

	def find_waypoints(self, goal_x, goal_y, goal_z):
		grasp_pose = PoseStamped()		# Set grasp_pose's data structure

		grasp_pose = self.move_group.get_current_pose()		# Make grasp_pose be the current pose

		grasp_pose.header.stamp = rospy.Time.now()
		grasp_pose.header.frame_id = "world"

		grasp_pose.pose.position.x = goal_x	
		grasp_pose.pose.position.y = goal_y
		grasp_pose.pose.position.z = goal_z

		# quat = tf.transformations.quaternion_from_euler(0, -np.pi/2, 0) #rotation about z axis
		# grasp_pose.pose.orientation = Quaternion(*quat)

		self.move_group.set_pose_target(grasp_pose)		# Assign the goal pose
		self.move_group.go()	# Plan and execute the manipulator

	def teleop(self):
		grasp_pose = self.move_group.get_current_pose()

		status = 'asdsa'
		while True:
			print(" Press [w] to move forward along +X direction\n Press [s] to move along -X direction\n \
Press [a] to move along -Y direction\n Press [d] to move along +Y direction\n \
Press [r] to move upward\n Press [f] to move downward\n Press [x] to exit teleop mode")
			status = raw_input()
			if (status == 'w'):
				# print("As you commanded Captain")
				grasp_pose.pose.position.x += 0.1	
			elif (status == 's'):
				# print("As you commanded Captain")
				grasp_pose.pose.position.x -= 0.1
			elif (status == 'a'):
				# print("As you commanded Captain")
				grasp_pose.pose.position.y -= 0.1
			elif (status == 'd'):
				# print("As you commanded Captain")
				grasp_pose.pose.position.y += 0.1
			elif (status == 'r'):
				# print("As you commanded Captain")
				grasp_pose.pose.position.z += 0.1
			elif (status == 'f'):
				# print("As you commanded Captain")
				grasp_pose.pose.position.z -= 0.1
			elif (status == 'x'): 
				return
			else:
				print("Press a valid key\n")
			self.move_group.set_pose_target(grasp_pose)		# Assign the goal pose
			self.move_group.go()
			# print("----------Next command captain!!-------------------")

	# def curr_position():
	# 	curr_pos = self.move_group.get_current_pose()
	# 	curr_x = curr_pos.pose.position.x
	# 	curr_y = curr_pos.pose.position.y
	# 	curr_z = curr_pos.pose.position.z

	# 	return curr_x, curr_y, curr_z

def main():
	group = moveit_commander.MoveGroupCommander("panda_arm")

	instance = FromTerminal()
	answer = 'abcgd'

	while True:
		curr_position = group.get_current_pose()

		answer = raw_input(" Press [h] to go to home position\n Press [n] to enter new position\n Press [x] to exit\n Press [t] to teleoperate robot\n")
		
		if answer == 'h':
			instance.home_position()
		
		elif answer == 'n':
			print("\nPlease enter coordinates of goal position \n")
			try:
				goal_x = input("x: ")
			except SyntaxError:
				goal_x = curr_position.pose.position.x
			try:
				goal_y = input("y: ")
			except SyntaxError:
				goal_y = curr_position.pose.position.y
			try:
				goal_z = input("z: ")
			except SyntaxError:
				goal_z = curr_position.pose.position.z
			instance.find_waypoints(goal_x, goal_y, goal_z)

		elif answer == 't':
			instance.teleop()

		elif answer == 'x':
			return
			

if __name__ == '__main__':
	print("Your wish is my command Venky Stormblessed")
	main()

