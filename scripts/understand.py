#!/usr/bin/env python

import sys
import rospy
import copy
import time

from geometry_msgs.msg import PoseStamped, Pose

import tf
from geometry_msgs.msg import Quaternion
import numpy as np
import moveit_commander
import moveit_msgs.msg


# An __init__ is compulsary, hence a class is compulsary
class Grasper():
	"""docstring for  Grasper"""
	def __init__(self):
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('understand', anonymous=True)

		self.move_group = moveit_commander.MoveGroupCommander("panda_arm")

	def plan(self):
		curr_pose = PoseStamped()
		curr_pose.header.stamp = rospy.Time.now()
		curr_pose.header.frame_id = "base_link"
		print("")
		print("PoseStamped: ", curr_pose)
		check_pose = Pose()
		print("")
		print ("Pose: ", check_pose)
		# To display in rviz
		display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)
		waypoints = []

		wpose = self.move_group.get_current_pose().pose
		print("xxxxxxxxxxxxxxxxx----------WPOSE------------------------xxxxxxxxxxxxxxxxxxxxx")
		print(wpose)
		wpose.position.z += 0.1
		waypoints.append(copy.deepcopy(wpose))

		(plan, fraction) = self.move_group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01, 0.0)        # eef_step, jump_step
                                           
		# display_trajectory_publisher.publish(plan)
		self.move_group.execute(plan, wait = True)

	def my_plan(self):
		# self.move_group.set_goal_tolerance(0.1)
		grasp_pose = PoseStamped()

		print("Planning in ",self.move_group.get_planning_frame(), "frame")
		grasp_pose = self.move_group.get_current_pose()
		
		grasp_pose.header.stamp = rospy.Time.now()
		grasp_pose.header.frame_id = "world"
		grasp_pose.pose.position.x = 0.4
		grasp_pose.pose.position.y = 0.2
		grasp_pose.pose.position.z = 0.8
		grasp_pose.pose.orientation.x = -0.92407151731
		grasp_pose.pose.orientation.y = 0.382219446895
		grasp_pose.pose.orientation.z = 0
		
		# quat = tf.transformations.quaternion_from_euler(0, -np.pi/2, 0) #rotation about z axis
		# grasp_pose.pose.orientation = Quaternion(*quat)
		
		self.move_group.set_pose_target(grasp_pose)
		self.move_group.go()

		print(grasp_pose)
		# plan = self.move_group.plan()
		# self.move_group.execute(plan, wait=True)

def main():
	instance = Grasper()
	print("-----Planning and executing----")
	instance.my_plan()

if __name__ == '__main__':
	i = 0
	while i < 1:
		print("__Hello Venky__::Your Info at: ", i)
		main()
		# check_poseStamped = PoseStamped()
		# print("PoseStamped: ", check_poseStamped)
		# check_pose = Pose()
		# print ("Pose: ", check_pose)
		i += 1

