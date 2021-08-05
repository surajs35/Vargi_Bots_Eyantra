#! /usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg

import yaml
import os
import math
import time
import sys
import cv2
import copy
import numpy as np

from pyzbar.pyzbar import decode
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from collections import OrderedDict

from std_srvs.srv import Empty
from std_msgs.msg import String

from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.msg import LogicalCameraImage

from pkg_task4.srv import get_info,get_infoResponse

class Ur5_1_Moveit:

	# Constructor
	def __init__(self):

		self._robot_ns = '/' +  "ur5_1"
		self._planning_group = "manipulator"

		self._commander = moveit_commander.roscpp_initialize(sys.argv)
		self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
		self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
		self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
		self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
		self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
		self._exectute_trajectory_client.wait_for_server()

		self._planning_frame = self._group.get_planning_frame()
		self._eef_link = self._group.get_end_effector_link()
		self._group_names = self._robot.get_group_names()
		self._box_name = ''
		self._packages=OrderedDict()
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,self.callback)
		self.service = rospy.Service('get_box_info', get_info, self.package_publish)
		
		# Attribute to store computed trajectory by the planner
		self._computed_plan = ''
		self._group.allow_replanning(True)
		
		# Current State of the Robot is needed to add box to planning scene
		self._curr_state = self._robot.get_current_state()
		
		rospy.loginfo(
			'\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
		rospy.loginfo(
			'\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
		rospy.loginfo(
			'\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')


		rp = rospkg.RosPack()
		self._pkg_path = rp.get_path('pkg_task4')
		self._file_path = self._pkg_path + '/config/saved_trajectories/'
		rospy.loginfo( "Package Path: {}".format(self._file_path) )
		rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

	# To get the feed of camera_1 of packages colour
	def callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			rospy.logerr(e)

		(rows,cols,channels) = cv_image.shape

		image = cv_image
		img = image
		contrast_img = cv2.addWeighted(img, 2.5, np.zeros(img.shape, img.dtype), 0, 0)
		denoised_img = cv2.fastNlMeansDenoisingColored(contrast_img ,None,10,10,7,21)


		result = self.get_qr_data(denoised_img)
		rospy.loginfo(result)
		if(result=="DONE"):
			self.image_sub.unregister()
		else:
			pass
		cv2.waitKey(3)

	# To get the decoded message of qrcode
	def get_qr_data(self, arg_image):

		qr_result = decode(arg_image)

		for qrcode in qr_result:
			(x,y,w,h) = qrcode.rect
			pkg_name = self.pkg_name(x,y)
			self._packages.update({pkg_name:qrcode.data})
		rospy.loginfo(self._packages)
		if (len(self._packages)==12):
			return ('DONE')
		else :
			self._packages.clear()
			return ('FAIL')

	# To detect the package name based on position of the package
	def pkg_name(self, x, y):
		switcher = {
			128315:"packagen00",315315:"packagen01",503316:"packagen02",128496:"packagen10",316496:"packagen11",502495:"packagen12",
			129643:"packagen20",315643:"packagen21",502643:"packagen22",130796:"packagen30",318797:"packagen31",501796:"packagen32"
		}
		return switcher.get(int(str(x)+str(y)),"Invalid_x_and_y")

	# To send box_colour to another node
	def package_publish(self, obj):
		return get_infoResponse(self._packages[obj.boxname])

	# To set the joint angles of ur5
	def set_joint_angles(self, arg_list_joint_angles):

		list_joint_values = self._group.get_current_joint_values()
		rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
		rospy.loginfo(list_joint_values)

		self._group.set_joint_value_target(arg_list_joint_angles)
		self._computed_plan = self._group.plan()
		flag_plan = self._group.go(wait=True)

		list_joint_values = self._group.get_current_joint_values()
		rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
		rospy.loginfo(list_joint_values)

		pose_values = self._group.get_current_pose().pose
		rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
		rospy.loginfo(pose_values)

		if (flag_plan == True):
			rospy.loginfo(
				'\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
		else:
			rospy.logerr(
				'\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

		return flag_plan

	# To set joint angles in more attempts if failed
	def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):

		number_attempts = 0
		flag_success = False
		
		while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
			number_attempts += 1
			flag_success = self.set_joint_angles(arg_list_joint_angles)
			rospy.logwarn("attempts: {}".format(number_attempts) )
			# self.clear_octomap()

	# To move the ur5 in cartesian path
	def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
		# 1. Create a empty list to hold waypoints
		waypoints = []

		# 2. Add Current Pose to the list of waypoints
		wpose = self._group.get_current_pose().pose
		waypoints.append(copy.deepcopy(wpose))

		# 3. Create a New waypoint
		wpose.position.z += trans_z  # First move up (z)
		wpose.position.y += trans_y  # and sideways (y)
		wpose.position.x += trans_x 
		# 4. Add the new waypoint to the list of waypoints
		waypoints.append(copy.deepcopy(wpose))


		# 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
		(plan, fraction) = self._group.compute_cartesian_path(
			waypoints,   # waypoints to follow
			0.01,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
			0.0,
			True)         # Jump Threshold
		rospy.loginfo("Path computed successfully. Moving the arm.")

		self._computed_plan = plan
		num_pts = len(plan.joint_trajectory.points)
		if (num_pts >= 3):
			del plan.joint_trajectory.points[0]
			del plan.joint_trajectory.points[1]
		
		# 6. Make the arm follow the Computed Cartesian Path
		self._group.execute(plan)


	# To add package in Rviz
	def addbox(self, box_name, box_pos):
		scene = self._scene
		box_position = geometry_msgs.msg.PoseStamped()
		box_position.header.frame_id = self._robot.get_planning_frame()
		box_position.pose.position.x=box_pos[0]
		box_position.pose.position.y=box_pos[1]
		box_position.pose.position.z=box_pos[2]

		scene.add_box(box_name, box_position, size=(0.15, 0.15, 0.15))
		rospy.loginfo("Added Box")

	# To attach the package to vacuum gripper
	def attach_box(self,box_name):

		robot = self._robot
		scene = self._scene
		eef_link = self._eef_link
		group_names = self._group_names

		grasping_group = self._planning_group
		touch_links = robot.get_link_names(group=grasping_group)
		scene.attach_box(eef_link, box_name, touch_links=touch_links)
		activate_gripper = rospy.ServiceProxy('eyrc/vb/ur5/activate_vacuum_gripper/ur5_1',vacuumGripper)
		activate_gripper(True)
		rospy.loginfo('\033[94m' + "Box Attached" + '\033[0m')

	#To detach the package from vacuum gripper
	def detach_box(self,box_name):

		scene = self._scene
		eef_link = self._eef_link

		scene.remove_attached_object(eef_link, name=box_name)
		activate_gripper = rospy.ServiceProxy('eyrc/vb/ur5/activate_vacuum_gripper/ur5_1',vacuumGripper)
		activate_gripper(False)
		rospy.loginfo('\033[94m' + "Box Detached" + '\033[0m')
		scene.remove_world_object(box_name)


	# To activate the Conyevor belt
	def activation_of_belt(self, power):
		activate_belt = rospy.ServiceProxy('eyrc/vb/conveyor/set_power',conveyorBeltPowerMsg)
		activate_belt(power)
		rospy.loginfo('\033[94m' + "Conveyor Belt activated." + '\033[0m')

	# To deactivate the Conyevor belt 
	def deactivation_of_belt(self):
		activate_belt = rospy.ServiceProxy('eyrc/vb/conveyor/set_power',conveyorBeltPowerMsg)
		activate_belt(0)
		rospy.loginfo('\033[94m' + "Conveyor Belt de-activated." + '\033[0m')


	# To move the saved trajectory
	def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
		file_path = arg_file_path + arg_file_name
		
		with open(file_path, 'r') as file_open:
			loaded_plan = yaml.load(file_open)
		
		ret = self._group.execute(loaded_plan)
		# rospy.logerr(ret)
		return ret

	# To move the saved trajectory in multiple attempts if failed
	def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
		number_attempts = 0
		flag_success = False

		while ( (number_attempts <= arg_max_attempts) and (flag_success is False) ):
			number_attempts += 1
			flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
			rospy.logwarn("attempts: {}".format(number_attempts) )
			# # self.clear_octomap()
		
		return True


def main():

	# initialize node
	rospy.init_node('node_ur5_1_t4', anonymous=True)
	rospy.sleep(10)

	#Creating object of class ur5_1_Moveit()
	ur5_1 = Ur5_1_Moveit()

	#Adding boxes to rviz
	box_pos_00=[0.280000, 6.590000-7, 1.917499]
	ur5_1.addbox("packagen00",box_pos_00)

	box_pos_01=[0.0, 6.590000-7, 1.917499]
	ur5_1.addbox("packagen01",box_pos_01)

	box_pos_02=[-0.280000, 6.590000-7, 1.917499]
	ur5_1.addbox("packagen02",box_pos_02)

	box_pos_10=[0.280000, 6.590000-7, 1.647499]
	ur5_1.addbox("packagen10",box_pos_10)

	box_pos_11=[0.0, 6.590000-7, 1.647499]
	ur5_1.addbox("packagen11",box_pos_11)

	box_pos_12=[-0.280000, 6.590000-7, 1.647499]
	ur5_1.addbox("packagen12",box_pos_12)

	box_pos_20=[0.280000, 6.590000-7, 1.427499]
	ur5_1.addbox("packagen20",box_pos_20)

	box_pos_21=[0.0, 6.590000-7, 1.427499]
	ur5_1.addbox("packagen21",box_pos_21)

	box_pos_22=[-0.280000, 6.590000-7, 1.427499]
	ur5_1.addbox("packagen22",box_pos_22)

	box_pos_30=[0.280000, 6.590000-7, 1.197499]
	ur5_1.addbox("packagen30",box_pos_30)

	box_pos_31=[0.0, 6.590000-7, 1.197499]
	ur5_1.addbox("packagen31",box_pos_31)

	box_pos_32=[-0.280000, 6.590000-7, 1.197499]
	ur5_1.addbox("packagen32",box_pos_32)

	#joint angles to reach the package
	joint_angles_00_0 = [math.radians(-56),math.radians(-69),math.radians(6),math.radians(-117),math.radians(-126),math.radians(0)]
	
	joint_angles_01_0 = [math.radians(-122),math.radians(-93),math.radians(33),math.radians(-120),math.radians(-66),math.radians(0)]
	
	joint_angles_02_0 = [math.radians(-162),math.radians(-70),math.radians(7),math.radians(-117),math.radians(-26),math.radians(0)]

	joint_angles_10_0 = [math.radians(-56),math.radians(-97),math.radians(85),math.radians(-168),math.radians(-126),math.radians(0)]

	joint_angles_11_0 = [math.radians(-120),math.radians(-116),math.radians(96),math.radians(-160),math.radians(-62),math.radians(0)]

	joint_angles_12_0 = [math.radians(-162),math.radians(-98),math.radians(84),math.radians(-166),math.radians(-20),math.radians(0)]

	joint_angles_20_0 = [math.radians(-56),math.radians(-98),math.radians(89),math.radians(9),math.radians(126),math.radians(0)]

	joint_angles_21_0 = [math.radians(-122),math.radians(-119),math.radians(103),math.radians(15),math.radians(57),math.radians(0)]

	joint_angles_22_0 = [math.radians(55),math.radians(-81),math.radians(-93),math.radians(174),math.radians(-126),math.radians(0)]

	joint_angles_30_0 = [math.radians(-53),math.radians(-76),math.radians(129),math.radians(127),math.radians(-130),math.radians(0)]

	joint_angles_32_0 = [math.radians(54),math.radians(-87),math.radians(-118),math.radians(-156),math.radians(-129),math.radians(0)]

	# drop_joint_angles = [math.radians(-177),math.radians(-34),math.radians(45),math.radians(-98),math.radians(-91),math.radians(0)]
	# drop_joint_angles_1 = [math.radians(0),math.radians(-144),math.radians(-46),math.radians(-79),math.radians(89),math.radians(0)]
	


	pkgs = [(joint_angles_00_0,"packagen00",'packagen00_1.yaml','packagen00_2.yaml'),(joint_angles_01_0,"packagen01",'packagen01_1.yaml','packagen01_2.yaml'),(joint_angles_02_0,"packagen02",'packagen02_1.yaml','packagen02_2.yaml'),
			(joint_angles_10_0,"packagen10",'packagen10_1.yaml','packagen10_2.yaml'),(joint_angles_11_0,"packagen11",'packagen11_1.yaml','packagen11_2.yaml'),(joint_angles_12_0,"packagen12",'packagen12_1.yaml','packagen12_2.yaml'),
			(joint_angles_20_0,"packagen20",'packagen20_1.yaml','packagen20_2.yaml'),(joint_angles_22_0,"packagen22",'packagen22_1.yaml','packagen22_2.yaml'),(joint_angles_32_0,"packagen32",'packagen32_1.yaml','packagen32_2.yaml')]
	
	# To pick and place the package from shelf to belt
	for first,pack,way,drop in pkgs:
		ur5_1.hard_set_joint_angles(first, 5)
		ur5_1.attach_box(pack)
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, way, 5)
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, drop, 5)
		ur5_1.detach_box(pack)
		ur5_1.activation_of_belt(100)


	rospy.spin()


if __name__ == '__main__':
	main()	
	