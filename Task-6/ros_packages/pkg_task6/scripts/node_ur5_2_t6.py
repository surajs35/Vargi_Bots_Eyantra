#! /usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

import yaml
import os
import math
import time
import sys
import copy
import numpy as np

import tf2_ros
import tf2_msgs.msg

from std_srvs.srv import Empty
from std_msgs.msg import String

from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.msg import LogicalCameraImage

from pkg_task6.srv import get_info
from pkg_task6.msg import shipping

class Ur5_2_Moveit:
    """
        This class is used to control the Ur5_2 robot and its Rviz planning scene.
    """

    # Constructor
    def __init__(self):
        '''This used to construct the rviz planning scene of ur5_2 robot.''' 

        self._robot_ns = '/'  + "ur5_2"
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
        self._group.allow_replanning(True)
        self.flag_position = 0

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()
        # rospy.loginfo(self._group.get_current_pose().pose)
        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')
            
        rospy.loginfo('\033[94m' + " >>> Ur5_2 Moveit init done." + '\033[0m')


    # To set the joint angles of ur5
    def set_joint_angles(self, arg_list_joint_angles):
        """
            This function is used to set the joint angles in desired manner.

            Arguments:
                arg_list_joint_angles (lists): It is a list of joint angles.

            Return:
                flag_plan (boolean): It tells us whether the the joint angles have been set succesfully or not.
        """

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
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

    # To add package in Rviz
    def addbox(self, box_name, box_pos):
        """
            This function adds a box mesh in Rviz

            Arguments:
                box_name (string): Box name
                box_pos (list): x,y,z positions of box in rviz.
        """
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
        """
            This function is used to attach the package to vacuum gripper of Ur5_2.

            Arguments:
                box_name (string): Box name to attach it to the end effector.
        """
        robot = self._robot
        scene = self._scene
        eef_link = self._eef_link
        group_names = self._group_names

        grasping_group = self._planning_group
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        activate_gripper = rospy.ServiceProxy('eyrc/vb/ur5/activate_vacuum_gripper/ur5_2',vacuumGripper)
        activate_gripper(True)
        rospy.loginfo('\033[94m' + "Box Attached" + '\033[0m')

    # To detach box from rviz
    def detach_box(self,box_name):
        """
            This function is used to detatch the box from Vacuum gripper.

            Arguments:
                box_name (string): To detach the box in rviz.
        """
        scene = self._scene
        eef_link = self._eef_link

        scene.remove_attached_object(eef_link, name=box_name)
        activate_gripper = rospy.ServiceProxy('eyrc/vb/ur5/activate_vacuum_gripper/ur5_2',vacuumGripper)
        activate_gripper(False)
        rospy.loginfo('\033[94m' + "Box Detached" + '\033[0m')
        scene.remove_world_object(box_name)

    # To activate the Conyevor belt
    def activation_of_belt(self, power):
        """
            This is used to activate the conveyor belt.

            Arguments:
                power (int): To specify the power of the belt.
        """
        activate_belt = rospy.ServiceProxy('eyrc/vb/conveyor/set_power',conveyorBeltPowerMsg)
        activate_belt(power)
        rospy.loginfo('\033[94m' + "Conveyor Belt activated." + '\033[0m')

    # To deactivate the Conyevor belt 
    def deactivation_of_belt(self):
        """
            This is used to deactive the belt.
        """
        activate_belt = rospy.ServiceProxy('eyrc/vb/conveyor/set_power',conveyorBeltPowerMsg)
        activate_belt(0)
        rospy.loginfo('\033[94m' + "Conveyor Belt de-activated." + '\033[0m')


    # To detect the package through Logical Camera 
    def callback_func(self,data):
        """
            This is a callback function to deactive the belt when the box comes under the Logical camera 2.

            Arguments:
                data (string): It is the data obtained from the camera.

        """
        try:
            for i in range(len(data.models)):
                if(data.models[i].type.startswith("packagen")  and data.models[i].pose.position.y <= 0.2):
                    self.deactivation_of_belt()
                    self._box_name = data.models[i].type
                    self.flag_position = 1

        except(Exception):
            rospy.loginfo("callback_func_error")

    # To move the ur5 in cartesian path
    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        """
            This is used for cartesian translation of the end effector of the robot.

            Arguments:
                trans_x (int): Amount of traversing in x direction.
                trans_y (int): Amount of traversing in y direction.
                trans_z (int): Amount of traversing in z direction
        """
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

        # self._computed_plan = plan
        num_pts = len(plan.joint_trajectory.points)
        if (num_pts >= 3):
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]
        
        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)

    # To set the joint angles multiple attempts if failed
    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):
        """
            This function attempts to set joint angles with given number of attempts.

            Arguments:
                arg_list_joint_angles (lists): List of joint angles of Ur5_2
                arg_max_attempts (int): Number of attempts it can retry.
        """

        number_attempts = 0
        flag_success = False
        
        while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # self.clear_octomap()


class SimpleClient:
    """
        This class is simple action client is used for updating spreadsheet and to get the box color.
    """

    # Constructor
    def __init__(self):
        
        rospy.wait_for_service('get_box_info')
        try:
            self.service_box_info = rospy.ServiceProxy('get_box_info', get_info)
            self._handle_ros_pub = rospy.Publisher("/eyrc/vb/ordershipped", shipping, queue_size=10)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    # To get the box colour through sevice 
    def box_colour_detect(self, box_name):
        try:    
            self.msg = self.service_box_info(box_name)
            return self.msg.boxcolour
        except rospy.ServiceException as e:
            print("Service call failed:: %s"%e)

    # To handle the order shipment package name.
    def order_ship(self, pkg_name):
        ord_ship = shipping()
        ord_ship.order_shipped = pkg_name
        rospy.loginfo(ord_ship.order_shipped)
        self._handle_ros_pub.publish(ord_ship)


def main():
    # 1. Initialize ROS Node
    rospy.init_node('node_ur5_2_t6', anonymous=True)

    rospy.sleep(5)

    # Creating Object 
    ur5_2 = Ur5_2_Moveit()
    client = SimpleClient()

    # rospy.sleep(10)



    # JointAngles of home pose
    lst_joint_angles_home_pose = [math.radians(-4),
                          math.radians(-158),
                          math.radians(-23),
                          math.radians(-89),
                          math.radians(90),
                          math.radians(0)]

    # JointAngles to reach red_bin 
    lst_joint_angles_red_bin = [math.radians(-85),
                          math.radians(-102),
                          math.radians(-79),
                          math.radians(-66),
                          math.radians(90),
                          math.radians(0)]

    # JointAngles to reach yellow_bin 
    lst_joint_angles_yellow_bin = [math.radians(-175),
                          math.radians(-102),
                          math.radians(-79),
                          math.radians(-66),
                          math.radians(90),
                          math.radians(0)]

    # JointAngles to reach green_bin 
    lst_joint_angles_green_bin = [math.radians(95),
                          math.radians(-102),
                          math.radians(-79),
                          math.radians(-66),
                          math.radians(90),
                          math.radians(0)]

    #Subscribing to the Camera topic to detect packages 
    p = rospy.Subscriber("/eyrc/vb/logical_camera_2", LogicalCameraImage, ur5_2.callback_func,queue_size=10)

    ur5_2.hard_set_joint_angles(lst_joint_angles_home_pose,5)
    box_pose = list() 
    box_pose.append(ur5_2._group.get_current_pose().pose.position.x)
    box_pose.append(ur5_2._group.get_current_pose().pose.position.y)
    box_pose.append(ur5_2._group.get_current_pose().pose.position.z-0.2)
    done_pkgs = list()
    box_colour_ = ""
    
    # Loop to pick and place the packages from Conveyor belt
    while not rospy.is_shutdown():
        try:
            if ur5_2.flag_position == 1:
                box_colour_  = client.box_colour_detect(ur5_2._box_name)

                if(box_colour_ == "red"):
                    ur5_2.addbox(ur5_2._box_name, box_pose)                    
                    ur5_2.attach_box(ur5_2._box_name)
                    ur5_2.hard_set_joint_angles(lst_joint_angles_red_bin,5)
                    ur5_2.activation_of_belt(100)
                    ur5_2.detach_box(ur5_2._box_name)
                    ur5_2.flag_position = 0
                    ur5_2.hard_set_joint_angles(lst_joint_angles_home_pose,5)
                    client.order_ship(ur5_2._box_name)
                    
                    
                elif(box_colour_ == "yellow"):
                    ur5_2.addbox(ur5_2._box_name, box_pose)                    
                    ur5_2.attach_box(ur5_2._box_name)
                    ur5_2.hard_set_joint_angles(lst_joint_angles_yellow_bin,5)
                    ur5_2.activation_of_belt(100)
                    ur5_2.detach_box(ur5_2._box_name)
                    ur5_2.flag_position = 0
                    ur5_2.hard_set_joint_angles(lst_joint_angles_home_pose,5)
                    client.order_ship(ur5_2._box_name)
                     
                elif(box_colour_ == "green"): 
                    ur5_2.addbox(ur5_2._box_name, box_pose)                   
                    ur5_2.attach_box(ur5_2._box_name)
                    ur5_2.hard_set_joint_angles(lst_joint_angles_green_bin,5)
                    ur5_2.activation_of_belt(100)
                    ur5_2.detach_box(ur5_2._box_name)
                    ur5_2.flag_position = 0
                    ur5_2.hard_set_joint_angles(lst_joint_angles_home_pose,5)
                    client.order_ship(ur5_2._box_name)

                
        except(Exception):
            continue
  

if __name__ == "__main__":
    main()