#!/usr/bin/env python

# ROS Node - Action Client 

import rospy
import actionlib
import time
import json
import datetime

from pkg_ros_iot_bridge.msg import msgMqttSub

from pkg_ros_iot_bridge.msg import msgRosIotAction      # Message Class that is used by ROS Actions internally
from pkg_ros_iot_bridge.msg import msgRosIotGoal        # Message Class that is used for Goal Messages
from pkg_ros_iot_bridge.msg import msgRosIotResult      # Message Class that is used for Result Messages

from pkg_task6.msg import com_ur5_1Action
from pkg_task6.msg import com_ur5_1Goal
from pkg_task6.msg import shipping

class SimpleActionClient:
	"""This class is action client. To send goals to other nodes.
	"""
	
	# Constructor
	def __init__(self):
		"""
		This initializes the ActionClient.
		"""
		# Initialize Simple Action Server
		self._ac = actionlib.SimpleActionClient('/action_ur5', com_ur5_1Action)

		self._ac.wait_for_server()
		rospy.loginfo("Action server is up, we can send new goals!")
		self.res = ""
		self.res1= ""
		self.ims_msg = ""
		self.pkg_name = ""
		self.order_list = list()
		self.order_list_pkg = list()
		self.ros_iot_order = ""
		self.ros_iot_ship = ""
		self.inc_order = ""
		self.count = 0
		self.ord_ros = 0
		self.flag = 0

	def send_inv_goal(self, arg_msg):
		"""
			To send inventory goal to Ros Iot bridge node.

			Argument:
				arg_msg (string): The message goal to send. 
		"""
		goal = com_ur5_1Goal()
		goal.ims_info = arg_msg
		self.ims_msg = arg_msg

		self._ac.send_goal(goal, done_cb=self.done_callback,feedback_cb=self.feedback_callback)
		rospy.loginfo("Goal has been sent.")

	# Function print result on Goal completion
	def done_callback(self, status, result):
		"""
			This function is called when the result is received for the goal sent.

			Arguments:
				Status (int): Status whether the goal is processed or not from the number.
		"""
		rospy.loginfo("Status is : " + str(status))
		rospy.loginfo("Result is : " + str(result))
		# rospy.loginfo(json.loads(result.pkg_info))
		if self.ims_msg == "Inventory":
			self.res = self.inv_update(json.loads(result.pkg_info))
			self.res1 = self.res
			rospy.loginfo(self.res)
		elif self.ims_msg.startswith("packagen"):
			for i in self.order_list:
				if i["package_name"] == self.ims_msg:
					self.ros_iot_order = i
					break
			
			self.res = result.order_dispatched
			self.ord_ros = 1
			self.goal_handling()
	
	def goal_handling(self):
		"""
			To send goals to Ur5_1 to pick and place the packages required.
		"""
		try:
			self.send_inv_goal(self.order_list_pkg[0][1])
			self.order_list_pkg.pop(0) 
			rospy.sleep(2)
			self.res = ""		
		except(Exception):
			rospy.loginfo("No Goal Found")
			self.count = 0

	# Function to print feedback while Goal is being processed
	def feedback_callback(self, feedback):
		"""
			It gives the feedback of processing goal.

			Argument:
				feedback (int): It provides the percentage of goal completed.
		"""
		rospy.loginfo(feedback)

	def inv_update(self, packages):
		"""
			This function creates a list of parameter required to update spreadsheet.

			Argument:
				packages (dictionary): It contains key value pair of package name and its color.

			Return:
				inv1 (list): List of packages present in the inventory.
		"""
		inv1 = list()
		x = datetime.datetime.now()
		for k,v in packages.items():
			if v == "red":
				inv1.append((str(k[-2:]),"R"+k[-2:]+x.strftime("%m")+x.strftime("%y"), "Medicine", "HP", "R"+k[-2:-1]+" "+"C"+k[-1], 450, 1))
			elif v == "yellow":
				inv1.append((str(k[-2:]),"Y"+k[-2:]+x.strftime("%m")+x.strftime("%y"), "Food", "MP", "R"+k[-2:-1]+" "+"C"+k[-1], 250, 1))
			elif v == "green":
				inv1.append((str(k[-2:]),"G"+k[-2:]+x.strftime("%m")+x.strftime("%y"), "Clothes", "LP", "R"+k[-2:-1]+" "+"C"+k[-1], 150, 1))

		return sorted(inv1)

	def func_dis(self,mymsg):
		"""
			This function sends the goal to pick the packages ordered.

			Argument:
				mymsg (object): Incoming order from mqtt dictionary encoded as string.
		"""
		inc_ord = json.loads(mymsg.message)
		
		for i in range(len(self.res1)):
			if inc_ord["item"] == self.res1[i][2] and inc_ord["item"] == "Medicine" :
				self.pkg_name = "packagen"+str(self.res1[i][0])
				self.order_list_pkg.append(("1",self.pkg_name))
				self.res1.pop(i)
				break
			elif inc_ord["item"] == self.res1[i][2] and inc_ord["item"] == "Food":
				self.pkg_name = "packagen"+str(self.res1[i][0])
				self.order_list_pkg.append(("2",self.pkg_name))
				self.res1.pop(i)
				break
			elif inc_ord["item"] == self.res1[i][2] and inc_ord["item"] == "Clothes":
				self.pkg_name = "packagen"+str(self.res1[i][0])
				self.order_list_pkg.append(("3",self.pkg_name))
				self.res1.pop(i)
				break

		inc_ord.update({"package_name":self.pkg_name})
		self.order_list.append(inc_ord)

		self.order_list_pkg.sort()
		rospy.loginfo(self.order_list_pkg)
		rospy.loginfo(self.order_list)

		if self.count == 0:
			sim_time = rospy.get_rostime()
			if len(self.order_list_pkg)>1:
				self.send_inv_goal(self.order_list_pkg[0][1])
				self.order_list_pkg.pop(0) 
				self.count = self.count+1
			else:
				while not rospy.is_shutdown():
					sim_time_now = rospy.get_rostime()
					if sim_time_now.secs == sim_time.secs+7:
						if len(self.order_list_pkg) == 1 and self.count!=1:
							self.send_inv_goal(self.order_list_pkg[0][1])
							self.order_list_pkg.pop(0) 
							self.count = self.count+1
							break
						else:
							break

	def func_ship(self, mymsg):
		"""
			This function specifies which package and order has been shipped .

			Argument:
				mymsg (object): This contains which package has been shipped.
		"""
		for i in self.order_list:
			rospy.loginfo(mymsg)
			if i["package_name"] == mymsg.order_shipped:
				self.ros_iot_ship = i
				self.flag = 1



class RosIotBridgeActionClient:
	"""
		This class is used as aciton client to send goal to ros iot bridge.
	"""

	# Constructor
	def __init__(self):
		"""
			It is used to intialize the action client.
		"""

		# Initialize Action Client
		self._ac = actionlib.ActionClient('/action_ros_iot', msgRosIotAction)
		
		# Dictionary to Store all the goal handels
		self._goal_handles = {}

		# Store the MQTT Topic on which to Publish in a variable
		param_config_iot = rospy.get_param('config_pyiot')
		self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']

		# Wait for Action Server that will use the action - '/action_iot_ros' to start
		self._ac.wait_for_server()
		rospy.loginfo("Action server up, we can send goals.")

	
	# This function will be called when there is a change of state in the Action Client State Machine
	def on_transition(self, goal_handle):
		"""
			This function will be called when there is a change of state in the Action Client State Machine
			from on_goal() to on_transition(). goal_handle generated by send_goal() is used here.

			Argument:
				goal_handle (object): It contains the goal that has been sent to action server. 
		"""
		
		# from on_goal() to on_transition(). goal_handle generated by send_goal() is used here.
		
		result = msgRosIotResult()

		index = 0
		for i in self._goal_handles:
			if self._goal_handles[i] == goal_handle:
				index = i
				break

		rospy.loginfo("Transition Callback. Client Goal Handle #: " + str(index))
		rospy.loginfo("Comm. State: " + str(goal_handle.get_comm_state()) )
		rospy.loginfo("Goal Status: " + str(goal_handle.get_goal_status()) )
		
		# Comm State - Monitors the State Machine of the Client which is different from Server's
		# Comm State = 2 -> Active
		# Comm State = 3 -> Wating for Result
		# Comm State = 7 -> Done
		
		# if (Comm State == ACTIVE)
		if goal_handle.get_comm_state() == 2:
			rospy.loginfo(str(index) + ": Goal just went active.")
		
		# if (Comm State == DONE)
		if goal_handle.get_comm_state() == 7:
			rospy.loginfo(str(index) + ": Goal is DONE")
			rospy.loginfo(goal_handle.get_terminal_state())
			
			# get_result() gets the result produced by the Action Server
			result = goal_handle.get_result()
			rospy.loginfo(result.flag_success)

			if (result.flag_success == True):
				rospy.loginfo("Goal successfully completed. Client Goal Handle #: " + str(index))
			else:
				rospy.loginfo("Goal failed. Client Goal Handle #: " + str(index))


	# This function is used to send Goals to Action Server
	def send_goal(self, arg_protocol, arg_mode, arg_topic, arg_message, arg_ims):
		"""
			This is for sending goal to update spreadsheets and also to mqtt

			Arguments:
				arg_protocol (string): Specifying the protocol(mqtt).
				arg_mode (string): Specfying whether to subscribe or publish to mqtt topic 
				arg_topic (string): The topic to publish/subscribe.
				arg_message (string): Message to publish/subscribe.
				arg_ims (string): Dictionary encoded as string containing the parameter required to update the spreadsheet. 
		"""
		# Create a Goal Message object
		goal = msgRosIotGoal()

		goal.protocol = arg_protocol
		goal.mode = arg_mode
		goal.topic = arg_topic
		goal.message = arg_message 
		goal.ims = arg_ims


		rospy.loginfo("Send goal.")
		
		# self.on_transition - It is a function pointer to a function which will be called when 
		#                       there is a change of state in the Action Client State Machine
		goal_handle = self._ac.send_goal(goal, self.on_transition, None)

		return goal_handle

	


# Main Function
def main():

	# 1. Initialize ROS Node
	rospy.init_node('node_vb_client_t6')

	while not rospy.is_shutdown():
		
			sim_time_now = rospy.get_rostime()
			if sim_time_now.secs > 25:
				break

	rospy.loginfo("CLINET STARTED")

	# 2. Create a object for Simple Action Client.
	action_client = RosIotBridgeActionClient()
	obj_server = SimpleActionClient()

	obj_server.send_inv_goal("Inventory")
	rospy.sleep(1)
	# rospy.loginfo(obj_server.res)
	for i in obj_server.res:
		goal_handle1 = action_client.send_goal("mqtt", "pub", action_client._config_mqtt_pub_topic, "Inventory", json.dumps(i))
		rospy.sleep(2)

	#Subscribe to the Ros Topic to get the start message         
	sub_msg = rospy.Subscriber("/ros_iot_bridge/mqtt/sub", msgMqttSub, obj_server.func_dis)
	sub_msg = rospy.Subscriber("/eyrc/vb/ordershipped", shipping, obj_server.func_ship)
	
	while True:
		if obj_server.ord_ros == 1:
			goal_handle = action_client.send_goal("mqtt", "pub", action_client._config_mqtt_pub_topic, "Order_dispatched", json.dumps(obj_server.ros_iot_order))
			rospy.sleep(1)
			obj_server.ord_ros = 0

		if obj_server.flag == 1:
			goal_handle = action_client.send_goal("mqtt", "pub", action_client._config_mqtt_pub_topic, "Order_shipped", json.dumps(obj_server.ros_iot_ship))
			rospy.sleep(1)
			obj_server.flag = 0

	# 4. Loop forever
	rospy.spin()


if __name__ == '__main__':
	main()