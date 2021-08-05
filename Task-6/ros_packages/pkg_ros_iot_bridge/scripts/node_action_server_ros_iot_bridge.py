#!/usr/bin/env python

# ROS Node - Action Server - IoT ROS Bridge

import rospy
import actionlib
import threading
import json
import datetime
import time

from pkg_ros_iot_bridge.msg import msgRosIotAction      # Message Class that is used by ROS Actions internally
from pkg_ros_iot_bridge.msg import msgRosIotGoal        # Message Class that is used for Goal Messages
from pkg_ros_iot_bridge.msg import msgRosIotResult      # Message Class that is used for Result Messages
from pkg_ros_iot_bridge.msg import msgRosIotFeedback    # Message Class that is used for Feedback Messages    

from pkg_ros_iot_bridge.msg import msgMqttSub           # Message Class for MQTT Subscription Messages

from pyiot import iot                                   # Custom Python Module to perfrom MQTT Tasks


class RosIotBridgeActionServer:

	# Constructor
	def __init__(self):
		# Initialize the Action Server
		self._as = actionlib.ActionServer('/action_ros_iot',
										  msgRosIotAction,
										  self.on_goal,
										  self.on_cancel,
										  auto_start=False)

		'''
			* self.on_goal - It is the fuction pointer which points to a function which will be called
							 when the Action Server receives a Goal.

			* self.on_cancel - It is the fuction pointer which points to a function which will be called
							 when the Action Server receives a Cancel Request.
		'''

		# Read and Store IoT Configuration data from Parameter Server
		param_config_iot = rospy.get_param('config_pyiot')
		self._config_mqtt_server_url = param_config_iot['mqtt']['server_url']
		self._config_mqtt_server_port = param_config_iot['mqtt']['server_port']
		self._config_mqtt_sub_topic = param_config_iot['mqtt']['topic_sub']
		self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']
		self._config_mqtt_qos = param_config_iot['mqtt']['qos']
		self._config_mqtt_sub_cb_ros_topic = param_config_iot['mqtt']['sub_cb_ros_topic']
		self._config_google_apps_spread_sheet_id = param_config_iot['google_apps']['spread_sheet_id']
		print(param_config_iot)

		self.order_time = list()
		
		# Initialize ROS Topic Publication
		# Incoming message from MQTT Subscription will be published on a ROS Topic (/ros_iot_bridge/mqtt/sub).
		# ROS Nodes can subscribe to this ROS Topic (/ros_iot_bridge/mqtt/sub) to get messages from MQTT Subscription.
		self._handle_ros_pub = rospy.Publisher(self._config_mqtt_sub_cb_ros_topic, msgMqttSub, queue_size=10)


		# Subscribe to MQTT Topic (eyrc/xYzqLm/iot_to_ros) which is defined in 'config_iot_ros.yaml'.
		# self.mqtt_sub_callback() function will be called when there is a message from MQTT Subscription.
		ret = iot.mqtt_subscribe_thread_start(  self.mqtt_sub_callback, 
														self._config_mqtt_server_url, 
														self._config_mqtt_server_port, 
														self._config_mqtt_sub_topic, 
														self._config_mqtt_qos   )
		if(ret == 0):
			rospy.loginfo("MQTT Subscribe Thread Started")
		else:
			rospy.logerr("Failed to start MQTT Subscribe Thread")


		# Start the Action Server
		self._as.start()
		
		rospy.loginfo("Started ROS-IoT Bridge Action Server.")
		

	
	# This is a callback function for MQTT Subscriptions
	def mqtt_sub_callback(self, client, userdata, message):
		'''
		This function gets the messages published to the MQTT Topic
		and publishes the message to another topic such that it can 
		received by ROS nodes and update the spreadsheet
		
		Arguments:
			message(str) : message that has been passed to the MQTT topic
			client : The messages that has been sent by the user.


		'''
		payload = str(message.payload.decode("utf-8"))
	
		print("[MQTT SUB CB] Message: ", payload)
		print("[MQTT SUB CB] Topic: ", message.topic)

		in_ord = json.loads(payload)
		iot.spreadsheet_update({"id":"IncomingOrders","Order ID":in_ord["order_id"],"Order Date and Time":in_ord["order_time"],
								"Item":in_ord["item"],"Priority":self.get_priority(in_ord["item"]),"Order Quantity":in_ord["qty"],
								"City":in_ord["city"],"Longitude":in_ord["lon"],"Latitude":in_ord["lat"],"Cost":self.get_cost(in_ord["item"])})

		iot.spreadsheet_update({"id":"Dashboard","Order ID":in_ord["order_id"],"Order Time":in_ord["order_time"],
								"Item":in_ord["item"],"Priority":self.get_priority(in_ord["item"]),"Quantity":in_ord["qty"],
								"City":in_ord["city"],"Longitude":in_ord["lon"],"Latitude":in_ord["lat"]})

		in_ord.update({"Priority":self.get_priority(in_ord["item"])})
		payload = json.dumps(in_ord)

		self.order_time.append((in_ord["order_id"],in_ord["order_time"]))

		msg_mqtt_sub = msgMqttSub()
		msg_mqtt_sub.timestamp = rospy.Time.now()
		msg_mqtt_sub.topic = message.topic
		msg_mqtt_sub.message = payload
		
		self._handle_ros_pub.publish(msg_mqtt_sub) #pubish to the topic 
		
		
	
	def get_priority(self,item):
		'''
		This function returns the priority for the items

		Argumnets:
			item(str): it contains the name of the item

		Return:
			The Prioirty of the item is returned
		
		'''
		if item == "Medicine":
			return "HP"
		elif item == "Food":
			return "MP"
		elif item == "Clothes":
			return "LP"

	def get_cost(self,item):
		'''
		This function returns the cost of the items

		Arguments:
			item(str):it contains the name of the items

		Return:
			The cost of the item is returned
		'''
		if item == "Medicine":
			return "450"
		elif item == "Food":
			return "250"
		elif item == "Clothes":
			return "150"


	def get_time_str(self):
		'''
		This funtion returns the present time in a string format

		Return:
			str_time(str): This contains the present date and time 

		'''
		timestamp = int(time.time())
		value = datetime.datetime.fromtimestamp(timestamp)
		str_time = value.strftime('%Y-%m-%d %H:%M:%S')

		return str_time

	def get_est_date(self, priority):
		'''
		This function returns the estimated delivery date for the product

		Arguments:
			priority(str): This contains the priority of the item 

		Return:
			str_time(str): This returns estimated date and time of the package 

		'''
		presentday = datetime.datetime.now()
		if priority == "HP":
			est_date = presentday + datetime.timedelta(1)
		elif priority == "MP":
			est_date = presentday + datetime.timedelta(3)
		elif priority == "LP":
			est_date = presentday + datetime.timedelta(5)

		str_time = est_date.strftime('%Y-%m-%d')

		return str_time

	def get_time_taken(self, ord_id, time_ship):
		'''
		This returns the time difference between the ship date and order date

		Arguments:
			ord_id(str): This contains order id of the item
			time_ship(str) : This contains the shipping time

		Return:
			diff(int):This contains the difference of order time and ship time in seconds


		'''
		for ids,tim in self.order_time:
			if ids == ord_id:
				ord_date_obj = datetime.datetime.strptime(tim, '%Y-%m-%d %H:%M:%S')
				ship_date_obj = datetime.datetime.strptime(time_ship, '%Y-%m-%d %H:%M:%S')
				diff = ship_date_obj - ord_date_obj
				return diff.total_seconds()


	# This function will be called when Action Server receives a Goal
	def on_goal(self, goal_handle):
		'''
		This function will be called when a goal is received by the server

		Arguments:
			goal_handle(object):It contains on the goal sent by the client


		'''
		goal = goal_handle.get_goal()

		rospy.loginfo("Received new goal from Client")
		rospy.loginfo(goal)

		# Validate incoming goal parameters
		if(goal.protocol == "mqtt"):
			
			if((goal.mode == "pub") or (goal.mode == "sub")):
				goal_handle.set_accepted()
				
				# Start a new thread to process new goal from the client (For Asynchronous Processing of Goals)
				# 'self.process_goal' - is the function pointer which points to a function that will process incoming Goals
				thread = threading.Thread(  name="worker",
											target=self.process_goal,
											args=(goal_handle,) )
				thread.start()

			else:
				goal_handle.set_rejected()
				return
		
		else:
			goal_handle.set_rejected()
			return


	# This function is called is a separate thread to process Goal.
	def process_goal(self, goal_handle):
		'''
		This Function process the goal sent by the client and updates the spreadsheet based on the order processed

		Arguments:
			goal_handle(object):It contains on the goal sent by the client


		'''

		flag_success = False
		result = msgRosIotResult()

		goal_id = goal_handle.get_goal_id()
		rospy.loginfo("Processing goal : " + str(goal_id.id))

		goal = goal_handle.get_goal()

		
		# Goal Processing
		if(goal.protocol == "mqtt"):
			rospy.logwarn("MQTT")

			if(goal.mode == "pub"):
				rospy.logwarn("MQTT PUB Goal ID: " + str(goal_id.id))

				rospy.logwarn(goal.topic + " > " + goal.message)
				
				if goal.message == "Inventory":
					inv_list = json.loads(goal.ims)
					iot.spreadsheet_update({"id":str(goal.message), "SKU":str(inv_list[1]), "Item":str(inv_list[2]), 
						"Priority":str(inv_list[3]), "Storage Number":str(inv_list[4]), "Cost":inv_list[5], "Quantity":inv_list[6]})
					
				ret = iot.mqtt_publish( self._config_mqtt_server_url,self._config_mqtt_server_port,goal.topic,goal.message,self._config_mqtt_qos)

				if goal.message == "Order_dispatched":
					x = self.get_time_str()
					ord_dis = json.loads(goal.ims)
					iot.spreadsheet_update({"id":"OrdersDispatched","Order ID":ord_dis["order_id"],"City":ord_dis["city"],"Item":ord_dis["item"],
											"Priority":ord_dis["Priority"],"Dispatch Quantity":1,"Cost":self.get_cost(ord_dis["item"]),
											"Dispatch Status":"YES","Dispatch Date and Time": x})

					iot.spreadsheet_update({"id":"Dashboard","Order Dispatched":"YES", "Dispatch Time": x, "Order ID":ord_dis["order_id"]})
					

				if goal.message == "Order_shipped":
					x = self.get_time_str()
					ord_dis = json.loads(goal.ims)
					iot.spreadsheet_update({"id":"OrdersShipped","Order ID":ord_dis["order_id"],"City":ord_dis["city"],"Item":ord_dis["item"],
											"Priority":ord_dis["Priority"],"Shipped Quantity":1,"Cost":self.get_cost(ord_dis["item"]),
											"Shipped Status":"YES","Shipped Date and Time": x,"Estimated Time of Delivery":self.get_est_date(ord_dis["Priority"])})

				

					iot.spreadsheet_update({"id":"Dashboard","Order Shipped":"YES", "Shipping Time": x, "Order ID":ord_dis["order_id"],
											"Time Taken":self.get_time_taken(ord_dis["order_id"],x)})
					
				if(ret == 0):
					rospy.loginfo("MQTT Publish Successful.")
					result.flag_success = True
				else:
					rospy.logerr("MQTT Failed to Publish")
					result.flag_success = False

			elif(goal.mode == "sub"):
				rospy.logwarn("MQTT SUB Goal ID: " + str(goal_id.id))
				rospy.logwarn(goal.topic)

				ret = iot.mqtt_subscribe_thread_start(  self.mqtt_sub_callback, 
														self._config_mqtt_server_url, 
														self._config_mqtt_server_port, 
														goal.topic, 
														self._config_mqtt_qos   )
				if(ret == 0):
					rospy.loginfo("MQTT Subscribe Thread Started")
					result.flag_success = True
				else:
					rospy.logerr("Failed to start MQTT Subscribe Thread")
					result.flag_success = False

		rospy.loginfo("Send goal result to client")
		if (result.flag_success == True):
			rospy.loginfo("Succeeded")
			goal_handle.set_succeeded(result)
		else:
			rospy.loginfo("Goal Failed. Aborting.")
			goal_handle.set_aborted(result)

		rospy.loginfo("Goal ID: " + str(goal_id.id) + " Goal Processing Done.")

	
		

	# This function will be called when Goal Cancel request is send to the Action Server
	def on_cancel(self, goal_handle):
		'''
		This function cancels the goal sent by the client
		
		Arguments:
			goal_handle(object):It contains on the goal sent by the client

		'''
		rospy.loginfo("Received cancel request.")
		goal_id = goal_handle.get_goal_id()


# Main
def main():
	rospy.init_node('node_action_server_ros_iot_bridge')

	action_server = RosIotBridgeActionServer()

	rospy.spin()



if __name__ == '__main__':
	main()