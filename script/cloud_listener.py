#! /usr/bin/python

import sys
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2

class CloudListener:

	def __init__(self, topic1, topic2):
		self.subscriber1 = rospy.Subscriber(topic1, PointCloud2, self.on_topic1_received)
		print('CloudListener: subscribed ' + topic1)
		self.subscriber2 = rospy.Subscriber(topic2, PointCloud2, self.on_topic2_received)
		print('CloudListener: subscribed ' + topic2)
		self.topic1_name = topic1
		self.topic2_name = topic2
		self.topic1_shift = np.zeros(3)
		self.topic2_shift = np.zeros(3)
		print('CloudListener: initialized')
		self.message_cnt = 0

	def on_topic1_received(self, data):
		rospy.loginfo(rospy.get_caller_id() + " " + self.topic1_name + " received")
		self.message_cnt += 1

	def on_topic2_received(self, data):
		rospy.loginfo(rospy.get_caller_id() + " " + self.topic2_name + " received")
		self.message_cnt += 1

if __name__ == '__main__':
	if len(sys.argv) != 5:
		print("CloudListener: error: insert two topic names as input arguments")
	else:
		listener = CloudListener(sys.argv[1], sys.argv[2])
		rospy.init_node('cloud_listener', anonymous=True)
		rospy.spin()
		print('CloudListener: finished spinning')
		print('Message count: ' + str(listener.message_cnt))
