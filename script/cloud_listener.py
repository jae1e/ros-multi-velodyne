#! /usr/bin/python

import sys
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from pointclouds import *

class CloudListener:

	def __init__(self, topic1, topic2):
		self.publisher1 = rospy.Publisher('translated/' + topic1, PointCloud2, queue_size=10)
		self.publisher2 = rospy.Publisher('translated/' + topic2, PointCloud2, queue_size=10)

		self.subscriber1 = rospy.Subscriber(topic1, PointCloud2, self.on_topic1_received)
		rospy.loginfo('CloudListener: subscribed ' + topic1)
		self.subscriber2 = rospy.Subscriber(topic2, PointCloud2, self.on_topic2_received)
		rospy.loginfo('CloudListener: subscribed ' + topic2)

		self.topic1_name = topic1
		self.topic2_name = topic2

		self.topic1_shift = np.array([1,1,1], dtype=np.float32)
		self.topic2_shift = np.array([2,2,2], dtype=np.float32)
		rospy.loginfo('CloudListener: initialized')

	def process_points(self, data, topic, shift, publisher):
		points = pointcloud2_to_xyz_array(data, dtype=np.float32)
		rospy.loginfo('CloudListener: ' + topic + ' shape: ' + str(points.shape))
		shifted_points = np.copy(points)
		shifted_points[:] += shift
		shifted_msg = xyz_float32_array_to_pointcloud2(shifted_points, frame_id='velodyne')
		publisher.publish(shifted_msg)
		rospy.loginfo('CloudListener: translated/' + topic + ' published')

	def on_topic1_received(self, data):
		# rospy.loginfo(rospy.get_caller_id() + " " + self.topic1_name + " received")
		self.process_points(data, self.topic1_name, self.topic1_shift, self.publisher1)

	def on_topic2_received(self, data):
		# rospy.loginfo(rospy.get_caller_id() + " " + self.topic2_name + " received")
		self.process_points(data, self.topic2_name, self.topic2_shift, self.publisher2)

if __name__ == '__main__':
	if len(sys.argv) != 5:
		rospy.loginfo("CloudListener: error: insert two topic names as input arguments")
	else:
		listener = CloudListener(sys.argv[1], sys.argv[2])
		rospy.init_node('cloud_listener', anonymous=True)
		rospy.spin()
		rospy.loginfo('CloudListener: finished spinning')
