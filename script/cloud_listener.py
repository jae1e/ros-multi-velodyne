#! /usr/bin/python

import sys
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from pointclouds import *
from sklearn.cluster import DBSCAN

class Cluster:
	
	def __init__(self, id, resolution):
		self.id = id
		self.resolution = resolution
		self.min = np.zeros(3)
		self.max = np.zeros(3)

	def add_points(self, xy_id, z):
		xy = xy_id.astype(float) * self.resolution
		self.min[:2] = np.min(xy, axis=0)
		self.max[:2] = np.max(xy, axis=0)
		self.min[2] = np.min(z) 	
		self.max[2] = np.max(z)

class CloudListener:

	def __init__(self, topic):
		self.subscriber = rospy.Subscriber(topic, PointCloud2, self.on_topic_received)
		rospy.loginfo('CloudListener: subscribed ' + topic)
		self.publisher = rospy.Publisher('markers', MarkerArray, queue_size=10)
		self.topic_name = topic
		# init markers
		marker_count = 100
		self.marker_array = MarkerArray()
		for i in range(marker_count):
			m = Marker()
			m.id = i
			m.header.frame_id = 'velodyne'
			m.type = m.CUBE
			m.action = m.ADD
			m.pose.position.x = 0.0
			m.pose.position.y = 0.0
			m.pose.position.z = 0.0
			m.pose.orientation.x = 0.0
			m.pose.orientation.y = 0.0
			m.pose.orientation.z = 0.0
			m.pose.orientation.w = 0.0
			m.scale.x = 0.2
			m.scale.y = 0.2
			m.scale.z = 0.2
			m.color.a = 0.0
			m.color.r = 1.0
			m.color.g = 1.0
			m.color.b = 0.0
			self.marker_array.markers.append(m)
		# variables
		self.min_x = 0.5
		self.min_y = -4.0
		self.grid_width = 8.0
		self.max_width = 1.5
		self.min_width = 0.2
		self.sensor_z = 1.0
		self.resolution = 0.05
		self.min_z = self.sensor_z
		self.max_z = 2.0
		self.cluster_eps = 3
		self.cluster_min_samples = 5
		rospy.loginfo('CloudListener: initialized')

	def on_topic_received(self, data):
		rospy.loginfo(rospy.get_caller_id() + " " + self.topic_name + " received")
		points = pointcloud2_to_xyz_array(data, dtype=np.float32)
		clusters = self.cluster_points(points)
		self.publish_clusters(clusters)
		rospy.loginfo(rospy.get_caller_id() + "marker published")

	def publish_clusters(self, clusters):
		mcnt = 0
		ccnt = 0
		# mark clusters
		while ccnt < len(clusters):
			cl = clusters[ccnt]
			pos = np.mean((cl.min, cl.max), axis=0)
			size = cl.max - cl.min
			ccnt += 1
			if np.max(size[:2]) > self.max_width or np.max(size[:2]) < self.min_width:
				continue
			rospy.loginfo(rospy.get_caller_id()
										+ str(pos[0]) + ' ,' \
										+ str(pos[1]) + ' ,' \
										+ str(pos[2]) + ' ,' \
										+ str(size[0]) + ' ,' \
										+ str(size[1]) + ' ,' \
										+ str(size[2]) + ' ,' \
										)
			m = self.marker_array.markers[mcnt]
			# don't publish when the size is not within threshold
			m.pose.position.x = pos[0] + self.min_x
			m.pose.position.y = pos[1] + self.min_y
			m.pose.position.z = pos[2] - self.sensor_z
			m.scale.x = size[0]
			m.scale.y = size[1]
			m.scale.z = 0# size[2] + self.sensor_z
			m.color.a = 0.5
			mcnt += 1
		# hide markers
		while mcnt < len(self.marker_array.markers):
			m = self.marker_array.markers[mcnt]
			m.color.a = 0.0	
			mcnt += 1
		# publish markers
		self.publisher.publish(self.marker_array)

	def cluster_points(self, points):
		# points min subtraction
		points[:,0] -= self.min_x
		points[:,1] -= self.min_y
		# points calibration
		points[:,2] += self.sensor_z
		# filter out point out of xy range
		n_col = int(self.grid_width / self.resolution)
		xy_idx = (points[:,:2]/self.resolution).astype(int)
		xy_idx_x = xy_idx[:,0]
		xy_idx_y = xy_idx[:,1]
		valid_x_bv = np.logical_and(xy_idx_x > 0, xy_idx_x < n_col)
		valid_y_bv = np.logical_and(xy_idx_y > 0, xy_idx_y < n_col)
		valid_idx_bv = np.logical_and(valid_x_bv, valid_y_bv)
		points = points[valid_idx_bv]
		xy_idx_x = xy_idx_x[valid_idx_bv]
		xy_idx_y = xy_idx_y[valid_idx_bv]
		# elevmap generation
		elevmap = np.zeros((n_col,n_col))
		z = points[:,2]
		zsort = z.argsort()
		xid_zsort = xy_idx_x[zsort]
		yid_zsort = xy_idx_y[zsort]
		z_zsort = z[zsort]
		elevmap[xid_zsort[:],yid_zsort[:]] = z_zsort
		# filter out grid out of z range
		z_filter_bv = np.logical_or(elevmap[:,:] < self.min_z, elevmap[:,:] > self.max_z)
		elevmap[z_filter_bv] = 0
		# convert to points
		grid_points = np.asarray(np.where(elevmap>0)).T
		# cluster points
		clusters = []
		if len(grid_points) > self.cluster_min_samples:
			mod = DBSCAN(eps=self.cluster_eps, min_samples=self.cluster_min_samples)
			mod.fit(grid_points)
			clabels = np.asarray(mod.labels_)
			clabels_unique = np.unique(clabels)
			clabels_unique = clabels_unique[clabels_unique>-1]
			n_cluster = len(clabels_unique)
			clustered_points = grid_points[clabels>-1]
			clabels = clabels[clabels>-1]
			# make clusters
			for ci in range(n_cluster):
				cl = Cluster(ci, self.resolution)
				cl_xy_id = clustered_points[clabels==ci]
				cl_z = elevmap[cl_xy_id[:,0],cl_xy_id[:,1]]
				cl.add_points(cl_xy_id, cl_z)
				clusters.append(cl)
		return clusters
	
if __name__ == '__main__':
	if len(sys.argv) != 4:
		rospy.loginfo("CloudListener: error: insert a topic names as input arguments")
	else:
		listener = CloudListener(sys.argv[1])
		rospy.init_node('cloud_listener', anonymous=True)
		rospy.spin()
		rospy.loginfo('CloudListener: finished spinning')
