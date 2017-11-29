## ROS-multi-velodyne

This is the ROS package containing example code of
* launch files converting multiple Velodyne sensor inputs to multiple PointCloud2 topics
* Python script and a launch file of a ROS node subscribing multiple PointCloud2 topics and publishing modified PointCloud2 topics

Written by Jaeil Park, 2017

### Key files

#### [launch/cloud_nodelet.launch](launch/cloud_nodelet.launch)
File to launch nodelets to receive velodyne sensor inputs (velodyne_packets) and convert to point cloud messages. You can modify driver options and conversion options with arguments. For example, you can change the topic name with 'topic_name' argument.

#### [launch/cloud_listener.launch](launch/cloud_listener.launch)
File to launch 'cloud_listner.py' node. See 'script/cloud_listener.py'.

#### [script/cloud_listener.py](script/cloud_listener.py)
A ROS node subscribing two PointCloud2 topics emitted by cloud_nodelet.launch. When a PointCloud2 topic is received, the node translates points in the topic by 'topicN_shift'. The node publishes translated points with a prefix 'translated' which makes the name of a new topic 'translated/original_topic_name'. Frame id of the new topic is 'velodyne', same as the original topic's. 

#### [test_script/run_test.launch](test_script/run_test.launch)
A launch file launching two 'cloud_nodelet.launch' files to receive messages from two Velodyne sensors and 'cloud_listener.launch' file to manipulate point clouds. 
* Important: You have to use a different port number for each Velodyne sensor.
