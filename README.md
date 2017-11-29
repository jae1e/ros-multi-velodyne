## ROS-Multi-velodyne

* Convert multiple velodyne sensor inputs to multiple PointCloud2 messages
* Interface to sync multiple point clouds

### Important files

#### launch/cloud_nodelet.launch
File to launch a nodelet to convert velodyne sensor input (velodyne_packets) to velodyne point cloud message. You can change the topic name with 'topic_name' argument, and set the property of the sensor device. For example usage, see 'test_script/run_test.launch'

#### launch/cloud_listener.launch
File to launch 'cloud_listner.py' node. See 'script/cloud_listner.py'.
