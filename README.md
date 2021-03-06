## Normal Distribution Transform Matching in ROS2

Using <i>ardent</i> as ros2 distribution and <i>kinetic</i> as ros1 distribution on Ubuntu 16.04.Substitute with your respective distributions.
Download and extract ndt_matching.zip inside the <i>workspace/src/</i> folder
Here the workspace folder is <i>~/ndt_match_ws </i>

To build the package :
```shell
$ cd ~/ndt_match_ws
$ source /opt/ros/ardent/setup.bash 
$ ament build --only-packages ndt_matching
```

To start roscore in terminal 1:
```shell
$ source /opt/ros/kinetic/setup.bash
$ roscore
```

To start the ros2 bridge use terminal 1 : 
```shell
$ source /opt/ros/ardent/setup.bash 
$ ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

To publish map in ros1 using terminal 2:
```shell
$ source /opt/ros/kinetic/setup.bash
$ rosrun pcl_ros pcd_to_pointcloud map.pcd cloud_pcd:=/map
```
To play the bag in ros1 use terminal 3 :
```shell
$ source /opt/ros/kinetic/setup.bash
$ rosbag play lidar_data.bag
```

To run the ndt_node in ros2 use terminal 4:
```shell
$ source /opt/ros/ardent/setup.bash 
$ source ~/ndt_match_ws/install/local_setup.bash
$ ~/ndt_match_ws/install/lib/ndt_matching/ndt_node
```

To view the estimated pose use terminal 5:
```shell
$ source /opt/ros/ardent/setup.bash 
$ ros2 topic echo /estimated_pose
```


#### Topics published

```shell
Terminal 2 : /map

Terminal 3: /filtered_points , /current_pose and others which are not subscribed to.

Terminal 4 : /estimated_pose
```

### Instructions to run the tests

#### Unit tests

<i>CMakeLists.txt</i> exists inside <i>~/ndt_match_ws/src/ndt_matching</i>.
On line 82 of <i>CMakeLists.txt</i> replace the path of <i>map.pcd</i> in the <i>PCD</i> variable as:
```shell
ament_add_gtest(ndt_test test/ndt_test.cpp ENV PCD=<path_to_map.pcd>)
```

Unit Test will run only if this file is found on the system.

To run unit tests in terminal :
```shell
$ source /opt/ros/ardent/setup.bash 
$ source ~/ndt_match_ws/install/setup.bash
$ ament test --only-packages ndt_matching --ctest-args -R ndt_test
```

#### Integration Test

To run the ndt_node in ros2 use terminal 1:
```shell
$ source /opt/ros/ardent/setup.bash 
$ source ~/ndt_match_ws/install/setup.bash
$ ~/ndt_match_ws/install/lib/ndt_matching/ndt_node
```

To run integration tests in terminal 2 :
```shell
$ source /opt/ros/ardent/setup.bash 
$ source ~/ndt_match_ws/install/setup.bash
$ ament test --only-packages ndt_matching --ctest-args -R integration_test
```
