# ROS READ CAN DATA FORM KIA SOUL EV

### Intro

This is simple node for get steering wheel angle data and speed data from KIA SOUL EV's CAN data.

### Environment

* ROS Kinetic

### Dependency

* kvaser_interface(https://github.com/astuff/kvaser_interface)
* can_msgs(in ros_canopen https://github.com/ros-industrial/ros_canopen)

### How To Use

Use below command.

`rosrun ros_read_candata ros_read_candata_node`

or go to launch directory and use below command

`roslaunch read_can_data.launch`



## The `ros_read_candata` Node

#### TOPICS

*/TwistStamped*

This topic is published by the node. It contains 2d linear vector. You can get yaw angle and velocity using this 2d linear vector.



*/can_tx* [can_msgs::Frame]

This topic is subscribed to by the node. It expects to read CAN data from KIA SOUL EV.