# ROS READ CAN DATA FORM KIA SOUL EV

### Intro

This is simple node for get steering wheel angle data and speed data from each 4 wheels.

### Environment

* ROS Kinetic

### Dependency

* kvaser_interface(you need kvaser CAN converter)
* can_msgs(in ros_canopen)



### How To Use
First you have to change this ros directory name as 'ros_read_candata'.
And then, use below command.
`rosrun ros_read_candata ros_read_candata_node`
