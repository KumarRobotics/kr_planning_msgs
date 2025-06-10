# kr_planning_msgs

## Change Log

- Created `README.md` for the package.
- Updated the msgs in kr_planning_msgs package to ROS2 format
- Updated the rviz_plugin implementation to be ROS2 built compatiable
- ```update_nh_.setCallbackQueue(context_->getThreadedQueue());``` is deleted from map_display.cpp as ROS2 does not support such functions anymore. ROS2 replacement code is not implemented yet in the file to be equivalent this line of code. Simply deleted it to be able to compile. 

## Compatibility

This package is built compilable with ROS2 jazzy and ubuntu 24.04, but it has **not been tested yet**. It's mainly updated to support the compilation of ros2 version of kr_autonomous_flight. 