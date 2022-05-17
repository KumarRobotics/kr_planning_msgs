cmake_minimum_required(VERSION 3.10)
project(kr_planning_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs nav_msgs
                                        message_generation)

add_message_files(
  DIRECTORY
  msg)

generate_messages(DEPENDENCIES geometry_msgs nav_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs nav_msgs message_runtime)
