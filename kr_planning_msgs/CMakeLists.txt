# cmake_minimum_required(VERSION 3.10)
# project(kr_planning_msgs)

# find_package(catkin REQUIRED COMPONENTS geometry_msgs nav_msgs
#                                         message_generation)

# add_message_files(
#   DIRECTORY
#   msg)

# add_action_files(DIRECTORY action)
  
# generate_messages(DEPENDENCIES geometry_msgs nav_msgs actionlib_msgs)

# catkin_package(CATKIN_DEPENDS geometry_msgs nav_msgs actionlib_msgs message_runtime)  

cmake_minimum_required(VERSION 3.5)
project(kr_planning_msgs)

# 1) Use ament instead of catkin
find_package(ament_cmake REQUIRED)

# 2) Pull in the ROS 2 interface generators
find_package(rosidl_default_generators REQUIRED)

# 3) Declare dependencies on any packages your .msg/.action refer to
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# 4) Grab all of your .msg and .action files
file(GLOB MSG_FILES "msg/*.msg")
file(GLOB ACTION_FILES "action/*.action")

# 5) Generate ROS 2 interfaces from them
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${ACTION_FILES}
  DEPENDENCIES
    geometry_msgs
    nav_msgs
    action_msgs
)

# 6) Export that we need the generated runtime library
ament_export_dependencies(rosidl_default_runtime)

# 7) Install the interface definitions so downstream can find them
install(
  DIRECTORY msg action
  DESTINATION share/${PROJECT_NAME}
)

# 8) Standard ament package boilerplate
ament_package()
