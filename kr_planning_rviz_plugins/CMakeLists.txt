# cmake_minimum_required(VERSION 3.10)
# project(kr_planning_rviz_plugins)

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find_package(
#   catkin REQUIRED
#   COMPONENTS rviz
#              roscpp
#              pcl_ros
#             #  eigen_conversions
#              kr_planning_msgs
#              tf_conversions)

# find_package(Eigen3 REQUIRED)

# add_definitions(-DQT_NO_KEYWORDS)

# set(CMAKE_AUTOMOC ON)
# # This setting causes Qt's "MOC" generation to happen automatically. this does
# # not moc things in include!!!!!!! only in src

# message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
# find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
# set(QT_LIBRARIES Qt5::Widgets)

# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES ${PROJECT_NAME} kr_planning_utils
#   CATKIN_DEPENDS kr_planning_msgs
#   DEPENDS EIGEN3)

# add_library(kr_planning_utils src/utils/data_ros_utils.cpp src/utils/map_util.cpp)
# target_include_directories(kr_planning_utils PUBLIC ${catkin_INCLUDE_DIRS}
#                                                   include)
# target_link_libraries(kr_planning_utils PUBLIC ${catkin_LIBRARIES} Eigen3::Eigen)

# set(SOURCE_FILES
#     src/bound_visual.cpp
#     src/mesh_visual.cpp
#     src/path_visual.cpp
#     src/path_display.cpp
#     src/spline_trajectory_display.cpp
#     src/spline_trajectory_visual.cpp
#     src/path_array_display.cpp
#     src/map_display.cpp
#     src/primitive_visual.cpp
#     src/primitive_array_display.cpp
#     src/trajectory_visual.cpp
#     src/trajectory_display.cpp
#     )

# # Something to do with QT that these must be used as source files instead of includes?
# set(HEADER_FILES
#     include/${PROJECT_NAME}/bound_visual.h
#     include/${PROJECT_NAME}/mesh_visual.h
#     include/${PROJECT_NAME}/path_visual.h
#     include/${PROJECT_NAME}/path_display.h
#     include/${PROJECT_NAME}/spline_trajectory_display.h
#     include/${PROJECT_NAME}/spline_trajectory_visual.h
#     include/${PROJECT_NAME}/path_array_display.h
#     include/${PROJECT_NAME}/map_display.h
#     include/${PROJECT_NAME}/primitive_visual.h
#     include/${PROJECT_NAME}/primitive_array_display.h
#     include/${PROJECT_NAME}/trajectory_visual.h
#     include/${PROJECT_NAME}/trajectory_display.h
# )
# add_library(${PROJECT_NAME} ${HEADER_FILES} ${SOURCE_FILES} )

# target_include_directories(${PROJECT_NAME} PUBLIC include
#                                                         ${catkin_INCLUDE_DIRS})
# target_link_libraries(
#   ${PROJECT_NAME}
#   PUBLIC kr_planning_utils ${QT_LIBRARIES} ${catkin_LIBRARIES}
#          ${rviz_DEFAULT_PLUGIN_LIBRARIES})

# -----------------------------------------------------------         
# ROS2 version of CMakeLists.txt

cmake_minimum_required(VERSION 3.10)
project(kr_planning_rviz_plugins)

# Enforce C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ament and core ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(kr_planning_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED)
# find_package(OGRE REQUIRED)
# add_compile_definitions(OGRE_NO_MEMORY_ALLOCATOR)
include_directories(
  # Add ROS OGRE paths FIRST
  /opt/ros/jazzy/opt/rviz_ogre_vendor/include
  /opt/ros/jazyy/opt/rviz_ogre_vendor/include/OGRE
  # Add system path first for OGRE
  # /usr/include/OGRE
)
# Qt5
add_definitions(-DQT_NO_KEYWORDS)
set(CMAKE_AUTOMOC ON)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)

# ------------------------------------------------------------------
# kr_planning_utils library (shared, pure C++)
# ------------------------------------------------------------------
add_library(kr_planning_utils
  src/utils/data_ros_utils.cpp
  src/utils/map_util.cpp
)
ament_target_dependencies(kr_planning_utils
  rclcpp
  pcl_ros
  kr_planning_msgs
  Eigen3
  tf2_eigen
  geometry_msgs
  sensor_msgs
)
target_include_directories(kr_planning_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ------------------------------------------------------------------
# Main RViz plugin library
# ------------------------------------------------------------------
set(SOURCE_FILES
  src/bound_visual.cpp
  src/mesh_visual.cpp
  src/path_visual.cpp
  src/path_display.cpp
  src/spline_trajectory_display.cpp
  src/spline_trajectory_visual.cpp
  src/path_array_display.cpp
  src/map_display.cpp
  src/primitive_visual.cpp
  src/primitive_array_display.cpp
  src/trajectory_visual.cpp
  src/trajectory_display.cpp
)
set(HEADER_FILES
  include/${PROJECT_NAME}/bound_visual.h
  include/${PROJECT_NAME}/mesh_visual.h
  include/${PROJECT_NAME}/path_visual.h
  include/${PROJECT_NAME}/path_display.h
  include/${PROJECT_NAME}/spline_trajectory_display.h
  include/${PROJECT_NAME}/spline_trajectory_visual.h
  include/${PROJECT_NAME}/path_array_display.h
  include/${PROJECT_NAME}/map_display.h
  include/${PROJECT_NAME}/primitive_visual.h
  include/${PROJECT_NAME}/primitive_array_display.h
  include/${PROJECT_NAME}/trajectory_visual.h
  include/${PROJECT_NAME}/trajectory_display.h
)

add_library(${PROJECT_NAME}
  ${SOURCE_FILES}
  ${HEADER_FILES}
)
# Qt macro definitions
target_compile_definitions(${PROJECT_NAME} PRIVATE "QT_NO_KEYWORDS")
# Link against utilities, Qt and RViz2 defaults
target_link_libraries(${PROJECT_NAME}
  kr_planning_utils
  ${QT_LIBRARIES}
)
# ROS2/ament dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  rviz_default_plugins
  rviz_rendering
  pcl_ros
  kr_planning_msgs
  Eigen3
  tf2_eigen
  geometry_msgs
  sensor_msgs
  Boost
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ------------------------------------------------------------------
# Installation
# ------------------------------------------------------------------
# Libraries
install(
  TARGETS
    kr_planning_utils
    ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# Headers
install(
  DIRECTORY include/
  DESTINATION include/
)
# RViz2 plugin description (plugin_description.xml under the package root)
install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies and package
ament_export_dependencies(
  rclcpp
  rviz_common
  rviz_default_plugins
  rviz_rendering
  pcl_ros
  kr_planning_msgs
  Eigen3
  tf2_eigen
  geometry_msgs
  sensor_msgs
  Boost
)
ament_export_include_directories(include)
ament_export_libraries(kr_planning_utils ${PROJECT_NAME})

ament_package()