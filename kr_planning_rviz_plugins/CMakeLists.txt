cmake_minimum_required(VERSION 3.10)
project(kr_planning_rviz_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(
  catkin REQUIRED
  COMPONENTS rviz
             roscpp
             pcl_ros
             eigen_conversions
             kr_planning_msgs
             tf_conversions)

find_package(Eigen3 REQUIRED)

add_definitions(-DQT_NO_KEYWORDS)

set(CMAKE_AUTOMOC ON)
# This setting causes Qt's "MOC" generation to happen automatically. this does
# not moc things in include!!!!!!! only in src

message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} kr_planning_utils
  CATKIN_DEPENDS kr_planning_msgs)

add_library(kr_planning_utils src/utils/data_ros_utils.cpp src/utils/map_util.cpp)
target_include_directories(kr_planning_utils PUBLIC ${catkin_INCLUDE_DIRS}
                                                  include)
target_link_libraries(kr_planning_utils PUBLIC ${catkin_LIBRARIES} Eigen3::Eigen)

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

# Something to do with QT that these must be used as source files instead of includes?
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
add_library(${PROJECT_NAME} ${HEADER_FILES} ${SOURCE_FILES} )

target_include_directories(${PROJECT_NAME} PUBLIC include
                                                        ${catkin_INCLUDE_DIRS})
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC kr_planning_utils ${QT_LIBRARIES} ${catkin_LIBRARIES}
         ${rviz_DEFAULT_PLUGIN_LIBRARIES})
