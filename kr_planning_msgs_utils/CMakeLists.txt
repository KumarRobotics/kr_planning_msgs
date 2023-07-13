cmake_minimum_required(VERSION 3.10)
project(kr_planning_msgs_utils)

find_package(catkin REQUIRED COMPONENTS 
              roscpp 
              kr_planning_msgs
              kr_planning_rviz_plugins
              sensor_msgs)
find_package(SDL REQUIRED)
find_package(SDL_image REQUIRED)

catkin_package(CATKIN_DEPENDS roscpp kr_planning_msgs sensor_msgs kr_planning_rviz_plugins)  

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(image_to_map src/image_to_map.cpp)
target_link_libraries(image_to_map
                      ${catkin_LIBRARIES}
                      ${SDL_LIBRARY}
                      ${SDL_IMAGE_LIBRARIES})
add_dependencies(image_to_map ${catkin_EXPORTED_TARGETS})
