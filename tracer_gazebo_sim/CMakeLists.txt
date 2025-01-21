cmake_minimum_required(VERSION 2.8.3)
project(tracer_gazebo_sim)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS    
    roslaunch 
    roslint 
    roscpp 
    std_msgs
    sensor_msgs
    geometry_msgs
    tf2 
    tf2_ros
    nav_msgs
    tf
)
catkin_package(
  INCLUDE_DIRS include
#   LIBRARIES 
  CATKIN_DEPENDS roscpp sensor_msgs
#   DEPENDS Boost
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(tracer_gazebo STATIC src/tracer_skid_steer.cpp)
target_link_libraries(tracer_gazebo ${catkin_LIBRARIES})

add_executable(tracer_skid_steer_controller src/tracer_skid_steer_controller.cpp)
target_link_libraries(tracer_skid_steer_controller tracer_gazebo ${catkin_LIBRARIES})


# Add executables
add_executable(odometry_publisher src/odometry_publisher.cpp)


# Link executables with ROS libraries
target_link_libraries(odometry_publisher ${catkin_LIBRARIES})


# Optionally, add dependencies
add_dependencies(odometry_publisher ${catkin_EXPORTED_TARGETS})





catkin_install_python(PROGRAMS src/lidar_merge_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Install ##
#############

roslaunch_add_file_check(launch)

install(
  DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
