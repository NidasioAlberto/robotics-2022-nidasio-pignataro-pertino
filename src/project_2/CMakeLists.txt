cmake_minimum_required(VERSION 3.0.2)
project(project_2)

###########################
## Packages dependencies ##
###########################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  message_generation
  dynamic_reconfigure
  tf2
  tf2_ros
  cmake_modules
  cv_bridge
  OpenCV
)

find_package(Eigen REQUIRED)

find_package(OpenCV REQUIRED)

#####################
##     Services    ##
#####################
add_service_files(
  FILES
  SaveRobotTrajectory.srv
)

#####################
##     Others      ##
#####################

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################
## Configuration ##
###################
catkin_package(
  DEPENDS Eigen
  CATKIN_DEPENDS message_runtime tf2 tf2_ros cv_bridge
)
include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${tf2_ros_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

#################
## Executables ##
#################
add_executable(custom_odometry_computer src/custom_odometry_computer.cpp)
add_dependencies(custom_odometry_computer ${catkin_EXPORTED_TARGETS})
target_link_libraries(custom_odometry_computer ${catkin_LIBRARIES})

add_executable(custom_velocity_computer src/custom_velocity_computer.cpp)
add_dependencies(custom_velocity_computer ${catkin_EXPORTED_TARGETS})
target_link_libraries(custom_velocity_computer ${catkin_LIBRARIES})

add_executable(tf_tree_computer src/tf_tree_computer.cpp)
add_dependencies(tf_tree_computer ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf_tree_computer ${catkin_LIBRARIES})

add_executable(trajectory_printer src/trajectory_printer.cpp)
add_dependencies(trajectory_printer ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_printer ${catkin_LIBRARIES})
