cmake_minimum_required(VERSION 3.0.2)
project(project_1)

###########################
## Packages dependencies ##
###########################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  dynamic_reconfigure
  tf2
  tf2_ros
  cmake_modules
)
generate_dynamic_reconfigure_options(
  cfg/wheelsDataSource.cfg
  cfg/integrationMethod.cfg
)
find_package(Eigen REQUIRED)

#####################
##     Services    ##
#####################
add_service_files(
  FILES
  ResetStartingPose.srv
)

#####################
## Custom messages ##
#####################

add_message_files(
  FILES
  WheelsRpm.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################
## Configuration ##
###################
catkin_package(
  DEPENDS Eigen
  CATKIN_DEPENDS message_runtime tf2 tf2_ros
)
include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${tf2_ros_INCLUDE_DIRS})

#################
## Executables ##
#################
add_executable(odometry_computer src/odometry_computer.cpp)
add_dependencies(odometry_computer ${catkin_EXPORTED_TARGETS})
target_link_libraries(odometry_computer ${catkin_LIBRARIES})

add_executable(velocity_computer src/velocity_computer.cpp)
add_dependencies(velocity_computer ${catkin_EXPORTED_TARGETS})
target_link_libraries(velocity_computer ${catkin_LIBRARIES})

add_executable(wheels_velocity_computer src/wheels_velocity_computer.cpp)
add_dependencies(wheels_velocity_computer ${catkin_EXPORTED_TARGETS})
target_link_libraries(wheels_velocity_computer ${catkin_LIBRARIES})

add_executable(starting_position_detector src/starting_position_detector.cpp)
add_dependencies(starting_position_detector ${catkin_EXPORTED_TARGETS})
target_link_libraries(starting_position_detector ${catkin_LIBRARIES})