cmake_minimum_required(VERSION 3.0.2)
project(project_2)

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
# generate_dynamic_reconfigure_options(
#   cfg/odometryComputerParams.cfg
#   cfg/velocityComputerParams.cfg
#   cfg/wheelsVelocityComputerParams.cfg
# )
find_package(Eigen REQUIRED)

#####################
##     Services    ##
#####################
# add_service_files(
#   FILES
#   ResetStartingPose.srv
# )

#####################
## Custom messages ##
#####################

# add_message_files(
#   FILES
#   WheelsRpm.msg
# )
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
add_executable(custom_odometry_computer src/custom_odometry_computer.cpp)
add_dependencies(custom_odometry_computer ${catkin_EXPORTED_TARGETS})
target_link_libraries(custom_odometry_computer ${catkin_LIBRARIES})

add_executable(custom_velocity_computer src/custom_velocity_computer.cpp)
add_dependencies(custom_velocity_computer ${catkin_EXPORTED_TARGETS})
target_link_libraries(custom_velocity_computer ${catkin_LIBRARIES})

add_executable(tf_tree_computer src/tf_tree_computer.cpp)
add_dependencies(tf_tree_computer ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf_tree_computer ${catkin_LIBRARIES})

# add_executable(position_comparison src/position_comparison.cpp)
# add_dependencies(position_comparison ${catkin_EXPORTED_TARGETS})
# target_link_libraries(position_comparison ${catkin_LIBRARIES})

# add_executable(velocity_computer src/velocity_computer.cpp)
# add_dependencies(velocity_computer ${catkin_EXPORTED_TARGETS})
# target_link_libraries(velocity_computer ${catkin_LIBRARIES})

# add_executable(wheels_velocity_computer src/wheels_velocity_computer.cpp)
# add_dependencies(wheels_velocity_computer ${catkin_EXPORTED_TARGETS})
# target_link_libraries(wheels_velocity_computer ${catkin_LIBRARIES})

# add_executable(starting_position_detector src/starting_position_detector.cpp)
# add_dependencies(starting_position_detector ${catkin_EXPORTED_TARGETS})
# target_link_libraries(starting_position_detector ${catkin_LIBRARIES})
