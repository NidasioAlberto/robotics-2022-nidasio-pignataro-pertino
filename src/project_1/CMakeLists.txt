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
)

###################
## Configuration ##
###################
catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})