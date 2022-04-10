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
  cmake_modules
)
find_package(Eigen REQUIRED)

###################
## Configuration ##
###################
catkin_package(
  DEPENDS Eigen
)
include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

#################
## Executables ##
#################
add_executable(velocity_computer src/velocity_computer.cpp)
target_link_libraries(velocity_computer ${catkin_LIBRARIES})
