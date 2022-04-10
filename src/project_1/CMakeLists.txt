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
generate_dynamic_reconfigure_options(
  cfg/parameters.cfg
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
add_dependencies(velocity_computer ${catkin_EXPORTED_TARGETS})
target_link_libraries(velocity_computer ${catkin_LIBRARIES})

add_executable(odometry_computer src/odometry_computer.cpp)
add_dependencies(odometry_computer ${catkin_EXPORTED_TARGETS})
target_link_libraries(odometry_computer ${catkin_LIBRARIES})