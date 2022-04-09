cmake_minimum_required(VERSION 3.0.2)
project(bags_printer)

###########################
## Packages dependencies ##
###########################
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

###################
## Configuration ##
###################
catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})

#################
## Executables ##
#################
add_executable(${PROJECT_NAME}_node src/bags_printer_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
