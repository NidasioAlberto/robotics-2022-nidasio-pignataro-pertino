cmake_minimum_required(VERSION 3.0.2)
project(bag_printers)

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
add_executable(bag1_basic_printer src/bag1_basic_printer.cpp)
target_link_libraries(bag1_basic_printer ${catkin_LIBRARIES})
