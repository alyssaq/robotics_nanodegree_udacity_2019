cmake_minimum_required(VERSION 2.8.3)
project(pick_objects)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS actionlib move_base_msgs roscpp)
catkin_package()

###########
## Build ##
###########
include_directories(${catkin_INCLUDE_DIRS})
add_executable(pick_objects src/navigation_goal.cpp)
target_link_libraries(pick_objects ${catkin_LIBRARIES})

## Mark executables and/or libraries for installation
install(TARGETS pick_objects
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
