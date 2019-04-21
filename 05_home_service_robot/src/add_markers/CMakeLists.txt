cmake_minimum_required(VERSION 2.8.3)
project(add_markers)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS visualization_msgs roscpp)
catkin_package()

###########
## Build ##
###########
include_directories(${catkin_INCLUDE_DIRS})
add_executable(add_markers src/add_markers.cpp)
target_link_libraries(add_markers ${catkin_LIBRARIES})

## Mark executables and/or libraries for installation
install(TARGETS add_markers
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
