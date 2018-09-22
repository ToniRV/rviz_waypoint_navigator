cmake_minimum_required(VERSION 2.8.3)
project(rviz_path_planner)

find_package(catkin REQUIRED COMPONENTS interactive_markers roscpp visualization_msgs tf
  glog_catkin waypoint_navigator std_srvs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS interactive_markers roscpp visualization_msgs tf glog_catkin
  waypoint_navigator std_srvs
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rviz_path_planner src/rviz_path_planner.cpp)
target_link_libraries(rviz_path_planner
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS
  rviz_path_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
