cmake_minimum_required(VERSION 3.0.2)
project(forwarding_controllers)


find_package(catkin REQUIRED 
COMPONENTS
  actionlib
  control_msgs
  controller_interface
  hardware_interface
  industrial_msgs
  pluginlib
  roscpp
)

catkin_package(
  INCLUDE_DIRS
    include
#  LIBRARIES forwarding_controllers
  CATKIN_DEPENDS
    actionlib
    control_msgs
    controller_interface
    hardware_interface
    industrial_msgs
    pluginlib
    roscpp
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
  src/forwarding_controllers.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES
  ${PROJECT_NAME}_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
