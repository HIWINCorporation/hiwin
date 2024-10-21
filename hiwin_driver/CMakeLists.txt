cmake_minimum_required(VERSION 3.0.2)
project(hiwin_driver)


find_package(catkin REQUIRED 
  COMPONENTS
    actionlib
    controller_manager
    hardware_interface
    industrial_robot_status_interface
    pluginlib
    roscpp
)

catkin_package(
  INCLUDE_DIRS 
    include
#  LIBRARIES hiwin_driver
  CATKIN_DEPENDS
    actionlib
    controller_manager
    hardware_interface
    pluginlib
    roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(hiwin_driver_plugin
  src/hardware_interface.cpp
)
target_link_libraries(hiwin_driver_plugin ${catkin_LIBRARIES})
add_dependencies(hiwin_driver_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(hiwin_driver_node
  src/hardware_interface.cpp
  src/hardware_interface_node.cpp
)
target_link_libraries(hiwin_driver_node ${catkin_LIBRARIES})
add_dependencies(hiwin_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS hiwin_driver_plugin hiwin_driver_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)