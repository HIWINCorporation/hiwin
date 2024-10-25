cmake_minimum_required(VERSION 3.0.2)
project(hiwin_robot_client_library)

## Find catkin and any catkin packages
find_package(catkin REQUIRED
COMPONENTS
  roscpp
  std_msgs
)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS
    include
#  LIBRARIES hiwin_robot_client_library
  CATKIN_DEPENDS
    roscpp
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/socket/tcp_client.cpp
  src/commander.cpp
  src/hiwin_driver.cpp
)

## Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## Install targets (library and node executable) and header files
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
