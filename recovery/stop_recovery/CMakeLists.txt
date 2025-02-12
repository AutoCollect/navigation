cmake_minimum_required(VERSION 3.10)

# Suppress developer warnings (equivalent to using -Wno-dev on the command line)
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "Suppress developer warnings")

# Set policies to avoid deprecation warnings
cmake_policy(SET CMP0148 OLD)
cmake_policy(SET CMP0167 OLD)

project(stop_recovery)

find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    costmap_2d
    geometry_msgs
    nav_core
    pluginlib
    roscpp
)

find_package(Eigen3 REQUIRED)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)
# add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES stop_recovery
  CATKIN_DEPENDS
    geometry_msgs
    nav_core
    pluginlib
    roscpp
)

add_library(${PROJECT_NAME} src/stop_recovery.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS stop_recovery
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
       )

install(FILES stop_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

