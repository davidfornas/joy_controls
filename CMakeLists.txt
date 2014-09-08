cmake_minimum_required(VERSION 2.8.3)
project(joy_controls)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs mar_robot_arm5e nav_msgs)
find_package(VISP REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    mar_robot_arm5e
    nav_msgs
  DEPENDS
    VISP
)
add_definitions(${VISP_DEFINITIONS})
link_directories(${VISP_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${VISP_INCLUDE_DIRS})

add_executable(hybrid_teleop src/hybrid_teleop.cpp)
target_link_libraries(hybrid_teleop ${catkin_LIBRARIES})

add_executable(cartesian_teleop src/arm5_joy_cartesian_control.cpp)
target_link_libraries(cartesian_teleop ${catkin_LIBRARIES})

install(
  TARGETS
    hybrid_teleop
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
