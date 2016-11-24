cmake_minimum_required(VERSION 2.8.3)
project(joy_controls)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs geometry_msgs nav_msgs mar_robot_arm5e nav_msgs)
find_package(VISP REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    std_msgs
    geometry_msgs
    nav_msgs
    mar_robot_arm5e
    nav_msgs
  DEPENDS
    VISP
)
add_definitions(${VISP_DEFINITIONS})
link_directories(${VISP_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${VISP_INCLUDE_DIRS})

add_executable(hybrid_teleop src/hybrid_teleop.cpp)
add_executable(cartesian_teleop src/arm5_joy_cartesian_control.cpp)
add_executable(hybrid_teleop_with_dynamics src/hybrid_teleop_with_dynamics.cpp)
add_executable(hybrid_teleop_hobbit src/hybrid_teleop_hobbit.cpp)

target_link_libraries(hybrid_teleop ${catkin_LIBRARIES})
target_link_libraries(cartesian_teleop ${catkin_LIBRARIES})
target_link_libraries(hybrid_teleop_with_dynamics ${catkin_LIBRARIES})
target_link_libraries(hybrid_teleop_hobbit ${catkin_LIBRARIES})

install(
  TARGETS
    hybrid_teleop
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
