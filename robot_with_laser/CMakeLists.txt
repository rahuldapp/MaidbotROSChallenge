cmake_minimum_required(VERSION 2.8.3)
project(robot_with_laser)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  geometry_msgs
)

catkin_package()

include_directories(
${catkin_INCLUDE_DIRS}
include/robot_with_laser)

add_executable(run_robot src/run_robot.cpp)
add_dependencies(run_robot  ${catkin_EXPORTED_TARGETS})
target_link_libraries(run_robot
  ${catkin_LIBRARIES}
)

add_executable(run_laser 
src/run_laser.cpp
src/laser_base.cpp
src/laser_one.cpp
src/laser_two.cpp)
add_dependencies(run_laser  ${catkin_EXPORTED_TARGETS})
target_link_libraries(run_laser
  ${catkin_LIBRARIES}
)