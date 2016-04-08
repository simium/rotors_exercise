cmake_minimum_required(VERSION 2.8.3)
project(rotors_exercise)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  cmake_modules
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES 
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs
  DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(estimator_node src/estimator.cpp)
add_dependencies(estimator_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(estimator_node ${catkin_LIBRARIES})



