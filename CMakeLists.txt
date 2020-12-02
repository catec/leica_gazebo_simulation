cmake_minimum_required(VERSION 2.8.3)
project(leica_gazebo_simulation)

set(VTK_LEGACY_SILENT ON)
add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  roscpp
  rospy
  std_msgs
  tf
  sensor_msgs
  laser_geometry
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(laserscan_to_pointcloud 
  src/laserscan_to_pointcloud.cpp
)

target_link_libraries(laserscan_to_pointcloud
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# add_executable(plugin 
#   plugin/plugin.cpp
# )

# target_link_libraries(plugin
#   ${catkin_LIBRARIES}
# )

# Build the plugin
# add_library(leica_c5_sensor_plugin SHARED plugin/leica_c5_sensor_plugin.cc)
# target_link_libraries(leica_c5_sensor_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

add_library(leica_plugin plugin/leica_plugin.cpp)
target_link_libraries(leica_plugin ${catkin_LIBRARIES} GpuRayPlugin)