cmake_minimum_required(VERSION 2.8.3)
project(leica_gazebo_simulation)

set(VTK_LEGACY_SILENT ON)
add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
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
  leica_scanstation_utils
  leica_scanstation_msgs
)

catkin_package(INCLUDE_DIRS include)

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

add_executable(leica_service_listener 
  src/leica_service_listener.cpp
)

target_link_libraries(leica_service_listener
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(leica_plugin plugin/leica_plugin.cpp)
target_link_libraries(leica_plugin ${catkin_LIBRARIES} GpuRayPlugin)


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(test_laserscan_to_pointcloud test/test_laserscan_to_pointcloud.test test/test_laserscan_to_pointcloud.cpp)
  target_link_libraries(test_laserscan_to_pointcloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})

endif()