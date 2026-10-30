cmake_minimum_required(VERSION 3.12)
project(unitree_lidar_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(UNILIDAR_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../unitree_lidar_sdk)

include_directories(
  ${PCL_INCLUDE_DIRS}
  include
  ${UNILIDAR_SDK_DIR}/include
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(unitree_lidar_ros2_node src/unitree_lidar_ros2_node.cpp)

target_link_libraries(unitree_lidar_ros2_node
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_srvs_TARGETS}
  tf2_ros::tf2_ros
  pcl_conversions::pcl_conversions
  ${PCL_LIBRARIES}
  ${UNILIDAR_SDK_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}/libunilidar_sdk2.a
)

install(TARGETS
unitree_lidar_ros2_node
  DESTINATION lib/${PROJECT_NAME}
)

# install rviz file and launch file
install(FILES rviz/view.rviz launch/launch.py
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
