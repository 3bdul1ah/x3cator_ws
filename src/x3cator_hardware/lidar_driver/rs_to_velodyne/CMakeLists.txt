# Directory structure
rs_to_velodyne/
├── CMakeLists.txt
├── package.xml
└── src/
    └── rs_to_velodyne.cpp

# CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(rs_to_velodyne)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

# Include PCL directories
include_directories(
  ${PCL_INCLUDE_DIRS}
)

# Create executable
add_executable(rs_to_velodyne src/rs_to_velodyne.cpp)
target_link_libraries(rs_to_velodyne
  ${PCL_LIBRARIES}
)
ament_target_dependencies(rs_to_velodyne
  rclcpp
  sensor_msgs
  pcl_conversions
)

# Install
install(TARGETS
  rs_to_velodyne
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()