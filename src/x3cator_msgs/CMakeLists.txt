cmake_minimum_required(VERSION 3.5)
project(x3cator_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Enable compiler warnings for GCC or Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
##########################################
  "msg/FourGasSensor.msg" 
  "srv/FourGasSensor.srv" 

  "msg/WheelVelocity.msg" 
  "srv/WheelVelocity.srv" 
##########################################
  DEPENDENCIES std_msgs
)

# If building tests, enable linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip linter checking for copyrights if not needed
  # set(ament_cmake_copyright_FOUND TRUE)
  # Skip cpplint (only works in a git repo)
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Declare the package
ament_package()
