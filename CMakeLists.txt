cmake_minimum_required(VERSION 3.5)
project(monitor)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Threads REQUIRED)

include_directories(include)

# Build
add_executable(pub_demo src/exec/pub_exec.cpp)
ament_target_dependencies(pub_demo
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  Threads
)

add_executable(sub_demo src/exec/sub_exec.cpp)
ament_target_dependencies(sub_demo
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  Threads
)

add_library(demo_library SHARED src/pub.cpp src/sub.cpp)
ament_target_dependencies(demo_library
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  Threads
)

target_link_libraries(pub_demo
  demo_library
)

target_link_libraries(sub_demo
  demo_library
)

install(TARGETS
  demo_library
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  pub_demo
  sub_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
