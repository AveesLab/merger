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
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Threads REQUIRED)

include_directories(include)

# Build
add_executable(demo_node1 src/exec/demo_exec1.cpp)
ament_target_dependencies(demo_node1
  rclcpp
  std_msgs
  sensor_msgs
  vision_msgs
  cv_bridge
  Threads
)

add_library(demo_library1 SHARED src/demo1.cpp)
ament_target_dependencies(demo_library1
  rclcpp
  std_msgs
  sensor_msgs
  vision_msgs
  cv_bridge
  Threads
)

target_link_libraries(demo_node1
  demo_library1
  pcanbasic
)
add_executable(demo_node2 src/exec/demo_exec2.cpp)
ament_target_dependencies(demo_node2
  rclcpp
  std_msgs
  sensor_msgs
  vision_msgs
  cv_bridge
  Threads
)

add_library(demo_library2 SHARED src/demo2.cpp)
ament_target_dependencies(demo_library2
  rclcpp
  std_msgs
  sensor_msgs
  vision_msgs
  cv_bridge
  Threads
)

target_link_libraries(demo_node2
  demo_library2
  pcanbasic
)

add_executable(demo_node3 src/exec/demo_exec3.cpp)
ament_target_dependencies(demo_node3
  rclcpp
  std_msgs
  sensor_msgs
  vision_msgs
  cv_bridge
  Threads
)

add_library(demo_library3 SHARED src/demo3.cpp)
ament_target_dependencies(demo_library3
  rclcpp
  std_msgs
  sensor_msgs
  vision_msgs
  cv_bridge
  Threads
)

target_link_libraries(demo_node3
  demo_library3
  pcanbasic
)
install(TARGETS
  demo_library1
  demo_library2
  demo_library3
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  demo_node1
  demo_node2
  demo_node3
  DESTINATION lib/${PROJECT_NAME}
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
