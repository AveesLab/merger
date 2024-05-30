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

# Path to * project folder.
set(DARKNET_PATH ${PROJECT_SOURCE_DIR}/include/objectdetection/darknet)

# Give a custom name to shared library which is provided by * when the one compiled.
set(DARKNET_LIB_NAME libdarknet)

# Find required packages
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${DARKNET_PATH}/include)

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
add_executable(demo_node src/exec/demo_exec.cpp)
ament_target_dependencies(demo_node
  rclcpp
  std_msgs
  sensor_msgs
  vision_msgs
  cv_bridge
  Threads
)

add_library(demo_cam_library SHARED src/demo_camera.cpp)
ament_target_dependencies(demo_cam_library
  rclcpp
  std_msgs
  sensor_msgs
  vision_msgs
  cv_bridge
  Threads
)

add_library(demo_det_library SHARED 
   src/demo_detection.cpp
   include/objectdetection/objectdetection.cpp
   )
ament_target_dependencies(demo_det_library
  rclcpp
  std_msgs
  sensor_msgs
  vision_msgs
  cv_bridge
  Threads
)

add_library(${DARKNET_LIB_NAME} SHARED IMPORTED)

set_property(TARGET ${DARKNET_LIB_NAME} PROPERTY IMPORTED_LOCATION "${DARKNET_PATH}/libdarknet.so")

target_link_libraries(demo_node
  demo_cam_library
  demo_det_library
  ${DARKNET_LIB_NAME}
  pcanbasic
)

install(TARGETS
  demo_cam_library
  demo_det_library
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  demo_node
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
