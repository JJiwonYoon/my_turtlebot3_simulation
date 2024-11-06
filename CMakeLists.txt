cmake_minimum_required(VERSION 3.5)
project(my_turtlebot3_simulation)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Define executables with unique target names
add_executable(bot_node src/bot.cpp)
ament_target_dependencies(bot_node rclcpp sensor_msgs nav_msgs std_msgs geometry_msgs)

add_executable(lidar_detection_node src/lidar_detection.cpp)
ament_target_dependencies(lidar_detection_node rclcpp sensor_msgs nav_msgs std_msgs geometry_msgs)

# Install targets
install(TARGETS
  bot_node
  lidar_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

# Add testing dependencies if testing is enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Package setup
ament_package()
