# CMakeLists.txt

cmake_minimum_required(VERSION 3.8)
project(rosbot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Add include directories
include_directories(
  include
)

# Add the lidar_node executable
add_executable(lidar_node src/lidar.cpp)

# Link dependencies
ament_target_dependencies(lidar_node rclcpp)

# Install target (copy the executable to the install directory)
install(TARGETS lidar_node DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch urdf meshes config
  DESTINATION share/${PROJECT_NAME}/
)

# Enable testing if needed
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
