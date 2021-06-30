cmake_minimum_required(VERSION 3.5)
project(ros2-hoverboard-driver)

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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# Add custom msgs
find_package(wheel_msgs REQUIRED)

## TODO : UNDERTSAND BETTER WHAT THESE FUNCTIONS DO
# Add /include directory so ROS2 is aware of it
include_directories(include)
# Add whatever source code we need to compile as a library (its headers can be in /include)
add_library(hoverboard SHARED
            src/hoverboard.cpp)
# Add other dependencies needed by the source code declared above
ament_target_dependencies(hoverboard rclcpp std_msgs wheel_msgs)

# Add the code we want to compile to have an executable (so ros2 run can execute it)
add_executable(main src/main.cpp)
# Declare we need the precompiled library we defined above
target_link_libraries(main hoverboard)
# Other dependencies (outsource)
ament_target_dependencies(main rclcpp std_msgs wheel_msgs)


# Install C files for ros2 run.
install(TARGETS
  hoverboard
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install C files for ros2 run.
install(TARGETS
  main
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
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
