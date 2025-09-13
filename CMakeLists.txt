cmake_minimum_required(VERSION 3.8)
project(mavlink_gui_tester)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

# Set up Qt5
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Include directories
include_directories(include)
include_directories(/home/imanoob/ros2_jazzy/src/stm32_mavlink_interface/include/mavlink/c_library_v2)
include_directories(/home/imanoob/ros2_jazzy/src/stm32_mavlink_interface/include/mavlink/c_library_v2/common)

# Generate MOC files explicitly
qt5_wrap_cpp(MOC_FILES
  include/mavlink_gui_tester/mavlink_gui_widget.hpp
)

# Create executable
add_executable(mavlink_gui_tester_node
  src/main.cpp
  src/mavlink_gui_widget.cpp
  src/line_graph_widget.cpp
  ${MOC_FILES}
)

# Link Qt5 libraries before ament dependencies
target_link_libraries(mavlink_gui_tester_node
  Qt5::Core
  Qt5::Widgets
)

# Specify libraries to link a library or executable target against
ament_target_dependencies(mavlink_gui_tester_node
  rclcpp
  std_msgs
  geometry_msgs
  diagnostic_msgs
  rclcpp_action
  rclcpp_components
)


# Install targets
install(TARGETS
  mavlink_gui_tester_node
  DESTINATION lib/${PROJECT_NAME})

# Install include files
install(DIRECTORY include/
  DESTINATION include/)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
