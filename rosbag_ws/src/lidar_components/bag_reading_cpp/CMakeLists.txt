cmake_minimum_required(VERSION 3.8)
project(bag_reading_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

add_executable(bag_reader src/bag_reader.cpp)
ament_target_dependencies(bag_reader rclcpp rosbag2_transport sensor_msgs rclcpp_components)

add_library(bag_spoofed_reader_component SHARED src/bag_spoofed_reader.cpp)
ament_target_dependencies(bag_spoofed_reader_component rclcpp rosbag2_transport sensor_msgs rclcpp_components rosidl_default_runtime)

rclcpp_components_register_nodes(bag_spoofed_reader_component "lidar_components::PlaybackSpoofedNode")

# add_executable(bag_spoofed_reader src/bag_spoofed_reader.cpp)
# ament_target_dependencies(bag_spoofed_reader rclcpp rosbag2_transport sensor_msgs)

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

install(TARGETS
  bag_reader
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME})

ament_export_targets(bag_spoofed_reader_component)
install(TARGETS bag_spoofed_reader_component
        EXPORT bag_spoofed_reader_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)


# install(TARGETS
#   bag_reader
#   DESTINATION lib/${PROJECT_NAME}
# )



ament_package()
