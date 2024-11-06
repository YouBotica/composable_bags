cmake_minimum_required(VERSION 3.5)
project(cloud_transform)

# set(CMAKE_BUILD_TYPE RelWithDebInfo) # FIXME: Comment this

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)
# find_package(PCL REQUIRED) # QUIET COMPONENTS common features filters io segmentation surface)
find_package(autoware_auto_perception_msgs REQUIRED)

# Include directories for PCL and Eigen3
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# find_package(pcl_conversions REQUIRED) # DEPRECATED as ament_auto_find_build_dependencies() links them on package.xml
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_components REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_eigen REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)
# find_package(tf2_ros REQUIRED)

# set(dependencies
#   EIGEN3
#   PCL
# )

# Set dependencies list
set(dependencies
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  pcl_conversions
  # PCL
  Eigen3
  autoware_auto_perception_msgs
)



ament_auto_add_executable(cloud_transform
  src/cloud_transform.cpp
)

# Create a library for the component (shared library)
ament_auto_add_library(alt_cloud_transform_component SHARED
  src/transformer_alt.cpp
)
# ament_auto_add_executable(alt_cloud_transform
#   src/transformer_alt.cpp
# )

# Link libraries to the executables
target_link_libraries(cloud_transform
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

# Link dependencies for the component
target_link_libraries(alt_cloud_transform_component
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)
# target_link_libraries(alt_cloud_transform
#   ${PCL_LIBRARIES}
#   Eigen3::Eigen
# )

rclcpp_components_register_node(alt_cloud_transform_component
  PLUGIN "cloud_transform::CloudTransformerAlt"
  EXECUTABLE alt_cloud_transform_component_exec
)

# Include directories for PCL and Eigen3
include_directories(
  ${PCL_INCLUDE_DIRS}
  Eigen3::Eigen
)

# export information to downstream packages
ament_auto_package(
  INSTALL_TO_SHARE config
)

# tests
if(AMENT_ENABLE_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  #ament_add_gtest(${PROJECT_NAME}_gtest test/my_test.cpp)
  #target_link_libraries(${PROJECT_NAME}_gtest ${rclcpp_LIBRARIES} ${rmw_connext_cpp_LIBRARIES} ${std_interfaces})

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
