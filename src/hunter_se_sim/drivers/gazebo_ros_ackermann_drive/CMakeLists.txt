cmake_minimum_required(VERSION 3.5)
project(gazebo_ros_ackermann_drive)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

if(WIN32)
  add_compile_definitions(
    _USE_MATH_DEFINES
    NOMINMAX
    WIN32_LEAN_AND_MEAN
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})

# gazebo_ros_ackermann_drive
add_library(gazebo_ros_ackermann_drive SHARED
  src/gazebo_ros_ackermann_drive.cpp
)
target_include_directories(gazebo_ros_ackermann_drive PUBLIC include)
ament_target_dependencies(gazebo_ros_ackermann_drive
  "gazebo_dev"
  "gazebo_ros"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "std_msgs"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
)
ament_export_libraries(gazebo_ros_ackermann_drive)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(gazebo_msgs)
ament_export_dependencies(gazebo_ros)

if(NOT WIN32)
  if(NOT APPLE)
    set(
      AMENT_CMAKE_ENVIRONMENT_HOOKS_DESC_gazebo_ros_ackermann_drive
      "prepend-non-duplicate;LD_LIBRARY_PATH;${GAZEBO_PLUGIN_PATH}")
  else()
    set(
      AMENT_CMAKE_ENVIRONMENT_HOOKS_DESC_gazebo_ros_ackermann_drive
      "prepend-non-duplicate;DYLD_LIBRARY_PATH;${GAZEBO_PLUGIN_PATH}")
  endif()
endif()
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo_ros_ackermann_drive.sh.in")

ament_package()

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
    gazebo_ros_ackermann_drive
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)