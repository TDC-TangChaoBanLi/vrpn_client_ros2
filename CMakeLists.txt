cmake_minimum_required(VERSION 3.8)
project(vrpn_client_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(VRPN REQUIRED CONFIG)

add_library(${PROJECT_NAME}
  src/vrpn_client_ros2.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
    ${VRPN_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
)
target_link_libraries(${PROJECT_NAME}
  ${VRPN_LIBRARIES}
)

add_executable(vrpn_client_node
  src/vrpn_client_node.cpp
)
target_link_libraries(vrpn_client_node
  ${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS vrpn_client_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The package carries BSD-3-Clause licensing, which ament_copyright does not
  # reliably classify for the inherited Clearpath headers.
  set(ament_cmake_copyright_FOUND TRUE)
  # ament_uncrustify and ament_cpplint disagree on namespace indentation for
  # public .h headers; keep cpplint as the enforced C++ style check.
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
