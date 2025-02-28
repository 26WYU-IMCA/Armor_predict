cmake_minimum_required(VERSION 3.8)
project(target_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(node_parameter REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

add_executable(camera src/camera_node.cpp)
ament_target_dependencies(
camera
"rclcpp"
"sensor_msgs"
"OpenCV"
"cv_bridge"
)

add_executable(track src/armor_track.cpp)
ament_target_dependencies(
track
"rclcpp"
"sensor_msgs"
"OpenCV"
"cv_bridge"
"node_parameter"
)


add_executable(predict src/armor_predit.cpp)
ament_target_dependencies(
predict
"rclcpp"
"sensor_msgs"
"OpenCV"
"cv_bridge"
"node_parameter"
)




add_library(target_lib STATIC include/target.cpp)
target_link_libraries(track target_lib)


add_library(kalman_lib STATIC include/kalman.cpp)
target_link_libraries(predict kalman_lib)



install(TARGETS
camera
track
predict
DESTINATION lib/${PROJECT_NAME}
)



install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})


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
