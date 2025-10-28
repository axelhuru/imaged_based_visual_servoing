cmake_minimum_required(VERSION 3.8)
project(visual_servoing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)  
find_package(image_transport REQUIRED)  
find_package(geometry_msgs REQUIRED)

add_executable(ball_detector src/ball_detector.cpp)

ament_target_dependencies(
  ball_detector
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport  
  geometry_msgs
  OpenCV
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  ball_detector
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
