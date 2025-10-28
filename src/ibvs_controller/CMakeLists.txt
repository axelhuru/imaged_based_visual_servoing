cmake_minimum_required(VERSION 3.8)
project(ibvs_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(VISP REQUIRED)

add_executable(ibvs_controller src/ibvs_controller.cpp)

ament_target_dependencies(
  ibvs_controller
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  geometry_msgs
  OpenCV
  VISP
)

target_include_directories(ibvs_controller PUBLIC ${VISP_INCLUDE_DIRS})
target_link_libraries(ibvs_controller ${VISP_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  ibvs_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
