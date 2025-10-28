cmake_minimum_required(VERSION 3.8)
project(my_simple_controllers)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
  
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17 -Wl,-z,defs)
endif()

set(CMAKE_VERBOSE_MAKEFILE off)

find_package(ament_cmake          REQUIRED)
set(THIS_PACKAGE_INCLUDE_DEPENDS
rclcpp              
rclcpp_lifecycle              
orocos_kdl          
controller_interface
visualization_msgs  
kdl_parser          
realtime_tools			
control_msgs       
tf2_msgs
tf2_ros
kdl_parser
pluginlib
generate_parameter_library
urdf
)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(
    my_simple_controller_parameters
    src/parameters.yaml
    )

#------------------------------- Targets ------------------------------#
# controller plugin library
add_library(${PROJECT_NAME} SHARED 
             src/state_controller.cpp
             src/cartesian_velocity_controller.cpp)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  ${GUROBI_INCLUDE_DIR}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
pluginlib_export_plugin_description_file(controller_interface plugins.xml)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${PROJECT_NAME} PUBLIC my_simple_controller_parameters)
#===================================================================================================
# Installation
#===================================================================================================
ament_export_targets(export_my_simple_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
install(TARGETS ${PROJECT_NAME} my_simple_controller_parameters
        EXPORT export_my_simple_controllers
        ARCHIVE DESTINATION lib 
        LIBRARY DESTINATION lib 
        RUNTIME DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY "include/"
  DESTINATION include
)
install(
  FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)
#===================================================================================================
# Trajectory Planner Node
add_executable(trajectory_planner src/trajectory_planner.cpp)
ament_target_dependencies(trajectory_planner rclcpp geometry_msgs)

install(TARGETS trajectory_planner
  DESTINATION lib/${PROJECT_NAME}
)
