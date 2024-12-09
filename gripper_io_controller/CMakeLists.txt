cmake_minimum_required(VERSION 3.8)
project(gripper_io_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  sensor_msgs
  controller_manager
  ros2_control_test_assets
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
  ament_cmake
  generate_parameter_library
  ament_cmake_gmock
  std_msgs
  control_msgs
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add gripper_io_controller library related compile commands
generate_parameter_library(gripper_io_controller_parameters
  src/gripper_io_controller.yaml
)
add_library(
  gripper_io_controller
  SHARED
  src/gripper_io_controller.cpp
)
target_include_directories(gripper_io_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(gripper_io_controller gripper_io_controller_parameters)
ament_target_dependencies(gripper_io_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(gripper_io_controller PRIVATE "GRIPPER_IO_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface gripper_io_controller.xml)

install(
  TARGETS
  gripper_io_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)

  ament_add_gmock(test_load_gripper_io_controller test/test_load_gripper_io_controller.cpp)
  target_include_directories(test_load_gripper_io_controller PRIVATE include)
  ament_target_dependencies(
    test_load_gripper_io_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  ament_add_gmock(test_gripper_io_controller test/test_gripper_io_controller.cpp)
  target_include_directories(test_gripper_io_controller PRIVATE include)
  target_link_libraries(test_gripper_io_controller gripper_io_controller)
  ament_target_dependencies(
    test_gripper_io_controller
    controller_interface
    hardware_interface
  )

endif()

ament_export_include_directories(
  include
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_libraries(
  gripper_io_controller
)

ament_package()
