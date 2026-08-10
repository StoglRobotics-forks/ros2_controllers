cmake_minimum_required(VERSION 3.16)
project(joint_trajectory_controller)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  angles
  backward_ros
  control_msgs
  control_toolbox
  controller_interface
  generate_parameter_library
  hardware_interface
  joint_limits
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  realtime_tools
  rsl
  ruckig
  tf2
  tf2_geometry_msgs
  trajectory_msgs
  urdf
  kinematics_interface
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(joint_trajectory_controller_parameters
  src/joint_trajectory_controller_parameters.yaml
  include/joint_trajectory_controller/validate_jtc_parameters.hpp
)

add_library(joint_trajectory_controller SHARED
  src/joint_trajectory_controller.cpp
  src/trajectory.cpp
)
target_compile_features(joint_trajectory_controller PUBLIC cxx_std_17)
if(WIN32)
  target_compile_definitions(joint_trajectory_controller PRIVATE _USE_MATH_DEFINES)
endif()
target_include_directories(joint_trajectory_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/joint_trajectory_controller>
)
target_link_libraries(joint_trajectory_controller PUBLIC
                      joint_trajectory_controller_parameters
                      control_toolbox::control_toolbox
                      controller_interface::controller_interface
                      hardware_interface::hardware_interface
                      pluginlib::pluginlib
                      rclcpp::rclcpp
                      rclcpp_action::rclcpp_action
                      rclcpp_lifecycle::rclcpp_lifecycle
                      realtime_tools::realtime_tools
                      rsl::rsl
                      ruckig::ruckig
                      urdf::urdf
                      angles::angles
                      kinematics_interface::kinematics_interface
                      ${trajectory_msgs_TARGETS}
                      ${control_msgs_TARGETS})

pluginlib_export_plugin_description_file(controller_interface joint_trajectory_plugin.xml)

# NOTE(rebase): cartesian_trajectory_generator subclasses JointTrajectoryController, so it's built
# as its own shared library on top of joint_trajectory_controller rather than folded into it.
# Uses the same modern target-based linking as joint_trajectory_controller above, rather than the
# original commit's ament_target_dependencies() (superseded by jazzy's target-based conventions).
add_library(cartesian_trajectory_generator SHARED
  src/cartesian_trajectory_generator.cpp
)
target_compile_features(cartesian_trajectory_generator PUBLIC cxx_std_17)
target_include_directories(cartesian_trajectory_generator PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/joint_trajectory_controller>
)
target_link_libraries(cartesian_trajectory_generator PUBLIC
                      joint_trajectory_controller
                      tf2::tf2
                      tf2_geometry_msgs::tf2_geometry_msgs
                      ${nav_msgs_TARGETS})

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_trajectory test/test_trajectory.cpp)
  target_link_libraries(test_trajectory
    joint_trajectory_controller
    ros2_control_test_assets::ros2_control_test_assets
  )
  target_compile_definitions(test_trajectory PRIVATE _USE_MATH_DEFINES)

  ament_add_gmock(test_tolerances test/test_tolerances.cpp)
  target_link_libraries(test_tolerances
    joint_trajectory_controller
    ros2_control_test_assets::ros2_control_test_assets
  )

  ament_add_gmock(test_trajectory_controller
    test/test_trajectory_controller.cpp)
  set_tests_properties(test_trajectory_controller PROPERTIES TIMEOUT 220)
  target_link_libraries(test_trajectory_controller
    joint_trajectory_controller
    ros2_control_test_assets::ros2_control_test_assets
  )
  target_compile_definitions(test_trajectory_controller PRIVATE _USE_MATH_DEFINES)

  add_definitions(-DTEST_FILES_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test")
  ament_add_gmock(test_load_joint_trajectory_controller
    test/test_load_joint_trajectory_controller.cpp
  )
  target_link_libraries(test_load_joint_trajectory_controller
    joint_trajectory_controller
    controller_manager::controller_manager
    ros2_control_test_assets::ros2_control_test_assets
  )

  ament_add_gmock(test_trajectory_actions
    test/test_trajectory_actions.cpp
  )
  set_tests_properties(test_trajectory_actions PROPERTIES TIMEOUT 330)
  target_link_libraries(test_trajectory_actions
    joint_trajectory_controller
    ros2_control_test_assets::ros2_control_test_assets)
endif()


install(
  DIRECTORY include/
  DESTINATION include/joint_trajectory_controller
)
install(TARGETS
    joint_trajectory_controller
    joint_trajectory_controller_parameters
    cartesian_trajectory_generator
  EXPORT export_joint_trajectory_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_joint_trajectory_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
