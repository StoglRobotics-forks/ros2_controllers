:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/io_gripper_controller/doc/userdoc.rst

.. _io_gripper_controller_userdoc:

io_gripper_controller
=============================

This controller implements ...

Parameters
,,,,,,,,,,,

This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters.

This controller adds the following parameters:

.. generate_parameter_library_details:: ../src/io_gripper_controller.yaml


Example parameters
....................

```
io_gripper_controller:
  ros__parameters:
    
    interface_name: gpio

    opening_ios:
      high: Greiferteil_oeffnen_WQG1
      low: Greiferteil_schliessen_WQG2

    closing_ios:
      high: Greiferteil_schliessen_WQG2
      low: Greiferteil_oeffnen_WQG1

    opened_states:
      high: Greifer_Geoeffnet_BG01_
      low: Greifer_Geschloschen_BG02_

    closed_states:
      high: Greifer_Geschloschen_BG02_
      low: Greifer_Geoeffnet_BG01_

    status_joint_name: "gripper_joint" # or how ever it is called the joint for closing
    joint_value_opened: 0.0  # see min in URDF
    joint_value_closed: 0.2  # see max in URDF

    ios_before_opening:
      high: Bremse_WQG7
      low: []