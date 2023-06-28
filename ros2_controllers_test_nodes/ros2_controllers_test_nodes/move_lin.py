# Copyright 2022 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Manuel Muth
#


import rclpy
from rclpy.node import Node

import yaml
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


class MoveLin(Node):
    def __init__(self):
        super().__init__("move_lin")
        # Declare all parameters
        self.declare_parameter("controller_name", "position_trajectory_controller")
        self.declare_parameter("wait_sec_between_publish", 5)
        self.declare_parameter("joints", [""])
        self.declare_parameter("goal_names", ["traj1", "traj2"])
        self.declare_parameter("check_starting_point", False)

        # Read parameters
        wait_sec_between_publish = self.get_parameter("wait_sec_between_publish").value
        goal_names = self.get_parameter("goal_names").value
        controller_name = self.get_parameter("controller_name").value
        self.joints = self.get_parameter("joints").value
        self.check_starting_point = self.get_parameter("check_starting_point").value
        self.starting_point = {}
        self.trajectories = []

        # starting point stuff
        if self.check_starting_point:
            # declare nested params
            for name in self.joints:
                param_name_tmp = "starting_point_limits" + "." + name
                self.declare_parameter(param_name_tmp, [-2 * 3.14159, 2 * 3.14159])
                self.starting_point[name] = self.get_parameter(param_name_tmp).value

            for name in self.joints:
                if len(self.starting_point[name]) != 2:
                    raise Exception('"starting_point" parameter is not set correctly!')
            self.joint_state_sub = self.create_subscription(
                JointState, "joint_states", self.joint_state_callback, 10
            )
        # initialize starting point status
        self.starting_point_ok = not self.check_starting_point

        self.joint_state_msg_received = False

        # Read all positions from parameters
        self.goals = ""
        for name in goal_names:
            self.declare_parameter(name)
            goal = self.get_parameter(name).value
            if goal is None:
                raise Exception(f'Values for goal "{name}" not set!')

            print(f"goal path:{goal}")
            traj_msg = self.get_traj(goal)
            self.trajectories.append(traj_msg)
            # float_goal = [float(value) for value in goal]
            # self.goals.append(float_goal)

        publish_topic = "/" + controller_name + "/" + "joint_trajectory"
        self.get_logger().info(
            f'Publishing {len(goal_names)} goals on topic "{publish_topic}"\
              every {wait_sec_between_publish} s'
        )

        self.get_logger().info(f"Publishing on topic:{publish_topic}")
        self.publisher_ = self.create_publisher(JointTrajectory, publish_topic, 1)
        self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)
        self.i = 0

    def timer_callback(self):

        if self.starting_point_ok:

            self.get_logger().info(
                f"Sending trajectory for joints{self.trajectories[self.i].joint_names}."
            )

            self.publisher_.publish(self.trajectories[self.i])

            self.i += 1
            self.i %= len(self.trajectories)

        elif self.check_starting_point and not self.joint_state_msg_received:
            self.get_logger().warn(
                'Start configuration could not be checked! Check "joint_state" topic!'
            )
        else:
            self.get_logger().warn("Start configuration is not within configured limits!")

    def get_traj(self, traj_path):
        with open(traj_path) as file:
            file_node = yaml.load(file, Loader=yaml.FullLoader)

        traj_msg = JointTrajectory()
        traj_node = file_node["trajectory"]

        traj_msg.header = Header()
        traj_msg.header.frame_id = traj_node["header"]["frame_id"]
        traj_msg.header.stamp.sec = traj_node["header"]["stamp"]["secs"]
        traj_msg.header.stamp.nanosec = traj_node["header"]["stamp"]["nanosec"]

        for joint_name in traj_node["joint_names"]:
            traj_msg.joint_names.append(joint_name)

        for i in range(len(traj_node["points"])):
            point_node = traj_node["points"][i]

            point = JointTrajectoryPoint()
            point.positions = point_node["positions"]
            point.velocities = point_node["velocities"]
            point.accelerations = point_node["accelerations"]
            point.effort = point_node["effort"]

            point.time_from_start = Duration()
            point.time_from_start.sec = point_node["time_from_start"]["secs"]
            point.time_from_start.nanosec = point_node["time_from_start"]["nanosec"]

            traj_msg.points.append(point)

        return traj_msg


def main(args=None):
    rclpy.init(args=args)

    publisher_forward_position = MoveLin()

    rclpy.spin(publisher_forward_position)
    publisher_forward_position.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
