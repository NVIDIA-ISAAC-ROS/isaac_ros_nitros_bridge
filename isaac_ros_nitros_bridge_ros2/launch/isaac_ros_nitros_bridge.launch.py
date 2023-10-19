# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import os
import subprocess
import time

import launch

from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch_testing.actions

ROS1_INSTALL_PATH = 'install_isolated/setup.bash'
BRIDGE_CONFIG_PATH = 'src/isaac_ros_nitros_bridge/config/nitros_bridge.yaml'


def launch_setup(context):
    rosbag_path = LaunchConfiguration('rosbag_path').perform(context)
    ros1_ws_path = LaunchConfiguration('ros1_ws_path').perform(context)

    roscore_cmd = 'source /opt/ros/noetic/setup.bash && roscore'
    subprocess.Popen(
        roscore_cmd,
        env=os.environ,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        shell=True,
        executable='/bin/bash'
    )
    # Wait one second to ensure roscore is fully launched
    time.sleep(1)

    ros1_setup_path = os.path.join(ros1_ws_path, ROS1_INSTALL_PATH)
    stop_ros_nodes_cmd = f'source {ros1_setup_path} && rosnode kill -a'
    subprocess.run(
        stop_ros_nodes_cmd,
        env=os.environ,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        shell=True,
        executable='/bin/bash'
    )

    bridge_config_absolute_path = os.path.join(ros1_ws_path, BRIDGE_CONFIG_PATH)
    ros1_converter_cmd = f'source {ros1_setup_path} \
        && rosparam load {bridge_config_absolute_path} \
        && roslaunch isaac_ros_nitros_bridge_ros1 r2b_converter.launch'
    subprocess.Popen(
        ros1_converter_cmd,
        env=os.environ,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        shell=True,
        executable='/bin/bash'
    )

    ros1_bridge_cmd = 'ros2 run ros1_bridge parameter_bridge'
    subprocess.Popen(
        ros1_bridge_cmd,
        env=os.environ,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        shell=True
    )

    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', rosbag_path, '--loop', '--remap',
             '/image:=/r2b/ros2_input_image'],
        output='screen')

    ros2_converter = ComposableNode(
        name='ros2_converter',
        namespace='r2b',
        package='isaac_ros_nitros_bridge_ros2',
        plugin='nvidia::isaac_ros::nitros_bridge::ImageConverterNode',
        remappings=[
            ('ros2_output_bridge_image', 'ros1_input_bridge_image'),
            ('ros2_input_bridge_image', 'ros1_output_bridge_image')
        ])

    container = ComposableNodeContainer(
        name='ros2_converter_container',
        namespace='r2b',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[ros2_converter],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )
    return [rosbag_play, container]


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            'rosbag_path',
            description='Path of the r2b rosbag'),
        DeclareLaunchArgument(
            'ros1_ws_path',
            description='Path of the ros1 workspace'),
    ]

    nodes = launch_args + [OpaqueFunction(function=launch_setup)]
    return launch.LaunchDescription(
        nodes + [
            # Start tests after a fixed delay for node startup
            launch.actions.TimerAction(
                period=30.0,
                actions=[launch_testing.actions.ReadyToTest()])
        ]
    )
